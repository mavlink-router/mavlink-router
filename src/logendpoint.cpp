/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "logendpoint.h"

#include <dirent.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <common/log.h>
#include <common/util.h>

#include "mainloop.h"

#define ALIVE_TIMEOUT 5
#define MAX_RETRIES   10

// clang-format off
const ConfFile::OptionsTable LogEndpoint::option_table[] = {
    {"Log",             false, ConfFile::parse_stdstring,           OPTIONS_TABLE_STRUCT_FIELD(LogOptions, logs_dir)},
    {"LogMode",         false, LogEndpoint::parse_log_mode,         OPTIONS_TABLE_STRUCT_FIELD(LogOptions, log_mode)},
    {"MavlinkDialect",  false, LogEndpoint::parse_mavlink_dialect,  OPTIONS_TABLE_STRUCT_FIELD(LogOptions, mavlink_dialect)},
    {"MinFreeSpace",    false, ConfFile::parse_ul,                  OPTIONS_TABLE_STRUCT_FIELD(LogOptions, min_free_space)},
    {"MaxLogFiles",     false, ConfFile::parse_ul,                  OPTIONS_TABLE_STRUCT_FIELD(LogOptions, max_log_files)},
    {"LogWriteQueueSize", false, ConfFile::parse_ul,                OPTIONS_TABLE_STRUCT_FIELD(LogOptions, write_queue_size)},
    {"LogSystemId",     false, LogEndpoint::parse_fcu_id,           OPTIONS_TABLE_STRUCT_FIELD(LogOptions, fcu_id)},
    {"LogTelemetry",    false, ConfFile::parse_bool,                OPTIONS_TABLE_STRUCT_FIELD(LogOptions, log_telemetry)},
    {}
};
// clang-format on

LogEndpoint::LogEndpoint(std::string name, LogOptions conf)
    : Endpoint{ENDPOINT_TYPE_LOG, std::move(name)}
    , _config{conf}
{
    assert(!_config.logs_dir.empty());
    _add_sys_comp_id(LOG_ENDPOINT_SYSTEM_ID, 0);
    if (_config.fcu_id != -1) {
        _target_system_id = _config.fcu_id;
    } else {
        _target_system_id = -1;
    }

    _writer = std::thread(&LogEndpoint::_writer_loop, this);
}

LogEndpoint::~LogEndpoint()
{
    {
        std::lock_guard<std::mutex> lock(_writer_mutex);
        _writer_exit = true;
    }
    _writer_cv.notify_one();
    _writer.join();
}

ssize_t LogEndpoint::_write(const void *data, size_t len, off_t offset)
{
    {
        std::lock_guard<std::mutex> lock(_writer_mutex);
        if (_writer_bytes + len > _config.write_queue_size) {
            if (!_writer_full) {
                log_warning("Log write queue full, dropping log data until the disk catches up");
                _writer_full = true;
            }
            errno = EAGAIN;
            return -1;
        }
        _writer_full = false;
    }

    const auto *bytes = static_cast<const uint8_t *>(data);
    _queue_job({WriteJob::Write, _file, offset, std::vector<uint8_t>(bytes, bytes + len)});
    return len;
}

void LogEndpoint::_queue_job(WriteJob job)
{
    {
        std::lock_guard<std::mutex> lock(_writer_mutex);
        _writer_bytes += job.data.size();
        _writer_jobs.push_back(std::move(job));
    }
    _writer_cv.notify_one();
}

void LogEndpoint::_writer_loop()
{
    // SIGINT and SIGTERM must interrupt the main loop's epoll_wait(), so this thread doesn't take them
    sigset_t signals;
    sigemptyset(&signals);
    sigaddset(&signals, SIGINT);
    sigaddset(&signals, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &signals, nullptr);

    bool failing = false;
    std::unique_lock<std::mutex> lock(_writer_mutex);
    while (true) {
        _writer_cv.wait(lock, [this] { return _writer_exit || !_writer_jobs.empty(); });
        if (_writer_jobs.empty()) {
            return; // exit requested and everything is written
        }
        WriteJob job = std::move(_writer_jobs.front());
        _writer_jobs.pop_front();
        lock.unlock();

        bool ok;
        if (job.kind == WriteJob::Write) {
            const ssize_t r = job.offset < 0
                ? write(job.fd, job.data.data(), job.data.size())
                : pwrite(job.fd, job.data.data(), job.data.size(), job.offset);
            ok = r == (ssize_t)job.data.size();
        } else {
            ok = fsync(job.fd) == 0;
            if (job.kind == WriteJob::Close) {
                close(job.fd);
            }
        }
        // Log only the first of a series of errors, e.g. when the disk is full
        if (!ok && !failing) {
            log_error("Unable to write log file: (%m)");
        }
        failing = !ok;

        lock.lock();
        _writer_bytes -= job.data.size();
    }
}

void LogEndpoint::_send_msg(const mavlink_message_t *msg, int target_sysid)
{
    uint8_t data[MAVLINK_MAX_PACKET_LEN] = {};
    struct buffer buffer = {};

    buffer.data = data;
    buffer.len = mavlink_msg_to_send_buffer(data, msg);
    buffer.curr.msg_id = msg->msgid;
    buffer.curr.target_sysid = target_sysid;
    buffer.curr.target_compid = MAV_COMP_ID_ALL;
    buffer.curr.src_sysid = msg->sysid;
    buffer.curr.src_compid = msg->compid;
    /* don't bother with it as it's only used by Log backends */
    buffer.curr.payload_len = 0;

    Mainloop::get_instance().route_msg(&buffer);

    _stat.read.total++;
    _stat.read.handled++;
    _stat.read.handled_bytes += buffer.len;
}

void LogEndpoint::mark_unfinished_logs()
{
    DIR *dir = opendir(_config.logs_dir.c_str());

    // Assume the directory does not exist if opendir failed
    if (!dir) {
        return;
    }

    struct dirent *ent;
    uint32_t u;

    while ((ent = readdir(dir)) != nullptr) {
        if (sscanf(ent->d_name, "%u-", &u) != 1) {
            continue;
        }

        char log_file[PATH_MAX];
        struct stat file_stat;
        if (snprintf(log_file, sizeof(log_file), "%s/%s", _config.logs_dir.c_str(), ent->d_name)
            >= (int)sizeof(log_file)) {
            continue;
        }

        if (stat(log_file, &file_stat)) {
            continue;
        }

        if (S_ISREG(file_stat.st_mode) && (file_stat.st_mode & S_IWUSR)) {
            log_info("File %s not read-only yet, marking as RO", ent->d_name);
            chmod(log_file, S_IRUSR | S_IRGRP | S_IROTH);
        }
    }
    closedir(dir);
}

void LogEndpoint::_delete_old_logs()
{
    struct statvfs buf;
    uint64_t free_space;

    if (statvfs(_config.logs_dir.c_str(), &buf) == 0) {
        free_space = (uint64_t)buf.f_bsize * buf.f_bavail;
    } else if (errno == ENOENT) {
        // Ignore error - we don't have any logs to delete if directory
        // doesn't exist
        return;
    } else {
        free_space = UINT64_MAX;
        log_error("[Log Deletion] Error when measuring free disk space: %m");
    }

    log_debug("[Log Deletion]  Total free space: %" PRIu64 "MB. Min free space: %luMB",
              free_space / (1ul << 20),
              _config.min_free_space / (1ul << 20));

    // This check is not necessary, it just saves on some file IO.
    if (free_space > _config.min_free_space && _config.max_log_files == 0) {
        return;
    }

    DIR *dir = opendir(_config.logs_dir.c_str());

    // Assume the directory does not exist if opendir failed
    if (!dir) {
        return;
    }

    int dir_fd = dirfd(dir);
    if (dir_fd < 0) {
        closedir(dir);
        log_error("[Log Deletion] Error getting dir file descriptor: %m");
        return;
    }

    // Map of index -> (filename, file size)
    // All three of these values are saved for later to reduce the amount of IO operations needed.
    std::map<unsigned long, std::tuple<std::string, unsigned long>> file_map;

    struct dirent *ent;
    uint32_t idx, year, month, day, hour, minute, second;

    // This loop just gathers the name, index, and size of each read-only file in the log dir
    while ((ent = readdir(dir)) != nullptr) {
        // Even though we don't need the timestamp, we want to match as much of the filename as
        // possible, so we don't accidentally delete something that isn't a log.
        if (sscanf(ent->d_name,
                   "%u-%u-%u-%u_%u-%u-%u.",
                   &idx,
                   &year,
                   &month,
                   &day,
                   &hour,
                   &minute,
                   &second)
            == 7) {
            struct stat file_stat;
            if (!fstatat(dir_fd, ent->d_name, &file_stat, 0)) {
                // Only bother with read-only files. If this function is somehow called while a file
                // is still being used (not read-only), it should not be deleted.
                if (!S_ISDIR(file_stat.st_mode) && !(file_stat.st_mode & S_IWUSR)) {
                    std::string str(ent->d_name);
                    file_map[idx] = std::make_tuple(str, file_stat.st_size);
                }
            }
        }
    }

    // If the configured value for min_free_space is 0, then we don't have to do anything special.
    int64_t bytes_to_delete = _config.min_free_space - free_space;
    // If the configured value for max_log_files is 0, then set this to -1 to indicate that we've
    // already deleted enough files.
    ssize_t files_to_delete
        = _config.max_log_files > 0 ? (ssize_t)file_map.size() - _config.max_log_files : -1;

    log_debug("[Log Deletion] Files to delete: %zd", files_to_delete);

    // Delete the logs in order until there's enough free space, and few enough files
    // It is possible for this loop to run only once and return immediately, if we don't actually
    // need to delete any files. Maps in C++ are ordered by the keys, so this iteration is
    // guaranteed to happen in index order (oldest -> newest)
    for (auto &pair : file_map) {
        if (bytes_to_delete <= 0 && files_to_delete <= 0) {
            break;
        }

        std::string &filename = std::get<0>(pair.second);
        const unsigned long filesize = std::get<1>(pair.second);
        char log_file[PATH_MAX];
        if (snprintf(log_file,
                     sizeof(log_file),
                     "%s/%s",
                     _config.logs_dir.c_str(),
                     filename.c_str())
            >= (int)sizeof(log_file)) {
            log_error("Directory + filename %s is longer than PATH_MAX of %d",
                      filename.c_str(),
                      PATH_MAX);
            continue;
        }

        int err_code = remove(log_file);
        if (err_code == 0) {
            bytes_to_delete -= filesize;
            files_to_delete--;
            log_info("[Log Deletion] Deleted old logfile %s", filename.c_str());
        } else {
            log_error("[Log Deletion] Error deleting old logfile %s: %m", filename.c_str());
        }
    }

    if (bytes_to_delete > 0) {
        log_error(
            "[Log Deletion] Deleted all closed logs, but there is still not enough free space.");
    }

    closedir(dir);
}

uint32_t LogEndpoint::_get_prefix(DIR *dir)
{
    struct dirent *ent;
    uint32_t u, prefix = 0;

    while ((ent = readdir(dir)) != nullptr) {
        if (sscanf(ent->d_name, "%u-", &u) == 1) {
            if (u >= prefix && u < UINT32_MAX) {
                prefix = ++u;
            }
        }
    }

    return prefix;
}

DIR *LogEndpoint::_open_or_create_dir(const char *name)
{
    int r;
    DIR *dir = opendir(name);

    // If failed because dir doesn't exist, try to create it
    if (!dir && errno == ENOENT) {
        r = mkdir_p(name, strlen(name), 0755);
        if (r < 0) {
            errno = -r;
            return nullptr;
        }
        dir = opendir(name);
    }

    return dir;
}

int LogEndpoint::_get_file(const char *extension)
{
    time_t t = time(nullptr);
    struct tm *timeinfo = localtime(&t);
    uint32_t i;
    int j, r;
    DIR *dir;
    int dir_fd;

    dir = _open_or_create_dir(_config.logs_dir.c_str());
    if (!dir) {
        log_error("Could not open log dir (%m)");
        return -1;
    }
    // Close dir when leaving function.
    std::shared_ptr<void> defer(dir, [](DIR *p) { closedir(p); });

    i = _get_prefix(dir);
    dir_fd = dirfd(dir);

    for (j = 0; j <= MAX_RETRIES; j++) {
        r = snprintf(_filename,
                     sizeof(_filename),
                     "%05u-%i-%02i-%02i_%02i-%02i-%02i.%s",
                     i + j,
                     timeinfo->tm_year + 1900,
                     timeinfo->tm_mon + 1,
                     timeinfo->tm_mday,
                     timeinfo->tm_hour,
                     timeinfo->tm_min,
                     timeinfo->tm_sec,
                     extension);

        if (r < 1 || (size_t)r >= sizeof(_filename)) {
            log_error("Error formatting Log file name: (%m)");
            return -1;
        }

        r = openat(dir_fd, _filename, O_WRONLY | O_CLOEXEC | O_CREAT | O_NONBLOCK | O_EXCL, 0644);
        if (r < 0) {
            if (errno != EEXIST) {
                log_error("Unable to open Log file(%s): (%m)", _filename);
                return -1;
            }
            continue;
        }

        // Ensure the directory entry of the file is written to disk
        const int dir_copy = dup(dir_fd);
        if (dir_copy >= 0) {
            _queue_job({WriteJob::Close, dir_copy, -1, {}});
        }

        return r;
    }

    log_error("Unable to create a Log file without override another file.");
    return -EEXIST;
}

void LogEndpoint::stop()
{
    Mainloop &mainloop = Mainloop::get_instance();
    if (_timeout.logging_start) {
        mainloop.del_timeout(_timeout.logging_start);
        _timeout.logging_start = nullptr;
    }

    if (_timeout.alive) {
        mainloop.del_timeout(_timeout.alive);
        _timeout.alive = nullptr;
    }

    if (_timeout.fsync) {
        mainloop.del_timeout(_timeout.fsync);
        _timeout.fsync = nullptr;
    }

    // The writer thread closes the file after writing everything queued for it
    _queue_job({WriteJob::Close, _file, -1, {}});
    _file = -1;

    // change file permissions to read-only to mark them as finished
    char log_file[PATH_MAX];
    if (snprintf(log_file, sizeof(log_file), "%s/%s", _config.logs_dir.c_str(), _filename)
        < (int)sizeof(log_file)) {
        chmod(log_file, S_IRUSR | S_IRGRP | S_IROTH);
    }
}

bool LogEndpoint::start()
{
    if (_file != -1) {
        log_warning("Log already started");
        return false;
    }

    // Clear up space before opening a new file
    _delete_old_logs();

    _file = _get_file(_get_logfile_extension());
    if (_file < 0) {
        _file = -1;
        return false;
    }

    _timeout.logging_start = Mainloop::get_instance().add_timeout(
        MSEC_PER_SEC,
        std::bind(&LogEndpoint::_logging_start_timeout, this),
        this);
    if (!_timeout.logging_start) {
        log_error("Unable to add timeout");
        goto logging_timeout_error;
    }

    // Call fsync once per second
    _timeout.fsync = Mainloop::get_instance().add_timeout(MSEC_PER_SEC,
                                                          std::bind(&LogEndpoint::_fsync, this),
                                                          this);
    if (!_timeout.fsync) {
        log_error("Unable to add timeout");
        goto fsync_timeout_error;
    }

    log_info("Logging target system_id=%u on %s", _target_system_id, _filename);

    return true;

fsync_timeout_error:
    Mainloop::get_instance().del_timeout(_timeout.logging_start);
    _timeout.logging_start = nullptr;
logging_timeout_error:
    close(_file);
    _file = -1;
    return false;
}

bool LogEndpoint::_alive_timeout()
{
    if (_timeout_write_total == _stat.write.total) {
        log_warning("No Log messages received in %u seconds restarting Log...", ALIVE_TIMEOUT);
        stop();
        start();
    }

    _timeout_write_total = _stat.write.total;
    return true;
}

bool LogEndpoint::_fsync()
{
    if (_file < 0) {
        return false;
    }

    _queue_job({WriteJob::Fsync, _file, -1, {}});
    return true;
}

void LogEndpoint::_remove_logging_start_timeout()
{
    Mainloop::get_instance().del_timeout(_timeout.logging_start);
    _timeout.logging_start = nullptr;
}

bool LogEndpoint::_start_alive_timeout()
{
    _timeout.alive
        = Mainloop::get_instance().add_timeout(MSEC_PER_SEC * ALIVE_TIMEOUT,
                                               std::bind(&LogEndpoint::_alive_timeout, this),
                                               this);
    return !!_timeout.alive;
}

void LogEndpoint::_handle_auto_start_stop(const struct buffer *pbuf)
{
    // wait until initialized
    if (_target_system_id == -1) {
        return;
    }

    if (_config.log_mode == LogMode::always) {
        if (_file == -1) {
            if (!start()) {
                _config.log_mode = LogMode::disabled;
            }
        }

        return;
    }

    if (_config.log_mode == LogMode::while_armed) {
        if (pbuf->curr.msg_id == MAVLINK_MSG_ID_HEARTBEAT
            && pbuf->curr.src_sysid == _target_system_id
            && pbuf->curr.src_compid == MAV_COMP_ID_AUTOPILOT1) {

            const mavlink_heartbeat_t *heartbeat = (mavlink_heartbeat_t *)pbuf->curr.payload;
            const bool is_armed = heartbeat->base_mode & MAV_MODE_FLAG_SAFETY_ARMED;

            if (_file == -1 && is_armed) {
                if (!start()) {
                    _config.log_mode = LogMode::disabled;
                }
            } else if (_file != -1 && !is_armed) {
                stop();
            }
        }
    }
}

int LogEndpoint::parse_mavlink_dialect(const char *val, size_t val_len, void *storage,
                                       size_t storage_len)
{
    assert(val);
    assert(storage);
    assert(val_len);

    auto *dialect = (LogOptions::MavDialect *)storage;

    if (storage_len < sizeof(LogOptions::mavlink_dialect)) {
        return -ENOBUFS;
    }
    if (val_len > INT_MAX) {
        return -EINVAL;
    }

    if (memcaseeq(val, val_len, "auto", sizeof("auto") - 1)) {
        *dialect = LogOptions::MavDialect::Auto;
    } else if (memcaseeq(val, val_len, "common", sizeof("common") - 1)) {
        *dialect = LogOptions::MavDialect::Common;
    } else if (memcaseeq(val, val_len, "ardupilotmega", sizeof("ardupilotmega") - 1)) {
        *dialect = LogOptions::MavDialect::Ardupilotmega;
    } else {
        log_error("Invalid argument for MavlinkDialect = %.*s", (int)val_len, val);
        return -EINVAL;
    }

    return 0;
}

#define MAX_LOG_MODE_SIZE 20
int LogEndpoint::parse_log_mode(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    assert(val);
    assert(storage);
    assert(val_len);

    if (storage_len < sizeof(LogOptions::log_mode)) {
        return -ENOBUFS;
    }
    if (val_len > MAX_LOG_MODE_SIZE) {
        return -EINVAL;
    }

    const char *log_mode_str = strndupa(val, val_len);
    LogMode log_mode;
    if (strcaseeq(log_mode_str, "always")) {
        log_mode = LogMode::always;
    } else if (strcaseeq(log_mode_str, "while-armed")) {
        log_mode = LogMode::while_armed;
    } else {
        log_error("Invalid argument for LogMode = %s", log_mode_str);
        return -EINVAL;
    }
    *((LogMode *)storage) = log_mode;

    return 0;
}
#undef MAX_LOG_MODE_SIZE

int LogEndpoint::parse_fcu_id(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    const int i_ret = ConfFile::parse_i(val, val_len, storage, storage_len);
    if (i_ret != 0) {
        return i_ret;
    }

    if (*(int *)storage > 255 || *(int *)storage <= 0) {
        log_error("Invalid argument for FcuId = %.*s, should be in [0, 255]", (int)val_len, val);
        return -EINVAL;
    }

    return 0;
}
