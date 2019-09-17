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
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <memory>

#include <common/log.h>
#include <common/util.h>

#include "mainloop.h"

#define ALIVE_TIMEOUT 5
#define MAX_RETRIES 10

LogEndpoint::LogEndpoint(const char *name, const char *logs_dir, LogMode mode)
    : Endpoint {name}
    , _logs_dir {logs_dir}
    , _mode(mode)
{
    assert(_logs_dir);
    _add_sys_comp_id(LOG_ENDPOINT_SYSTEM_ID << 8);
    _fsync_cb.aio_fildes = -1;

    aioinit aio_init_data {};
    aio_init_data.aio_threads = 1;
    aio_init_data.aio_num = 1;
    aio_init_data.aio_idle_time = 3; // make sure to keep the thread running
    aio_init(&aio_init_data);
}

void LogEndpoint::_send_msg(const mavlink_message_t *msg, int target_sysid)
{
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    struct buffer buffer {
        0, data
    };

    buffer.len = mavlink_msg_to_send_buffer(data, msg);
    Mainloop::get_instance().route_msg(&buffer, target_sysid, MAV_COMP_ID_ALL, msg->sysid,
                                       msg->compid);

    _stat.read.total++;
    _stat.read.handled++;
    _stat.read.handled_bytes += buffer.len;
}

void LogEndpoint::mark_unfinished_logs()
{
    DIR *dir = opendir(_logs_dir);

    // Assume the directory does not exist if opendir failed
    if (!dir) {
        return;
    }

    struct dirent *ent;
    uint32_t u;

    while ((ent = readdir(dir)) != nullptr) {
        if (sscanf(ent->d_name, "%u-", &u) != 1)
            continue;

        char log_file[PATH_MAX];
        struct stat file_stat;
        if (snprintf(log_file, sizeof(log_file), "%s/%s", _logs_dir, ent->d_name)
            >= (int)sizeof(log_file))
            continue;

        if (stat(log_file, &file_stat))
            continue;

        if (S_ISREG(file_stat.st_mode) && (file_stat.st_mode & S_IWUSR)) {
            log_info("File %s not read-only yet, marking as RO", ent->d_name);
            chmod(log_file, S_IRUSR | S_IRGRP | S_IROTH);
        }
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
            return NULL;
        }
        dir = opendir(name);
    }

    return dir;
}

int LogEndpoint::_get_file(const char *extension)
{
    time_t t = time(NULL);
    struct tm *timeinfo = localtime(&t);
    uint32_t i;
    int j, r;
    DIR *dir;
    int dir_fd;

    dir = _open_or_create_dir(_logs_dir);
    if (!dir) {
        log_error("Could not open log dir (%m)");
        return -1;
    }
    // Close dir when leaving function.
    std::shared_ptr<void> defer(dir, [](DIR *p) { closedir(p); });

    i = _get_prefix(dir);
    dir_fd = dirfd(dir);

    for (j = 0; j <= MAX_RETRIES; j++) {
        r = snprintf(_filename, sizeof(_filename), "%05u-%i-%02i-%02i_%02i-%02i-%02i.%s", i + j,
                     timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                     timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, extension);

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
        if (fsync(dir_fd) == -1) {
            log_error("fsync failed: %m");
        }

        return r;
    }

    log_error("Unable to create a Log file without override another file.");
    return -EEXIST;
}

void LogEndpoint::stop()
{
    Mainloop &mainloop = Mainloop::get_instance();
    if (_logging_start_timeout) {
        mainloop.del_timeout(_logging_start_timeout);
        _logging_start_timeout = nullptr;
    }

    if (_alive_check_timeout) {
        mainloop.del_timeout(_alive_check_timeout);
        _alive_check_timeout = nullptr;
    }

    if (_fsync_timeout) {
        mainloop.del_timeout(_fsync_timeout);
        _fsync_timeout = nullptr;
    }

    fsync(_file);
    close(_file);
    _file = -1;
    _fsync_cb.aio_fildes = -1;

    // change file permissions to read-only to mark them as finished
    char log_file[PATH_MAX];
    if (snprintf(log_file, sizeof(log_file), "%s/%s", _logs_dir, _filename) < (int)sizeof(log_file)) {
        chmod(log_file, S_IRUSR|S_IRGRP|S_IROTH);
    }
}

bool LogEndpoint::start()
{
    if (_file != -1) {
        log_warning("Log already started");
        return false;
    }

    _file = _get_file(_get_logfile_extension());
    if (_file < 0) {
        _file = -1;
        return false;
    }

    _logging_start_timeout = Mainloop::get_instance().add_timeout(
        MSEC_PER_SEC, std::bind(&LogEndpoint::_start_timeout, this), this);
    if (!_logging_start_timeout) {
        log_error("Unable to add timeout");
        goto timeout_error;
    }

    // Call fsync once per second
    _fsync_timeout = Mainloop::get_instance().add_timeout(
        MSEC_PER_SEC, std::bind(&LogEndpoint::_fsync, this), this);
    if (!_fsync_timeout) {
        log_error("Unable to add timeout");
        goto timeout_error;
    }

    log_info("Logging target system_id=%u on %s", _target_system_id, _filename);

    return true;

timeout_error:
    if (_logging_start_timeout) {
        Mainloop::get_instance().del_timeout(_fsync_timeout);
        _fsync_timeout = nullptr;
    }

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

    if (_fsync_cb.aio_fildes >= 0 && aio_error(&_fsync_cb) == EINPROGRESS) {
        // previous operation is still in progress
        return true;
    }
    _fsync_cb.aio_fildes = _file;
    _fsync_cb.aio_sigevent.sigev_notify = SIGEV_NONE;

    aio_fsync(O_SYNC, &_fsync_cb);

    return true;
}

void LogEndpoint::_remove_start_timeout()
{
    Mainloop::get_instance().del_timeout(_logging_start_timeout);
    _logging_start_timeout = nullptr;
}

bool LogEndpoint::_start_alive_timeout()
{
    _alive_check_timeout = Mainloop::get_instance().add_timeout(
        MSEC_PER_SEC * ALIVE_TIMEOUT, std::bind(&LogEndpoint::_alive_timeout, this), this);
    return !!_alive_check_timeout;
}

void LogEndpoint::_handle_auto_start_stop(uint32_t msg_id, uint8_t source_system_id,
        uint8_t source_component_id, uint8_t *payload)
{
    if (_target_system_id == -1) { // wait until initialized
        return;
    }
    if (_mode == LogMode::always) {
        if (_file == -1) {
            if (!start()) _mode = LogMode::disabled;
        }
    } else if (_mode == LogMode::while_armed) {
        if (msg_id == MAVLINK_MSG_ID_HEARTBEAT && source_system_id == _target_system_id
            && source_component_id == MAV_COMP_ID_AUTOPILOT1) {

            const mavlink_heartbeat_t *heartbeat = (mavlink_heartbeat_t *)payload;
            const bool is_armed = heartbeat->system_status == MAV_STATE_ACTIVE;

            if (_file == -1 && is_armed) {
                if (!start()) _mode = LogMode::disabled;
            } else if (_file != -1 && !is_armed) {
                stop();
            }
        }
    }
}
