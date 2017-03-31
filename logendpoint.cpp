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

#include "log.h"
#include "mainloop.h"
#include "util.h"

#define ALIVE_TIMEOUT 5
#define MAX_RETRIES 10

void LogEndpoint::_send_msg(const mavlink_message_t *msg, int target_sysid)
{
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    struct buffer buffer {
        0, data
    };

    buffer.len = mavlink_msg_to_send_buffer(data, msg);
    _mainloop->route_msg(&buffer, target_sysid, msg->sysid);

    _stat.read.total++;
    _stat.read.handled++;
    _stat.read.handled_bytes += buffer.len;
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

int LogEndpoint::_get_file(const char *extension, char *filename, size_t filename_size)
{
    time_t t = time(NULL);
    struct tm *timeinfo = localtime(&t);
    uint32_t i;
    int j, r;
    DIR *dir;

    dir = opendir(_logs_dir);
    if (!dir) {
        return log_error_errno(errno, "Could not open log dir (%m)");
    }
    // Close dir when leaving function.
    std::shared_ptr<void> defer(dir, [](DIR *p) { closedir(p); });

    i = _get_prefix(dir);

    for (j = 0; j <= MAX_RETRIES; j++) {
        r = snprintf(filename, filename_size, "%05u-%i-%02i-%02i_%02i-%02i-%02i.%s", i + j,
                     timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                     timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, extension);

        if (r < 1 || (size_t)r >= filename_size) {
            return log_error_errno(errno, "Error formatting Log file name: (%m)");
        }

        r = openat(dirfd(dir), filename, O_WRONLY | O_CLOEXEC | O_CREAT | O_NONBLOCK | O_EXCL,
                   0644);
        if (r < 0) {
            if (errno != EEXIST) {
                return log_error_errno(errno, "Unable to open Log file(%s): (%m)", filename);
            }
            continue;
        }

        return r;
    }

    log_error("Unable to create a Log file without override another file.");
    return -EEXIST;
}

void LogEndpoint::stop()
{
    if (_logging_start_timeout) {
        _mainloop->del_timeout(_logging_start_timeout);
        _logging_start_timeout = nullptr;
    }

    if (_alive_check_timeout) {
        _mainloop->del_timeout(_alive_check_timeout);
        _alive_check_timeout = nullptr;
    }

    fsync(_file);
    close(_file);
    _file = -1;
    _system_id = 0;
}

bool LogEndpoint::start()
{
    char filename[PATH_MAX];

    if (_file != -1) {
        log_warning("Log already started");
        return false;
    }

    _file = _get_file(_get_logfile_extension(), filename, sizeof(filename));
    if (_file < 0) {
        _file = -1;
        return false;
    }

    _logging_start_timeout
        = _mainloop->add_timeout(MSEC_PER_SEC, std::bind(&LogEndpoint::_start_timeout, this), this);
    if (!_logging_start_timeout) {
        log_error("Unable to add timeout");
        goto timeout_error;
    }

    log_info("Logging target system_id=%u on %s", _target_system_id, filename);

    return true;

timeout_error:
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

void LogEndpoint::_remove_start_timeout()
{
    _mainloop->del_timeout(_logging_start_timeout);
    _logging_start_timeout = nullptr;
}

bool LogEndpoint::_start_alive_timeout()
{
    _alive_check_timeout = _mainloop->add_timeout(
        MSEC_PER_SEC * ALIVE_TIMEOUT, std::bind(&LogEndpoint::_alive_timeout, this), this);
    return !!_alive_check_timeout;
}
