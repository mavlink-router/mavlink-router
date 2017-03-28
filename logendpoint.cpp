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

#include "log.h"
#include "mainloop.h"
#include "util.h"

#define MAX_LOG_FILE_PREFIX 999

static bool _logging_start_timeout_cb(void *data)
{
    LogEndpoint *log = static_cast<LogEndpoint *>(data);
    return log->_start_timeout();
}

static bool _alive_timeout_cb(void *data)
{
    LogEndpoint *log = static_cast<LogEndpoint *>(data);
    return log->_alive_timeout();
}

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
    _stat.read.handled_bytes += msg->len;
    if (msg->magic == MAVLINK_STX)
        _stat.read.handled_bytes += sizeof(struct mavlink_router_mavlink2_header);
    else
        _stat.read.handled_bytes += sizeof(struct mavlink_router_mavlink1_header);
}

int LogEndpoint::_get_prefix(const char *extension, const char *dirname, uint16_t *prefix)
{
    struct dirent *ent;
    DIR *dir;
    uint16_t u;

    dir = opendir(dirname);
    if (!dir) {
        return -errno;
    }

    *prefix = 0;

    while ((ent = readdir(dir)) != nullptr) {
        if (sscanf(ent->d_name, "%hu-", &u) == 1) {
            if (u >= *prefix && u < MAX_LOG_FILE_PREFIX) {
                *prefix = ++u;
            }
        }
    }

    closedir(dir);

    return 0;
}

int LogEndpoint::_get_file(const char *extension, char **filename_result)
{
    char filename[PATH_MAX] = {};

    time_t t = time(NULL);
    struct tm *timeinfo = localtime(&t);
    int r;
    uint16_t i;

    if (_get_prefix(extension, _logs_dir, &i) < 0) {
        log_error_errno(errno, "Error formatting Log file name: (%m)");
    }

    for (; i <= MAX_LOG_FILE_PREFIX; i++) {
        r = snprintf(filename, sizeof(filename), "%s/%03u-%i-%02i-%02i_%02i-%02i-%02i.%s",
                     _logs_dir, i, timeinfo->tm_year + 1900, timeinfo->tm_mon + 1,
                     timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
                     extension);

        if (r < 1 || (size_t)r >= sizeof(filename)) {
            log_error_errno(errno, "Error formatting Log file name: (%m)");
            return -EINVAL;
        }

        r = open(filename, O_WRONLY | O_CLOEXEC | O_CREAT | O_NONBLOCK | O_TRUNC,
                 S_IRUSR | S_IROTH | S_IRGRP);
        if (r >= 0) {
            *filename_result = strdup(filename);
            if (!*filename_result) {
                log_error_errno(errno, "Unable to open Log file: (%m)");
                close(r);
                return -ENOMEM;
            }
            return r;
        }
        if (errno != EEXIST) {
            log_error_errno(errno, "Unable to open Log file(%s): (%m)", filename);
            return -1;
        }
    }

    log_error("Unable to create a Log file without override another file.");
    return -1;
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
    char *filename = NULL;

    if (_file != -1) {
        log_warning("Log already started");
        return false;
    }

    _file = _get_file(_get_logfile_extension(), &filename);
    if (_file < 0) {
        _file = -1;
        return false;
    }

    _logging_start_timeout = _mainloop->add_timeout(MSEC_PER_SEC, _logging_start_timeout_cb, this);
    if (!_logging_start_timeout) {
        log_error("Unable to add timeout");
        goto timeout_error;
    }

    log_info("Logging target system_id=%u on %s", _target_system_id, filename);
    free(filename);

    return true;

timeout_error:
    close(_file);
    _file = -1;
    free(filename);
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
    _alive_check_timeout
        = _mainloop->add_timeout(MSEC_PER_SEC * ALIVE_TIMEOUT, _alive_timeout_cb, this);
    return !!_alive_check_timeout;
}
