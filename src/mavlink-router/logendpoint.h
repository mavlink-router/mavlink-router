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
#pragma once

#include <aio.h>
#include <assert.h>
#include <dirent.h>

#include "endpoint.h"
#include "timeout.h"

#define LOG_ENDPOINT_SYSTEM_ID 2


enum class LogMode {
    always = 0,         ///< Log from start until mavlink-router exits
    while_armed,        ///< Start logging when the vehicle is armed until it's disarmed

    disabled            ///< Do not try to start logging (only used internally)
};


class LogEndpoint : public Endpoint {
public:
    LogEndpoint(const char *name, const char *logs_dir, LogMode mode, unsigned long min_free_space,
                unsigned long max_files);

    virtual bool start();
    virtual void stop();

    /**
     * Check existing log files and mark logs as read-only if needed.
     * This handles the case where the system (or mavlink-router) crashed or
     * lost power.
     */
    void mark_unfinished_logs();

protected:
    const char *_logs_dir;
    int _target_system_id = -1;
    int _file = -1;
    unsigned long _min_free_space;
    unsigned long _max_files;
    LogMode _mode;

    Timeout *_logging_start_timeout = nullptr;
    Timeout *_fsync_timeout = nullptr;
    Timeout *_alive_check_timeout = nullptr;
    uint32_t _timeout_write_total = 0;
    aiocb _fsync_cb = {};

    virtual const char *_get_logfile_extension() = 0;

    void _send_msg(const mavlink_message_t *msg, int target_sysid);
    void _remove_start_timeout();
    bool _start_alive_timeout();

    virtual bool _start_timeout() = 0;
    virtual bool _alive_timeout();

    bool _fsync();

    void _handle_auto_start_stop(uint32_t msg_id, uint8_t source_system_id,
            uint8_t source_component_id, uint8_t *payload);

private:
    int _get_file(const char *extension);
    uint32_t _get_prefix(DIR *dir);
    DIR *_open_or_create_dir(const char *name);

    /**
     * Delete old logs until a certain amount of free space and total number of log files are met.
     * This can be configured using the .conf file, options MinFreeSpace and MaxLogFiles.
     */
    void _delete_old_logs();

    char _filename[64];
};
