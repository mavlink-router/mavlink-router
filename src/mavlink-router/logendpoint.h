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
    LogEndpoint(const char *name, const char *logs_dir, LogMode mode)
        : Endpoint{name, false}
        , _logs_dir{logs_dir}
        , _mode(mode)
    {
        assert(_logs_dir);
        _add_sys_comp_id(LOG_ENDPOINT_SYSTEM_ID << 8);
    }

    virtual bool start();
    virtual void stop();

    bool has_active_stop_timeout() { return _logging_stop_timeout != nullptr; }

protected:
    const char *_logs_dir;
    int _target_system_id = -1;
    int _file = -1;
    LogMode _mode;

    Timeout *_logging_start_timeout = nullptr;
    Timeout *_logging_stop_timeout = nullptr;
    Timeout *_alive_check_timeout = nullptr;
    uint32_t _timeout_write_total = 0;

    virtual const char *_get_logfile_extension() = 0;

    void _send_msg(const mavlink_message_t *msg, int target_sysid);
    void _remove_start_timeout();
    void _remove_stop_timeout();
    bool _start_alive_timeout();

    virtual bool _start_timeout() = 0;
    virtual bool _stop_timeout() = 0;
    virtual bool _alive_timeout();

    void _handle_auto_start_stop(uint32_t msg_id, uint8_t source_system_id,
            uint8_t source_component_id, uint8_t *payload);

private:
    int _get_file(const char *extension);
    uint32_t _get_prefix(DIR *dir);
    DIR *_open_or_create_dir(const char *name);

    char _filename[64];
};
