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

#include <memory>

#include "endpoint.h"
#include "logendpoint.h"

class AutoLog : public LogEndpoint {
public:

    AutoLog(const char *logs_dir, LogMode mode)
        : LogEndpoint{"AutoLog", logs_dir, mode}
    {
    }

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    bool start() override;
    void stop() override;
    void print_statistics() override;

protected:
    ssize_t _read_msg(uint8_t *buf, size_t len) override { return 0; }

    // These functions should never be called
    const char *_get_logfile_extension() override { return ""; };
    bool _start_timeout() override { return true; };

private:
    std::unique_ptr<LogEndpoint> _logger;
};
