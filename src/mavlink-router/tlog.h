/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2021 Patrick José Pereira <patrickelectric@gmail.com>
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

#include "logendpoint.h"

#define BUFFER_LEN 2048

class TLog : public LogEndpoint {
public:
    TLog(const char *logs_dir, LogMode mode, unsigned long min_free_space, unsigned long max_files)
        : LogEndpoint{"ULog", logs_dir, mode, min_free_space, max_files}
    {
    }

    bool start() override;
    void stop() override;

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

protected:
    ssize_t _read_msg(uint8_t *buf, size_t len) override { return 0; };
    bool _start_timeout() override;

    const char *_get_logfile_extension() override { return "tlog"; };
};
