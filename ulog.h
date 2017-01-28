/*
 * This file is part of the mavroute project
 *
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
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

#include "comm.h"

class ULog {
public:
    bool start(Mainloop *mainloop, Endpoint *master, const char *ulog_base_path);
    void stop();

    /**
     * @return true if message was consumed by ULog
     **/
    bool handle_fligh_stack_message(struct buffer *buffer);

    bool logging_start_timeout();

private:
    FILE *_file = NULL;

    Endpoint *_endpoint;
    Mainloop *_mainloop;
    Timeout *_timeout;

    void _logging_flush();
    void _logging_data_process(mavlink_logging_data_t *msg);
    uint16_t _expected_seq = 0;
    bool _waiting_header;
    uint8_t _buffer[2048];
    uint16_t _buffer_len = 0;
    bool _waiting_first_msg_offset;
};
