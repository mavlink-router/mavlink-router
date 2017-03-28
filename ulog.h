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

#include "endpoint.h"
#include "timeout.h"

#define TARGET_SYSTEM_ID 1
#define SYSTEM_ID 2

#define BUFFER_LEN 2048

class ULog : public Endpoint {
public:
    ULog(const char *logs_dir);

    bool start();
    bool logging_start_timeout();
    bool alive_check_timeout();

    void stop();

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

protected:
    ssize_t _read_msg(uint8_t *buf, size_t len) { return 0; };
    const int _target_system_id = TARGET_SYSTEM_ID;

private:
    const char *_logs_dir;
    int _file = -1;
    Timeout *_logging_start_timeout = nullptr;

    Timeout *_alive_check_timeout = nullptr;
    uint32_t _timeout_write_total = 0;

    uint16_t _expected_seq = 0;
    bool _waiting_header = true;
    bool _waiting_first_msg_offset = false;
    uint8_t _buffer[BUFFER_LEN];
    uint16_t _buffer_len = 0;
    /* Where valid data starts on buffer */
    uint16_t _buffer_index = 0;
    uint8_t _buffer_partial[BUFFER_LEN / 2];
    uint16_t _buffer_partial_len = 0;

    bool _logging_seq(uint16_t seq, bool *drop);
    void _logging_data_process(mavlink_logging_data_t *msg);
    bool _logging_flush();

    void _send_msg(const mavlink_message_t *msg, int target_sysid);
};
