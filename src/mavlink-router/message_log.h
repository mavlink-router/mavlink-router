/*
* This file is part of the MAVLink Router project
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
#include "timeout.h"
#include <chrono>
#include <common/mavlink.h>
#include <cstdint>
#include <string>
#include <unordered_map>

class Mainloop;

class MessageLog {
public:
    MessageLog(Mainloop& mainloop);
    ~MessageLog();

    void log_incoming(const char* source, const uint8_t *payload, uint8_t payload_len,
             const mavlink_msg_entry_t *msg_entry, uint8_t trimmed_zeros, uint32_t msg_id, uint8_t src_sysid, uint8_t src_compid
             );

private:
    template <class mavlink_message_t>
    inline void copy_message(const uint8_t *payload, uint8_t payload_len, uint8_t trimmed_zeros, mavlink_message_t& message);

    bool update_heartbeats();
    void handle_heartbeat(const char* source, const mavlink_heartbeat_t& msg, uint8_t src_compid);

    std::unordered_map<std::string, std::chrono::steady_clock::time_point> _heartbeats;
    Timeout* _update_timer = nullptr;
    Mainloop& _mainloop;
};