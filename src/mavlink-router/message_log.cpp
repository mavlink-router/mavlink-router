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
#include "mainloop.h"
#include "message_log.h"
#include "common/log.h"
#include <cinttypes>
#include <common/util.h>

MessageLog::MessageLog(Mainloop& mainloop)
    : _mainloop(mainloop)
{
    _update_timer = mainloop.add_timeout(
        MSEC_PER_SEC,
        std::bind(&MessageLog::update_heartbeats, this),
        this);
    if (!_update_timer) {
        log_error("Unable to add timeout for heartbeat timeouts");
    }
}

MessageLog::~MessageLog()
{
    if (_update_timer) {
        _mainloop.del_timeout(_update_timer);
    }
}

void MessageLog::log_incoming(const char* source, const uint8_t *payload, uint8_t payload_len,
                     const mavlink_msg_entry_t *msg_entry, uint8_t trimmed_zeros, uint32_t msg_id, uint8_t src_sysid,
                     uint8_t src_compid)
{
    if (msg_entry) {
        if (payload_len > msg_entry->max_msg_len) {
            payload_len = msg_entry->max_msg_len;
        }

        /* Handle messages */
        switch (msg_id) {
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            copy_message(payload, payload_len, trimmed_zeros, cmd);

            if (cmd.command != MAV_CMD_REQUEST_MESSAGE) { // Skip as it's too verbose and not interesting
                log_debug("mavlink_command_long: src: %s, compid: %i, cmd: %i, target: %i, params: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                          source, src_compid, cmd.command, cmd.target_component, cmd.param1,
                          cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6, cmd.param7);
            }
        }
            break;

        case MAVLINK_MSG_ID_COMMAND_INT: {
            mavlink_command_int_t cmd;
            copy_message(payload, payload_len, trimmed_zeros, cmd);

            if (cmd.command != MAV_CMD_REQUEST_MESSAGE) {
                log_debug("mavlink_command_int: src: %s, compid: %i, cmd: %i, target: %i, params: [%.1f, %.1f, %.1f, %.1f, %" PRId32 ", %" PRId32 ", %.1f]",
                          source, src_compid, cmd.command, cmd.target_component, cmd.param1,
                          cmd.param2, cmd.param3, cmd.param4, cmd.x, cmd.y, cmd.z);
            }
        }
            break;

        case MAVLINK_MSG_ID_SET_MODE: {
            mavlink_set_mode_t msg;
            copy_message(payload, payload_len, trimmed_zeros, msg);

            log_debug("mavlink_set_mode: src: %s, compid: %i, base_mode: %" PRIu8 ", custom_mode: %" PRIu32,
                      source, src_compid, msg.base_mode, msg.custom_mode);
        }
            break;

        case MAVLINK_MSG_ID_SERIAL_CONTROL: { /* Shell commands */
            mavlink_serial_control_t msg;
            copy_message(payload, payload_len, trimmed_zeros, msg);
            msg.data[MAVLINK_MSG_SERIAL_CONTROL_FIELD_DATA_LEN - 1] = '\0';

            log_debug("mavlink_serial_control: src: %s, compid: %i, target: %i, data: %s",
                      source, src_compid, msg.target_component, msg.data);
        }
            break;

        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t msg;
            copy_message(payload, payload_len, trimmed_zeros, msg);
            handle_heartbeat(source, msg, src_compid);
        }
            break;

        }
    }
}
bool MessageLog::update_heartbeats() {
    static const auto heartbeat_timeout = std::chrono::seconds(3);

    const auto now = std::chrono::steady_clock::now();
    // Check for expired heartbeats
    for (auto it = _heartbeats.begin(); it != _heartbeats.end();) {
        if(now - it->second > heartbeat_timeout) {
            log_debug("Heartbeat timeout: %s", it->first.c_str());
            it = _heartbeats.erase(it);
        } else {
            it++;
        }
    }
    return true;
}
void MessageLog::handle_heartbeat(const char *source, const mavlink_heartbeat_t &msg, uint8_t src_compid) {
    const std::string name = std::string(source) + " (compid: " + std::to_string(src_compid) + ")";
    const auto now = std::chrono::steady_clock::now();

    // Insert or update
    auto iter = _heartbeats.find(name);
    if (iter == _heartbeats.end()) {
        log_debug("New heartbeat: %s, type: %" PRIu8, name.c_str(), msg.type);
        _heartbeats[name] = now;
    } else {
        iter->second = now;
    }
}

template <class mavlink_message_t>
void MessageLog::copy_message(const uint8_t *payload, uint8_t payload_len, uint8_t trimmed_zeros,
                              mavlink_message_t &message)
{
    memcpy(&message, payload, payload_len);
    if (trimmed_zeros) {
        memset(((uint8_t *)&message) + payload_len, 0, trimmed_zeros);
    }
}
