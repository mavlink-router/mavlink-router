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
#include "autolog.h"

#include <common/log.h>

#include "binlog.h"
#include "ulog.h"

int AutoLog::write_msg(const struct buffer *buffer)
{
    if (_logger != nullptr) {
        return _logger->write_msg(buffer);
    }

    const bool mavlink2 = buffer->data[0] == MAVLINK_STX;
    uint32_t msg_id;
    uint8_t *payload;
    uint8_t source_system_id;

    if (mavlink2) {
        struct mavlink_router_mavlink2_header *hdr
            = (struct mavlink_router_mavlink2_header *)buffer->data;
        msg_id = hdr->msgid;
        payload = buffer->data + sizeof(*hdr);
        source_system_id = hdr->sysid;
    } else {
        struct mavlink_router_mavlink1_header *hdr
            = (struct mavlink_router_mavlink1_header *)buffer->data;
        msg_id = hdr->msgid;
        payload = buffer->data + sizeof(*hdr);
        source_system_id = hdr->sysid;
    }

    if (msg_id != MAVLINK_MSG_ID_HEARTBEAT || source_system_id != LOG_ENDPOINT_TARGET_SYSTEM_ID) {
        return buffer->len;
    }

    const mavlink_heartbeat_t *heartbeat = (mavlink_heartbeat_t *)payload;

    /* We check autopilot on heartbeat */
    log_debug("Got autopilot %u from heartbeat", heartbeat->autopilot);
    if (heartbeat->autopilot == MAV_AUTOPILOT_PX4) {
        _logger = std::unique_ptr<LogEndpoint>(new ULog(_logs_dir, _mode));
    } else if (heartbeat->autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        _logger = std::unique_ptr<LogEndpoint>(new BinLog(_logs_dir, _mode));
    } else {
        log_warning("Unidentified autopilot, cannot start flight stack logging");
    }

    return buffer->len;
}

void AutoLog::stop()
{
    if (_logger)
        _logger->stop();
}

bool AutoLog::start()
{
    return true;
}

void AutoLog::print_statistics()
{
    if (_logger) {
        _logger->print_statistics();
    } else {
        Endpoint::print_statistics();
    }
}
