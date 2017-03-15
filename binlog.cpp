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
#include "binlog.h"

#include <assert.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "log.h"
#include "mainloop.h"
#include "util.h"

bool BinLog::_start_timeout()
{
    mavlink_message_t msg;

    mavlink_msg_remote_log_block_status_pack(_system_id, MAV_COMP_ID_ALL, &msg, _target_system_id,
                                             MAV_COMP_ID_ALL, MAV_REMOTE_LOG_DATA_BLOCK_START, 1);
    _send_msg(&msg, _target_system_id);

    return true;
}

bool BinLog::start()
{
    if (!LogEndpoint::start()) {
        return false;
    }

    _last_acked_seqno = 0;
    _system_id = SYSTEM_ID;

    return true;
}

void BinLog::stop()
{
    mavlink_message_t msg;

    if (_file == -1) {
        log_error("BinLog not started");
        return;
    }

    mavlink_msg_remote_log_block_status_pack(_system_id, MAV_COMP_ID_ALL, &msg, _target_system_id,
                                             MAV_COMP_ID_ALL, MAV_REMOTE_LOG_DATA_BLOCK_STOP, 1);
    _send_msg(&msg, _target_system_id);

    LogEndpoint::stop();
}

int BinLog::write_msg(const struct buffer *buffer)
{
    const bool mavlink2 = buffer->data[0] == MAVLINK_STX;
    uint32_t msg_id;
    uint16_t payload_len;
    uint8_t trimmed_zeros;
    mavlink_remote_log_data_block_t *binlog_data;

    if (mavlink2) {
        struct mavlink_router_mavlink2_header *msg
            = (struct mavlink_router_mavlink2_header *)buffer->data;
        msg_id = msg->msgid;
        payload_len = msg->payload_len;
    } else {
        struct mavlink_router_mavlink1_header *msg
            = (struct mavlink_router_mavlink1_header *)buffer->data;
        msg_id = msg->msgid;
        payload_len = msg->payload_len;
    }

    /* Check if we are interested in this msg_id */
    if (msg_id != MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK) {
        return buffer->len;
    }

    uint8_t *payload;

    if (mavlink2) {
        payload = buffer->data + sizeof(struct mavlink_router_mavlink2_header);
        trimmed_zeros = get_trimmed_zeros(buffer);
    } else {
        payload = buffer->data + sizeof(struct mavlink_router_mavlink1_header);
        trimmed_zeros = 0;
    }

    if (trimmed_zeros) {
        binlog_data
            = (mavlink_remote_log_data_block_t *)alloca(sizeof(mavlink_remote_log_data_block_t));
        memcpy(binlog_data, payload, payload_len);
        memset(binlog_data + payload_len, 0, trimmed_zeros);
    } else {
        binlog_data = (mavlink_remote_log_data_block_t *)payload;
    }

    if (_logging_start_timeout) {
        // If message doesn't start a log session, just ignore it
        if (binlog_data->seqno == 0) {
            _remove_start_timeout();
            if (!_start_alive_timeout()) {
                log_warning("Could not start liveness timeout - mavlink router log won't be able "
                            "to detect if flight stack stopped");
            }
        } else {
            return buffer->len;
        }
    }
    _logging_data_process(binlog_data);

    _stat.write.total++;
    _stat.write.bytes += buffer->len;

    return buffer->len;
}

void BinLog::_send_ack(uint32_t seqno)
{
    mavlink_message_t msg;

    // Message filled a gap, or is duplicated. just send the ack
    if (seqno < _last_acked_seqno) {
        mavlink_msg_remote_log_block_status_pack(_system_id, MAV_COMP_ID_ALL, &msg,
                                                 _target_system_id, MAV_COMP_ID_ALL, seqno, 1);
        _send_msg(&msg, _target_system_id);
        return;
    }

    // TODO maybe a threshould of [n]acks to be sent?
    // TODO send ack to source only?
    // Send nacks regarding unseen seqno
    for (uint32_t i = _last_acked_seqno; i < seqno; i++) {
        mavlink_msg_remote_log_block_status_pack(_system_id, MAV_COMP_ID_ALL, &msg,
                                                 _target_system_id, MAV_COMP_ID_ALL, seqno, 0);
        _send_msg(&msg, _target_system_id);
    }

    // Send ack to seen seqno
    mavlink_msg_remote_log_block_status_pack(_system_id, MAV_COMP_ID_ALL, &msg, _target_system_id,
                                             MAV_COMP_ID_ALL, seqno, 1);
    _send_msg(&msg, _target_system_id);
    _last_acked_seqno = seqno;
}

void BinLog::_logging_data_process(mavlink_remote_log_data_block_t *msg)
{
    ssize_t r;

    if (lseek(_file, msg->seqno * MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN, SEEK_SET) < 0) {
        log_error_errno(errno, "lseek failed (%m)");
        return;
    }

    r = write(_file, msg->data, MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN);
    if (r != MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN) {
        // partial writes are handled by not sending ack. Flight stack should resend
        // msg. We hope that partial writes are rare enough
        log_error_errno(errno, "Error writing data (%m)");
        return;
    }

    // TODO should we send acks on a different fashion. e.g. queueing and sending?
    _send_ack(msg->seqno);
}
