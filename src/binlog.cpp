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

#include <common/log.h>
#include <common/util.h>

#include "mainloop.h"

bool BinLog::_logging_start_timeout()
{
    mavlink_message_t msg;

    mavlink_msg_remote_log_block_status_pack(LOG_ENDPOINT_SYSTEM_ID,
                                             MAV_COMP_ID_ALL,
                                             &msg,
                                             _target_system_id,
                                             MAV_COMP_ID_ALL,
                                             MAV_REMOTE_LOG_DATA_BLOCK_START,
                                             1);

    _send_msg(&msg, _target_system_id);

    return true;
}

bool BinLog::start()
{
    if (!LogEndpoint::start()) {
        return false;
    }

    _last_acked_seqno = 0;

    return true;
}

void BinLog::stop()
{
    if (_file == -1) {
        log_info("BinLog not started");
        return;
    }

    _send_stop();

    LogEndpoint::stop();
}

void BinLog::_send_stop()
{
    mavlink_message_t msg;

    mavlink_msg_remote_log_block_status_pack(LOG_ENDPOINT_SYSTEM_ID,
                                             MAV_COMP_ID_ALL,
                                             &msg,
                                             _target_system_id,
                                             MAV_COMP_ID_ALL,
                                             MAV_REMOTE_LOG_DATA_BLOCK_STOP,
                                             1);
    _send_msg(&msg, _target_system_id);
}

int BinLog::write_msg(const struct buffer *buffer)
{
    const bool mavlink2 = buffer->data[0] == MAVLINK_STX;
    uint8_t trimmed_zeros;
    mavlink_remote_log_data_block_t *binlog_data;

    /* set the expected system id to the first autopilot that we get a heartbeat from */
    if (_target_system_id == -1 && buffer->curr.msg_id == MAVLINK_MSG_ID_HEARTBEAT
        && buffer->curr.src_compid == MAV_COMP_ID_AUTOPILOT1) {
        _target_system_id = buffer->curr.src_sysid;
    }

    /* Check if we should start or stop logging */
    _handle_auto_start_stop(buffer);

    /* Check if we are interested in this msg_id */
    if (buffer->curr.msg_id != MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK) {
        return buffer->len;
    }

    const mavlink_msg_entry_t *msg_entry = mavlink_get_msg_entry(buffer->curr.msg_id);
    if (!msg_entry) {
        return buffer->len;
    }

    uint16_t payload_len = buffer->curr.payload_len;
    if (payload_len > msg_entry->max_msg_len) {
        payload_len = msg_entry->max_msg_len;
    }

    if (mavlink2) {
        trimmed_zeros = get_trimmed_zeros(msg_entry, buffer);
    } else {
        trimmed_zeros = 0;
    }

    if (trimmed_zeros) {
        binlog_data
            = (mavlink_remote_log_data_block_t *)alloca(sizeof(mavlink_remote_log_data_block_t));
        memcpy(binlog_data, buffer->curr.payload, payload_len);
        memset((uint8_t *)binlog_data + payload_len, 0, trimmed_zeros);
    } else {
        binlog_data = (mavlink_remote_log_data_block_t *)buffer->curr.payload;
    }

    if (_timeout.logging_start) {
        if (binlog_data->seqno == 0) {
            _remove_logging_start_timeout();
            if (!_start_alive_timeout()) {
                log_warning("Could not start liveness timeout - mavlink router log won't be able "
                            "to detect if flight stack stopped");
            }
        } else {
            // If message doesn't start a log session, send stop message
            _send_stop();
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
        mavlink_msg_remote_log_block_status_pack(LOG_ENDPOINT_SYSTEM_ID,
                                                 MAV_COMP_ID_ALL,
                                                 &msg,
                                                 _target_system_id,
                                                 MAV_COMP_ID_ALL,
                                                 seqno,
                                                 MAV_REMOTE_LOG_DATA_BLOCK_ACK);
        _send_msg(&msg, _target_system_id);
        return;
    }

    // TODO maybe a threshould of [n]acks to be sent?
    // TODO send ack to source only?
    // Send nacks regarding unseen seqno
    for (uint32_t i = _last_acked_seqno + 1; i < seqno; i++) {
        mavlink_msg_remote_log_block_status_pack(LOG_ENDPOINT_SYSTEM_ID,
                                                 MAV_COMP_ID_ALL,
                                                 &msg,
                                                 _target_system_id,
                                                 MAV_COMP_ID_ALL,
                                                 seqno,
                                                 MAV_REMOTE_LOG_DATA_BLOCK_NACK);
        _send_msg(&msg, _target_system_id);
    }

    // Send ack to seen seqno
    mavlink_msg_remote_log_block_status_pack(LOG_ENDPOINT_SYSTEM_ID,
                                             MAV_COMP_ID_ALL,
                                             &msg,
                                             _target_system_id,
                                             MAV_COMP_ID_ALL,
                                             seqno,
                                             MAV_REMOTE_LOG_DATA_BLOCK_ACK);
    _send_msg(&msg, _target_system_id);
    _last_acked_seqno = seqno;
}

void BinLog::_logging_data_process(mavlink_remote_log_data_block_t *msg)
{
    ssize_t r;

    if (lseek(_file, msg->seqno * MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN, SEEK_SET) < 0) {
        log_error("lseek failed (%m)");
        _restart();
        return;
    }

    r = write(_file, msg->data, MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN);
    if (r < 0 && errno != EAGAIN) {
        log_error("Error writing data (%m)");
        _restart();
        return;
    }
    if (r != MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN) {
        // partial writes are handled by not sending ack. Flight stack should resend
        // msg. We hope that partial writes are rare enough
        log_error("Log partial write %zd", r);
        return;
    }

    // TODO should we send acks on a different fashion. e.g. queueing and sending?
    _send_ack(msg->seqno);
}

void BinLog::_restart()
{
    log_info("Restarting log...");
    stop();
    start();
}

bool BinLog::_alive_timeout()
{
    if (_timeout_write_total == _stat.write.total) {
        // we probably just got FMT plus a MODE change
        // https://github.com/ArduPilot/ardupilot/blame/7b5ab8bb8cfd6fded545beb49585c54b38b04999/libraries/AP_Logger/AP_Logger_Backend.cpp#L454
        if (_last_acked_seqno > 400) {
            log_warning("No Log messages received in %u seconds restarting Log...", ALIVE_TIMEOUT);
            stop();
            start();
        } else {
            // silent restart
            LogEndpoint::_delete_last_log();
            stop();
            start();
        }

    }

    _timeout_write_total = _stat.write.total;
    return true;
}
