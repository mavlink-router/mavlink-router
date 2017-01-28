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

#include <mavlink.h>
#include <string.h>

#include "comm.h"
#include "log.h"
#include "ulog.h"
#include "util.h"

#define TARGET_SYSTEM_ID 1
#define SYSTEM_ID 2

#define ULOG_HEADER_SIZE 16
#define ULOG_MAGIC                               \
    {                                            \
        0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35 \
    }

#define NO_FIRST_MSG_OFFSET 255

struct _packed_ ulog_msg_header {
    uint16_t msg_size;
    uint8_t msg_type;
};

static bool _ulog_logging_start_timeout(void *data)
{
    ULog *ulog = static_cast<ULog *>(data);
    return ulog->logging_start_timeout();
}

bool ULog::logging_start_timeout()
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    bzero(&cmd, sizeof(cmd));
    cmd.command = MAV_CMD_LOGGING_START;
    cmd.target_component = MAV_COMP_ID_ALL;
    cmd.target_system = TARGET_SYSTEM_ID;

    mavlink_msg_command_long_encode(SYSTEM_ID, 0, &msg, &cmd);
    _endpoint->write_msg(&msg);

    return true;
}

static bool _on_master_msg_cb(struct buffer *buf, void *data)
{
    ULog *ulog = static_cast<ULog *>(data);
    return ulog->handle_fligh_stack_message(buf);
}

bool ULog::start(Mainloop *mainloop, Endpoint *master, const char *base_path)
{
    time_t t = time(NULL);
    struct tm *timeinfo = localtime(&t);
    char path[1024];

    null_check(mainloop, false);
    null_check(master, false);

    if (_file) {
        log_error("ULog already started");
        return false;
    }

    snprintf(path, sizeof(path), "%s/%i-%i-%i_%i-%i-%i.ulg", (base_path ? base_path : ""),
             timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour,
             timeinfo->tm_min, timeinfo->tm_sec);
    _file = fopen(path, "wb");
    if (!_file) {
        log_error_errno(errno, "Unable to open ULog file(%s): (%m)", path);
        return false;
    }

    _endpoint = master;
    _mainloop = mainloop;
    _mainloop->set_on_master_msg_callback(_on_master_msg_cb, this);
    _timeout = _mainloop->timeout_add(MSEC_PER_SEC, _ulog_logging_start_timeout, this);
    _waiting_header = true;
    _waiting_first_msg_offset = false;
    logging_start_timeout();

    return _timeout;
}

void ULog::stop()
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    if (!_file) {
        log_error("ULog not started");
        return;
    }

    bzero(&cmd, sizeof(cmd));
    cmd.command = MAV_CMD_LOGGING_STOP;
    cmd.target_component = MAV_COMP_ID_ALL;
    cmd.target_system = TARGET_SYSTEM_ID;

    mavlink_msg_command_long_encode(SYSTEM_ID, 0, &msg, &cmd);
    _endpoint->write_msg(&msg);

    fflush(_file);
    fclose(_file);
    _file = NULL;
    _expected_seq = 0;
    _mainloop->set_on_master_msg_callback(NULL, NULL);
}

bool ULog::handle_fligh_stack_message(struct buffer *buffer)
{
    const bool mavlink2 = buffer->data[0] == MAVLINK_STX;
    uint32_t msg_id;

    if (mavlink2) {
        struct mavlink_router_mavlink2_header *msg
            = (struct mavlink_router_mavlink2_header *)buffer->data;
        msg_id = msg->msgid;
    } else {
        struct mavlink_router_mavlink1_header *msg
            = (struct mavlink_router_mavlink1_header *)buffer->data;
        msg_id = msg->msgid;
    }

    /* Check if we are interested in this msg_id */
    if (msg_id != MAVLINK_MSG_ID_COMMAND_ACK && msg_id != MAVLINK_MSG_ID_LOGGING_DATA_ACKED
        && msg_id != MAVLINK_MSG_ID_LOGGING_DATA) {
        return false;
    }

    /*
     * As MAVLINK_MSG_ID_LOGGING_DATA_ACKED and MAVLINK_MSG_ID_LOGGING_DATA
     * are MAVLink 2 only messages lets untrim all messages that match
     * with the interested msg_ids.
     */
    struct buffer msg_buffer;
    uint8_t data[MAVLINK_MAX_PACKET_LEN * 4];
    uint8_t *payload;
    msg_buffer.data = data;

    _endpoint->untrim_msg(buffer, &msg_buffer);
    buffer = &msg_buffer;

    if (mavlink2)
        payload = buffer->data + sizeof(struct mavlink_router_mavlink2_header);
    else
        payload = buffer->data + sizeof(struct mavlink_router_mavlink1_header);

    /* Handle messages */
    switch (msg_id) {
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t *cmd = (mavlink_command_ack_t *)payload;

        if (!_timeout || cmd->command != MAV_CMD_LOGGING_START)
            return false;

        if (cmd->result == MAV_RESULT_ACCEPTED) {
            _mainloop->timeout_del(_timeout);
            _timeout = NULL;
        }
        break;
    }
    case MAVLINK_MSG_ID_LOGGING_DATA_ACKED: {
        mavlink_logging_data_acked_t *ulog_data_acked = (mavlink_logging_data_acked_t *)payload;
        mavlink_message_t msg;
        mavlink_logging_ack_t ack;

        ack.sequence = ulog_data_acked->sequence;
        ack.target_component = MAV_COMP_ID_ALL;
        ack.target_system = TARGET_SYSTEM_ID;
        mavlink_msg_logging_ack_encode(SYSTEM_ID, 0, &msg, &ack);
        _endpoint->write_msg(&msg);
    }
    case MAVLINK_MSG_ID_LOGGING_DATA: {
        mavlink_logging_data_t *ulog_data = (mavlink_logging_data_t *)payload;
        _logging_data_process(ulog_data);
        break;
    }
    }

    return true;
}

void ULog::_logging_data_process(mavlink_logging_data_t *msg)
{
    bool drops = false;

    /* Check for message drops */
    if (_expected_seq != msg->sequence) {
        drops = true;
        _expected_seq = msg->sequence;
    }
    _expected_seq++;

    /* Waiting for ULog header? */
    if (_waiting_header) {
        const uint8_t magic[] = ULOG_MAGIC;

        if (memcmp(magic, msg->data, sizeof(magic)))
            return;

        _waiting_header = false;
        fwrite(msg->data, 1, ULOG_HEADER_SIZE, _file);
        memmove(msg->data, &msg->data[ULOG_HEADER_SIZE], msg->length);
        msg->length -= ULOG_HEADER_SIZE;
    }

    if (drops) {
        _logging_flush();

        _buffer_len = 0;
        _waiting_first_msg_offset = true;
    }

    /*
     * Do not cause a buffer overflow, it should only happens if a ULog message
     * don't fit in _msg_buffer
     */
    if (_buffer_len + msg->length >= sizeof(_buffer)) {
        log_warning("Buffer full, dropping everything on buffer");

        _buffer_len = 0;
        _waiting_first_msg_offset = true;
    }

    uint8_t begin = 0;

    if (_waiting_first_msg_offset) {
        if (msg->first_message_offset == NO_FIRST_MSG_OFFSET) {
            /* no useful information in this message */
            return;
        }

        _waiting_first_msg_offset = false;
        begin = msg->first_message_offset;
    }

    if (!msg->length)
        return;

    memcpy(&_buffer[_buffer_len], &msg->data[begin], msg->length - begin);
    _buffer_len += (msg->length - begin);
    _logging_flush();
}

void ULog::_logging_flush()
{
    while (_buffer_len >= sizeof(struct ulog_msg_header)) {
        struct ulog_msg_header *header = (struct ulog_msg_header *)_buffer;
        const uint16_t full_msg_size = header->msg_size + sizeof(struct ulog_msg_header);

        if (full_msg_size > _buffer_len) {
            break;
        }

        fwrite(_buffer, 1, full_msg_size, _file);
        _buffer_len -= full_msg_size;
        memmove(_buffer, &_buffer[full_msg_size], _buffer_len);
    }
}
