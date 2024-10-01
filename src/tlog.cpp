/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2023 Petrosilius <petrosilius@searchwing.org>
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
#include "tlog.h"

#include <assert.h>
#include <endian.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>

#include <common/log.h>
#include <common/util.h>

// needed to send messages
#include <common/mavlink.h>
//#include <common/mavlink_msg_param_request_list.h>
#include <sys/socket.h>
//#include <mavlink_helpers.h>

#define SYSTEM_ID 10

static int fd;
static struct sockaddr_in sockaddr;

// Define the msg_send function at the top
static int msg_send(mavlink_message_t *msg)
{
    uint8_t data[MAVLINK_MAX_PACKET_LEN];

    uint16_t len = mavlink_msg_to_send_buffer(data, msg);
    return sendto(fd, data, len, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}

bool TLog::_logging_start_timeout()
{
    //_target_system_id = -1;
    return true;
}

bool TLog::start()
{
    if (!LogEndpoint::start()) {
        return false;
    }

    return true;
}

void TLog::stop()
{
    if (_file == -1) {
        log_info("TLog not started");
        return;
    }

    LogEndpoint::stop();
}

void TLog::_split_logfile()
{
    // Close the current log file
    if (_file != -1) {
        fsync(_file);
        close(_file);
        _file = -1;

        // Mark the file as read-only
        char log_file[PATH_MAX];
        if (snprintf(log_file, sizeof(log_file), "%s/%s", _config.logs_dir.c_str(), _filename)
            < (int)sizeof(log_file)) {
            chmod(log_file, S_IRUSR | S_IRGRP | S_IROTH);
        }
    }

    // Open a new log file with a unique name and timestamp
    _file = _get_file(_get_logfile_extension());
    if (_file < 0) {
        log_error("Failed to open new tlog file");
        return;
    }

    log_info("TLog file rotated, new file: %s", _filename);

    // Send MAVLink packet to request all parameters
    mavlink_message_t msg;
    mavlink_msg_param_request_list_pack(
        SYSTEM_ID, MAV_COMP_ID_ALL, &msg,
        _target_system_id, MAV_COMP_ID_ALL);

    // Send the MAVLink packet
    msg_send(&msg);
    log_info("Sent PARAM_REQUEST_LIST to system %d", _target_system_id);
}

void TLog::_check_and_split_logfile()
{
    if (_write_msg_count % 1000 != 0) {
        return;
    }

    struct stat file_stat;
    if (fstat(_file, &file_stat) != 0) {
        log_error("Failed to get file stats: %m");
        return;
    }

    if (file_stat.st_size < _max_tlog_file_size) {
        return;
    }

    log_info("Attempting to split file, current size: %ld, max size: %ld",
             file_stat.st_size,
             _max_tlog_file_size);
    _split_logfile();
}

int TLog::write_msg(const struct buffer *buffer)
{
    /* set the expected system id to the first autopilot that we get a heartbeat from */
    if (_target_system_id == -1 && buffer->curr.msg_id == MAVLINK_MSG_ID_HEARTBEAT
        && buffer->curr.src_compid == MAV_COMP_ID_AUTOPILOT1) {
        _target_system_id = buffer->curr.src_sysid;
    }

    /* Check if we should start or stop logging */
    _handle_auto_start_stop(buffer);

    /* Check and split the log file if necessary */
    _check_and_split_logfile();

    uint64_t ms_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();

    ms_since_epoch = htobe64(ms_since_epoch);

    write(_file, (void *)&ms_since_epoch, sizeof(uint64_t));
    write(_file, buffer->data, buffer->len);
    _write_msg_count++;
    return buffer->len;
}