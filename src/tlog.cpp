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
#include <sys/uio.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>

#include <common/log.h>
#include <common/util.h>

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

int TLog::write_msg(const struct buffer *buffer)
{
    /* set the expected system id to the first autopilot that we get a heartbeat from */
    if (_target_system_id == -1 && buffer->curr.msg_id == MAVLINK_MSG_ID_HEARTBEAT
        && buffer->curr.src_compid == MAV_COMP_ID_AUTOPILOT1) {
        _target_system_id = buffer->curr.src_sysid;
    }

    /* Check if we should start or stop logging */
    _handle_auto_start_stop(buffer);

    if (_file < 0) {
        return buffer->len;
    }

    const size_t record_size = sizeof(uint64_t) + buffer->len;

    uint64_t ms_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();

    ms_since_epoch = htobe64(ms_since_epoch);

    // This check is best-effort: other writers can consume space before writev().
    const int space_result = _check_minimum_free_space(record_size);
    if (space_result < 0) {
        if (space_result == -ENOSPC) {
            _config.log_mode = LogMode::disabled;
            stop();
        }
        return space_result;
    }

    struct iovec iov[2] = {
        {(void *)&ms_since_epoch, sizeof(ms_since_epoch)},
        {buffer->data, buffer->len},
    };
    ssize_t written;
    do {
        written = writev(_file, iov, 2);
    } while (written < 0 && errno == EINTR);
    if (written < 0) {
        const int write_errno = errno;
        log_error("Unable to write telemetry log record: %m");
        _config.log_mode = LogMode::disabled;
        stop();
        return -write_errno;
    }
    if (written != (ssize_t)record_size) {
        log_error("Short telemetry log write: wrote %zd of %zu bytes", written, record_size);
        _config.log_mode = LogMode::disabled;
        stop();
        return -EIO;
    }

    return buffer->len;
}
