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

#include <asm/termbits.h>
#include <errno.h>
#include <inttypes.h>

#define MAVLINK_PACKET_MAX_SIZE 255

struct buffer {
    unsigned int len;
    uint8_t *data;
};

class Endpoint {
public:
    Endpoint();
    virtual ~Endpoint();

    virtual int read_msg(struct buffer *pbuf) = 0;
    virtual int write_msg(const struct buffer *pbuf) = 0;
    virtual int flush_pending_msgs() = 0;

    struct buffer rx_buf;
    struct buffer tx_buf;
    int fd = -1;
};

class UartEndpoint : public Endpoint {
public:
    UartEndpoint() { }
    virtual ~UartEndpoint() { }
    int read_msg(struct buffer *pbuf) override;
    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    int open(const char *path, speed_t baudrate);
};

class UdpEndpoint : public Endpoint {
public:
    virtual ~UdpEndpoint() { }

    int read_msg(struct buffer *pbuf) override;
    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    int open(const char *ip, unsigned long port);
};
