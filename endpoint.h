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

#include <mavlink.h>

#include "comm.h"

class Endpoint {
public:
    Endpoint(const char *name, bool crc_check_enabled);
    virtual ~Endpoint();

    int read_msg(struct buffer *pbuf, int *target_system, int *target_component);
    void print_statistics();
    virtual int write_msg(const struct buffer *pbuf) = 0;
    virtual int flush_pending_msgs() = 0;

    uint8_t get_system_id() { return system_id; }
    uint8_t get_component_id() { return component_id; }

    struct buffer rx_buf;
    struct buffer tx_buf;
    int fd = -1;

protected:
    virtual ssize_t _read_msg(uint8_t *buf, size_t len) = 0;
    bool _check_crc(const mavlink_msg_entry_t *msg_entry);

    const char *_name;
    size_t _last_packet_len = 0;

    uint32_t _read_crc_errors = 0;
    uint32_t _read_total = 0;
    uint32_t _write_total = 0;
    const bool _crc_check_enabled;

    uint8_t system_id = 0;
    uint8_t component_id = 0;
};

class UartEndpoint : public Endpoint {
public:
    UartEndpoint() : Endpoint{"UART", true} { }
    virtual ~UartEndpoint() { }
    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    int open(const char *path, speed_t baudrate);
protected:
    ssize_t _read_msg(uint8_t *buf, size_t len) override;
};

class UdpEndpoint : public Endpoint {
public:
    UdpEndpoint();
    virtual ~UdpEndpoint() { }

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    int open(const char *ip, unsigned long port, bool bind = false);

    struct sockaddr_in sockaddr;

protected:
    ssize_t _read_msg(uint8_t *buf, size_t len) override;
};

class TcpEndpoint : public Endpoint {
public:
    TcpEndpoint();
    virtual ~TcpEndpoint();

    int accept(int listener_fd);

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    struct sockaddr_in sockaddr;

protected:
    ssize_t _read_msg(uint8_t *buf, size_t len) override;
};
