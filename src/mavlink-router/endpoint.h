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

#include <common/mavlink.h>

#include <memory>
#include <vector>

#include "comm.h"
#include "pollable.h"
#include "timeout.h"

class Mainloop;

/*
 * mavlink 2.0 packet in its wire format
 *
 * Packet size:
 *      sizeof(mavlink_router_mavlink2_header)
 *      + payload length
 *      + 2 (checksum)
 *      + signature (0 if not signed)
 */
struct _packed_ mavlink_router_mavlink2_header {
    uint8_t magic;
    uint8_t payload_len;
    uint8_t incompat_flags;
    uint8_t compat_flags;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint32_t msgid : 24;
};

/*
 * mavlink 1.0 packet in its wire format
 *
 * Packet size:
 *      sizeof(mavlink_router_mavlink1_header)
 *      + payload length
 *      + 2 (checksum)
 */
struct _packed_ mavlink_router_mavlink1_header {
    uint8_t magic;
    uint8_t payload_len;
    uint8_t seq;
    uint8_t sysid;
    uint8_t compid;
    uint8_t msgid;
};

class Endpoint : public Pollable {
public:
    /*
     * Success returns for @read_msg()
     */
    enum read_msg_result {
        ReadOk = 1,
        ReadUnkownMsg,
    };

    Endpoint(const char *name);
    virtual ~Endpoint();

    int handle_read() override;
    bool handle_canwrite() override;

    virtual void print_statistics();
    virtual int write_msg(const struct buffer *pbuf) = 0;
    virtual int flush_pending_msgs() = 0;

    void log_aggregate(unsigned int interval_sec);

    uint8_t get_trimmed_zeros(const mavlink_msg_entry_t *msg_entry, const struct buffer *buffer);

    bool has_sys_id(unsigned sysid);
    bool has_sys_comp_id(unsigned sys_comp_id);
    bool has_sys_comp_id(unsigned sysid, unsigned compid) {
        uint16_t sys_comp_id = ((sysid & 0xff) << 8) | (compid & 0xff);
        return has_sys_comp_id(sys_comp_id);
    }

    bool accept_msg(int target_sysid, int target_compid, uint8_t src_sysid, uint8_t src_compid, uint32_t msg_id);

    void add_message_to_filter(uint32_t msg_id) { _message_filter.push_back(msg_id); }

    struct buffer rx_buf;
    struct buffer tx_buf;

protected:
    virtual int read_msg(struct buffer *pbuf, int *target_system, int *target_compid,
                         uint8_t *src_sysid, uint8_t *src_compid, uint32_t *msg_id);
    virtual ssize_t _read_msg(uint8_t *buf, size_t len) = 0;
    bool _check_crc(const mavlink_msg_entry_t *msg_entry);
    void _add_sys_comp_id(uint16_t sys_comp_id);

    const char *_name;
    size_t _last_packet_len = 0;

    // Statistics
    struct {
        struct {
            uint64_t crc_error_bytes = 0;
            uint64_t handled_bytes = 0;
            uint32_t total = 0; // handled + crc error + seq lost
            uint32_t crc_error = 0;
            uint32_t handled = 0;
            uint32_t drop_seq_total = 0;
            uint8_t expected_seq = 0;
        } read;
        struct {
            uint64_t bytes = 0;
            uint32_t total = 0;
        } write;
    } _stat;

    uint32_t _incomplete_msgs = 0;
    std::vector<uint16_t> _sys_comp_ids;

private:
    std::vector<uint32_t> _message_filter;
};

class UartEndpoint : public Endpoint {
public:
    UartEndpoint()
        : Endpoint {"UART"}
    {
    }
    virtual ~UartEndpoint();
    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    int open(const char *path);
    int set_speed(speed_t baudrate);
    int set_flow_control(bool enabled);
    int add_speeds(std::vector<unsigned long> baudrates);

protected:
    int read_msg(struct buffer *pbuf, int *target_system, int *target_compid, uint8_t *src_sysid,
                 uint8_t *src_compid, uint32_t *msg_id) override;
    ssize_t _read_msg(uint8_t *buf, size_t len) override;

private:
    size_t _current_baud_idx = 0;
    Timeout *_change_baud_timeout = nullptr;
    std::vector<unsigned long> _baudrates;

    bool _change_baud_cb(void *data);
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
    ~TcpEndpoint();

    int accept(int listener_fd);
    int open(const char *ip, unsigned long port);
    void close();

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    struct sockaddr_in sockaddr;
    int retry_timeout = 0;

    inline const char *get_ip() {
        return _ip;
    }

    inline unsigned long get_port() {
        return _port;
    }

    bool is_valid() override { return _valid; };

protected:
    ssize_t _read_msg(uint8_t *buf, size_t len) override;

private:
    char *_ip = nullptr;
    unsigned long _port = 0;
    bool _valid = true;
};
