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

#include <common/conf_file.h>
#include <common/mavlink.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "comm.h"
#include "pollable.h"
#include "timeout.h"

#define DEFAULT_BAUDRATE 115200U

#define ENDPOINT_TYPE_UART "UART"
#define ENDPOINT_TYPE_UDP  "UDP"
#define ENDPOINT_TYPE_TCP  "TCP"
#define ENDPOINT_TYPE_LOG  "Log"

struct UartEndpointConfig {
    std::string name;
    std::string device;
    std::vector<uint32_t> baudrates;
    bool flowcontrol{false};
    std::vector<uint32_t> allow_msg_id_out;
    std::vector<uint32_t> block_msg_id_out;
    std::vector<uint8_t> allow_src_comp_out;
    std::vector<uint8_t> block_src_comp_out;
    std::vector<uint8_t> allow_src_sys_out;
    std::vector<uint8_t> block_src_sys_out;
    std::vector<uint32_t> allow_msg_id_in;
    std::vector<uint32_t> block_msg_id_in;
    std::vector<uint8_t> allow_src_comp_in;
    std::vector<uint8_t> block_src_comp_in;
    std::vector<uint8_t> allow_src_sys_in;
    std::vector<uint8_t> block_src_sys_in;
    std::string group;
};

struct UdpEndpointConfig {
    enum class Mode { Undefined = 0, Server, Client };

    std::string name;
    std::string address;
    unsigned long port;
    Mode mode;
    std::vector<uint32_t> allow_msg_id_out;
    std::vector<uint32_t> block_msg_id_out;
    std::vector<uint8_t> allow_src_comp_out;
    std::vector<uint8_t> block_src_comp_out;
    std::vector<uint8_t> allow_src_sys_out;
    std::vector<uint8_t> block_src_sys_out;
    std::vector<uint32_t> allow_msg_id_in;
    std::vector<uint32_t> block_msg_id_in;
    std::vector<uint8_t> allow_src_comp_in;
    std::vector<uint8_t> block_src_comp_in;
    std::vector<uint8_t> allow_src_sys_in;
    std::vector<uint8_t> block_src_sys_in;
    std::string group;
};

struct TcpEndpointConfig {
    std::string name;
    std::string address;
    unsigned long port;
    int retry_timeout{5};
    std::vector<uint32_t> allow_msg_id_out;
    std::vector<uint32_t> block_msg_id_out;
    std::vector<uint8_t> allow_src_comp_out;
    std::vector<uint8_t> block_src_comp_out;
    std::vector<uint8_t> allow_src_sys_out;
    std::vector<uint8_t> block_src_sys_out;
    std::vector<uint32_t> allow_msg_id_in;
    std::vector<uint32_t> block_msg_id_in;
    std::vector<uint8_t> allow_src_comp_in;
    std::vector<uint8_t> block_src_comp_in;
    std::vector<uint8_t> allow_src_sys_in;
    std::vector<uint8_t> block_src_sys_in;
    std::string group;
};

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

    /**
     * Return values for @accept_msg()
     */
    enum class AcceptState {
        Accepted = 1,
        Filtered,
        Rejected,
    };

    Endpoint(std::string type, std::string name);
    ~Endpoint() override;

    int handle_read() override;
    bool handle_canwrite() override;

    virtual void print_statistics();
    virtual int write_msg(const struct buffer *pbuf) = 0;
    virtual int flush_pending_msgs() = 0;

    void log_aggregate(unsigned int interval_sec);

    static uint8_t get_trimmed_zeros(const mavlink_msg_entry_t *msg_entry,
                                     const struct buffer *buffer);

    bool has_sys_id(unsigned sysid) const;
    bool has_sys_comp_id(unsigned sys_comp_id) const;
    bool has_sys_comp_id(unsigned sysid, unsigned compid) const
    {
        uint16_t sys_comp_id = ((sysid & 0xff) << 8) | (compid & 0xff);
        return has_sys_comp_id(sys_comp_id);
    }

    AcceptState accept_msg(const struct buffer *pbuf) const;

    void filter_add_allowed_out_msg_id(uint32_t msg_id)
    {
        _allowed_outgoing_msg_ids.push_back(msg_id);
    }
    void filter_add_blocked_out_msg_id(uint32_t msg_id)
    {
        _blocked_outgoing_msg_ids.push_back(msg_id);
    }
    void filter_add_allowed_out_src_comp(uint8_t src_comp)
    {
        _allowed_outgoing_src_comps.push_back(src_comp);
    }
    void filter_add_blocked_out_src_comp(uint8_t src_comp)
    {
        _blocked_outgoing_src_comps.push_back(src_comp);
    }
    void filter_add_allowed_out_src_sys(uint8_t src_sys)
    {
        _allowed_outgoing_src_systems.push_back(src_sys);
    }
    void filter_add_blocked_out_src_sys(uint8_t src_sys)
    {
        _blocked_outgoing_src_systems.push_back(src_sys);
    }
    void filter_add_allowed_in_msg_id(uint32_t msg_id)
    {
        _allowed_incoming_msg_ids.push_back(msg_id);
    }
    void filter_add_blocked_in_msg_id(uint32_t msg_id)
    {
        _blocked_incoming_msg_ids.push_back(msg_id);
    }
    void filter_add_allowed_in_src_comp(uint8_t src_comp)
    {
        _allowed_incoming_src_comps.push_back(src_comp);
    }
    void filter_add_blocked_in_src_comp(uint8_t src_comp)
    {
        _blocked_incoming_src_comps.push_back(src_comp);
    }
    void filter_add_allowed_in_src_sys(uint8_t src_sys)
    {
        _allowed_incoming_src_systems.push_back(src_sys);
    }
    void filter_add_blocked_in_src_sys(uint8_t src_sys)
    {
        _blocked_incoming_src_systems.push_back(src_sys);
    }

    bool allowed_by_dedup(const buffer *pbuf) const;
    bool allowed_by_incoming_filters(const struct buffer *pbuf) const;

    void link_group_member(std::shared_ptr<Endpoint> other);

    std::string get_type() const { return this->_type; }
    std::string get_group_name() const { return this->_group_name; };

    struct buffer rx_buf;
    struct buffer tx_buf;

    // An endpoint with this system id becomes a "sniffer" and all
    // messages are accepted.
    static uint16_t sniffer_sysid;

protected:
    virtual int read_msg(struct buffer *pbuf);
    virtual ssize_t _read_msg(uint8_t *buf, size_t len) = 0;
    bool _check_crc(const mavlink_msg_entry_t *msg_entry) const;
    void _add_sys_comp_id(uint8_t sysid, uint8_t compid);

    const std::string _type; ///< UART, UDP, TCP, Log
    std::string _name;       ///< Endpoint name from config file
    size_t _last_packet_len = 0;

    std::string _group_name{}; // empty name to disable endpoint groups
    std::vector<std::shared_ptr<Endpoint>> _group_members{};

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
    std::vector<uint32_t> _allowed_outgoing_msg_ids;
    std::vector<uint32_t> _blocked_outgoing_msg_ids;
    std::vector<uint8_t> _allowed_outgoing_src_comps;
    std::vector<uint8_t> _blocked_outgoing_src_comps;
    std::vector<uint8_t> _allowed_outgoing_src_systems;
    std::vector<uint8_t> _blocked_outgoing_src_systems;
    std::vector<uint32_t> _allowed_incoming_msg_ids;
    std::vector<uint32_t> _blocked_incoming_msg_ids;
    std::vector<uint8_t> _allowed_incoming_src_comps;
    std::vector<uint8_t> _blocked_incoming_src_comps;
    std::vector<uint8_t> _allowed_incoming_src_systems;
    std::vector<uint8_t> _blocked_incoming_src_systems;
};

class UartEndpoint : public Endpoint {
public:
    UartEndpoint(std::string name);
    ~UartEndpoint() override = default;

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    bool setup(UartEndpointConfig config); ///< open UART device and apply config

    static const ConfFile::OptionsTable option_table[];
    static const char *section_pattern;
    static bool validate_config(const UartEndpointConfig &config);

protected:
    bool open(const char *path);
    int set_speed(speed_t baudrate);
    int set_flow_control(bool enabled);
    int add_speeds(const std::vector<speed_t> &bauds);

    int read_msg(struct buffer *pbuf) override;
    ssize_t _read_msg(uint8_t *buf, size_t len) override;

private:
    size_t _current_baud_idx = 0;
    Timeout *_change_baud_timeout = nullptr;
    std::vector<uint32_t> _baudrates;

    bool _change_baud_cb(void *data);
};

class UdpEndpoint : public Endpoint {
public:
    UdpEndpoint(std::string name);
    ~UdpEndpoint() override;

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    bool setup(UdpEndpointConfig config); ///< open socket and apply config

    static const ConfFile::OptionsTable option_table[];
    static const char *section_pattern;
    static int parse_udp_mode(const char *val, size_t val_len, void *storage, size_t storage_len);
    static bool validate_config(const UdpEndpointConfig &config);

protected:
    bool open(const char *ip, unsigned long port,
              UdpEndpointConfig::Mode mode = UdpEndpointConfig::Mode::Client);
    int open_ipv4(const char *ip, unsigned long port, UdpEndpointConfig::Mode mode);
    int open_ipv6(const char *ip, unsigned long port, UdpEndpointConfig::Mode mode);

    ssize_t _read_msg(uint8_t *buf, size_t len) override;

    union {
        struct sockaddr_in v4;
        struct sockaddr_in6 v6;
    } config_sock;

    Timeout *nomessage_timeout = nullptr;
    bool _nomessage_timeout_cb(void *data);

private:
    bool is_ipv6;
    struct sockaddr_in sockaddr;
    struct sockaddr_in6 sockaddr6;
};

class TcpEndpoint : public Endpoint {
public:
    TcpEndpoint(std::string name);
    ~TcpEndpoint() override;

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }
    bool is_valid() override { return _valid; };
    bool is_critical() override { return false; };

    Endpoint::AcceptState accept_msg(const struct buffer *pbuf) const;

    int accept(int listener_fd);        ///< accept incoming connection
    bool setup(TcpEndpointConfig conf); ///< open connection and apply config
    bool reopen();                      ///< re-try connecting to the server
    void close();

    static const ConfFile::OptionsTable option_table[];
    static const char *section_pattern;
    static bool validate_config(const TcpEndpointConfig &config);

protected:
    bool open(const std::string &ip, unsigned long port);
    static int open_ipv4(const char *ip, unsigned long port, sockaddr_in &sockaddr);
    static int open_ipv6(const char *ip, unsigned long port, sockaddr_in6 &sockaddr6);

    ssize_t _read_msg(uint8_t *buf, size_t len) override;

    void _schedule_reconnect();
    bool _retry_timeout_cb(void *data);

private:
    std::string _ip{};
    unsigned long _port = 0;
    bool _valid = true;

    bool is_ipv6;
    int _retry_timeout = 0; // disable retry by default
    struct sockaddr_in sockaddr;
    struct sockaddr_in6 sockaddr6;
};
