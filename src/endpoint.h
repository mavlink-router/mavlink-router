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
    std::vector<speed_t> baudrates;
    bool flowcontrol{false};
    std::vector<uint8_t> allow_msg_id_out;
};

struct UdpEndpointConfig {
    enum class Mode { Undefined = 0, Server, Client };

    std::string name;
    std::string address;
    unsigned long port;
    Mode mode;
    std::vector<uint8_t> allow_msg_id_out;
};

struct TcpEndpointConfig {
    std::string name;
    std::string address;
    unsigned long port;
    int retry_timeout;
    std::vector<uint8_t> allow_msg_id_out;
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

    void filter_add_allowed_msg_id(uint32_t msg_id) { _allowed_msg_ids.push_back(msg_id); }

    std::string get_type() const { return this->_type; }

    struct buffer rx_buf;
    struct buffer tx_buf;

protected:
    virtual int read_msg(struct buffer *pbuf);
    virtual ssize_t _read_msg(uint8_t *buf, size_t len) = 0;
    bool _check_crc(const mavlink_msg_entry_t *msg_entry) const;
    void _add_sys_comp_id(uint8_t sysid, uint8_t compid);

    const std::string _type; ///< UART, UDP, TCP, Log
    std::string _name;       ///< Endpoint name from config file
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
    std::vector<uint32_t> _allowed_msg_ids;
};

class UartEndpoint : public Endpoint {
public:
    UartEndpoint(std::string name);
    ~UartEndpoint() override;
    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    bool setup(UartEndpointConfig config); ///< open UART device and apply config

    static const ConfFile::OptionsTable option_table[4];
    static const char *section_pattern;
    static int parse_baudrates(const char *val, size_t val_len, void *storage, size_t storage_len);
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
    ~UdpEndpoint() override = default;

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    bool setup(UdpEndpointConfig config); ///< open socket and apply config

    struct sockaddr_in sockaddr;
    struct sockaddr_in6 sockaddr6;

    bool ipv6;

    static const ConfFile::OptionsTable option_table[5];
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
};

class TcpEndpoint : public Endpoint {
public:
    TcpEndpoint(std::string name);
    ~TcpEndpoint() override;

    int accept(int listener_fd);        ///< accept incoming connection
    bool setup(TcpEndpointConfig conf); ///< open connection and apply config
    bool reopen();                      ///< re-try connecting to the server
    void close();

    int write_msg(const struct buffer *pbuf) override;
    int flush_pending_msgs() override { return -ENOSYS; }

    struct sockaddr_in sockaddr;
    struct sockaddr_in6 sockaddr6;
    bool ipv6;
    int retry_timeout = 0; // disable retry by default

    inline std::string get_ip() { return _ip; }
    inline unsigned long get_port() const { return _port; }
    bool is_valid() override { return _valid; };
    bool is_critical() override { return false; };

    static const ConfFile::OptionsTable option_table[4];
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
};
