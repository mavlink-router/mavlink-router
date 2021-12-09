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
#include "endpoint.h"

#include <algorithm>
#include <utility>

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <linux/serial.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <common/log.h>
#include <common/util.h>
#include <common/xtermios.h>

#include "mainloop.h"

#define RX_BUF_MAX_SIZE (MAVLINK_MAX_PACKET_LEN * 4)
#define TX_BUF_MAX_SIZE (8U * 1024U)

#define UART_BAUD_RETRY_SEC 5

static bool is_ipv6(const char *ip)
{
    /* Square brackets always exist on IPv6 addresses b/c of input validation */
    return strchr(ip, '[') != nullptr;
}

static bool ipv6_is_linklocal(const char *ip)
{
    /* link-local addresses start with fe80 */
    return strncmp(ip, "fe80", 4) == 0;
}

static bool ipv6_is_multicast(const char *ip)
{
    /* link-local addresses start with ff0x */
    return strncmp(ip, "ff0", 3) == 0;
}

static unsigned int ipv6_get_scope_id(const char *ip)
{
    struct ifaddrs *addrs;
    char ipAddress[NI_MAXHOST];
    unsigned scope = 0;
    getifaddrs(&addrs);

    /* search for our address in all interface addresses */
    for (ifaddrs *addr = addrs; addr; addr = addr->ifa_next) {
        if (addr->ifa_addr && addr->ifa_addr->sa_family == AF_INET6) {
            getnameinfo(addr->ifa_addr, sizeof(struct sockaddr_in6), ipAddress, sizeof(ipAddress),
                        nullptr, 0, NI_NUMERICHOST);

            /* cut the interface name from the end of a link-local address */
            auto *search = strrchr(ipAddress, '%');
            if (search != nullptr) {
                *search = '\0';
            }

            /* convert to a scope ID, if it's our interface */
            if (strcmp(ipAddress, ip) == 0) {
                scope = if_nametoindex(addr->ifa_name);
                break;
            }
        }
    }

    freeifaddrs(addrs);
    return scope;
}

Endpoint::Endpoint(std::string type, std::string name)
    : _type{std::move(type)}
    , _name{std::move(name)}
{
    rx_buf.data = (uint8_t *)malloc(RX_BUF_MAX_SIZE);
    rx_buf.len = 0;
    tx_buf.data = (uint8_t *)malloc(TX_BUF_MAX_SIZE);
    tx_buf.len = 0;

    assert(rx_buf.data);
    assert(tx_buf.data);
}

Endpoint::~Endpoint()
{
    free(rx_buf.data);
    free(tx_buf.data);
}

bool Endpoint::handle_canwrite()
{
    int r = flush_pending_msgs();
    return r == -EAGAIN;
}

int Endpoint::handle_read()
{
    int target_sysid, target_compid, r;
    uint8_t src_sysid, src_compid;
    uint32_t msg_id;
    struct buffer buf {
    };

    while ((r = read_msg(&buf, &target_sysid, &target_compid, &src_sysid, &src_compid, &msg_id))
           > 0) {
        Mainloop::get_instance().route_msg(&buf, target_sysid, target_compid, src_sysid, src_compid,
                                           msg_id);
    }

    return r;
}

int Endpoint::read_msg(struct buffer *pbuf, int *target_sysid, int *target_compid,
                       uint8_t *src_sysid, uint8_t *src_compid, uint32_t *msg_id)
{
    bool should_read_more = true;
    const mavlink_msg_entry_t *msg_entry;
    uint8_t *payload, seq, payload_len;

    if (fd < 0) {
        log_error("%s %s: Trying to read invalid fd", _type.c_str(), _name.c_str());
        return -EINVAL;
    }

    if (_last_packet_len != 0) {
        /*
         * read_msg() should be called in a loop after writting to each
         * output. However we don't want to keep busy looping on a single
         * endpoint reading more data. If we left data behind, move them
         * to the beginning and check we have a complete packet, but don't
         * read more data right now - it will be handled on next
         * iteration when more data is available
         */
        should_read_more = false;

        /* see TODO below about using bigger buffers: we could just walk on
         * the buffer rather than moving bytes */
        rx_buf.len -= _last_packet_len;
        if (rx_buf.len > 0) {
            memmove(rx_buf.data, rx_buf.data + _last_packet_len, rx_buf.len);
        }

        _last_packet_len = 0;
    }

    if (should_read_more) {
        ssize_t r = _read_msg(rx_buf.data + rx_buf.len, RX_BUF_MAX_SIZE - rx_buf.len);
        if (r <= 0) {
            return r;
        }

        log_debug("%s [%d]%s: got %zd bytes", _type.c_str(), fd, _name.c_str(), r);
        rx_buf.len += r;
    }

    bool mavlink2 = rx_buf.data[0] == MAVLINK_STX;
    bool mavlink1 = rx_buf.data[0] == MAVLINK_STX_MAVLINK1;

    /*
     * Find magic byte as the start byte:
     *
     * we either enter here due to new bytes being written to the
     * beginning of the buffer or due to _last_packet_len not being 0
     * above, which means we moved some bytes we read previously
     */
    if (!mavlink1 && !mavlink2) {
        unsigned int stx_pos = 0;

        for (unsigned int i = 1; i < (unsigned int)rx_buf.len; i++) {
            if (rx_buf.data[i] == MAVLINK_STX) {
                mavlink2 = true;
            } else if (rx_buf.data[i] == MAVLINK_STX_MAVLINK1) {
                mavlink1 = true;
            }

            if (mavlink1 || mavlink2) {
                stx_pos = i;
                break;
            }
        }

        /* Discarding data since we don't have a marker */
        if (stx_pos == 0) {
            rx_buf.len = 0;
            return 0;
        }

        /*
         * TODO: a larger buffer would allow to avoid the memmove in case a
         * new message would still fit in our buffer
         */
        rx_buf.len -= stx_pos;
        memmove(rx_buf.data, rx_buf.data + stx_pos, rx_buf.len);
    }

    const uint8_t checksum_len = 2;
    size_t expected_size;

    if (mavlink2) {
        auto *hdr = (struct mavlink_router_mavlink2_header *)rx_buf.data;

        if (rx_buf.len < sizeof(*hdr)) {
            return 0;
        }

        *msg_id = hdr->msgid;
        payload = rx_buf.data + sizeof(*hdr);
        seq = hdr->seq;
        *src_sysid = hdr->sysid;
        *src_compid = hdr->compid;
        payload_len = hdr->payload_len;

        expected_size = sizeof(*hdr);
        expected_size += hdr->payload_len;
        expected_size += checksum_len;
        if (hdr->incompat_flags & MAVLINK_IFLAG_SIGNED) {
            expected_size += MAVLINK_SIGNATURE_BLOCK_LEN;
        }
    } else {
        auto *hdr = (struct mavlink_router_mavlink1_header *)rx_buf.data;

        if (rx_buf.len < sizeof(*hdr)) {
            return 0;
        }

        *msg_id = hdr->msgid;
        payload = rx_buf.data + sizeof(*hdr);
        seq = hdr->seq;
        *src_sysid = hdr->sysid;
        *src_compid = hdr->compid;
        payload_len = hdr->payload_len;

        expected_size = sizeof(*hdr);
        expected_size += hdr->payload_len;
        expected_size += checksum_len;
    }

    /* check if we have a valid mavlink packet */
    if (rx_buf.len < expected_size) {
        return 0;
    }

    /* We always want to transmit one packet at a time; record the number
     * of bytes read in addition to the expected size and leave them for
     * the next iteration */
    _last_packet_len = expected_size;
    _stat.read.total++;

    msg_entry = mavlink_get_msg_entry(*msg_id);
    if (msg_entry) {
        /*
         * It is accepting and forwarding unknown messages ids because
         * it can be a new MAVLink message implemented only in
         * Ground Station and Flight Stack. Although it can also be a
         * corrupted message is better forward than silent drop it.
         */
        if (!_check_crc(msg_entry)) {
            _stat.read.crc_error++;
            _stat.read.crc_error_bytes += expected_size;
            return 0;
        }
        _add_sys_comp_id(((uint16_t)*src_sysid << 8) | *src_compid);
    }

    _stat.read.handled++;
    _stat.read.handled_bytes += expected_size;

    *target_sysid = -1;
    *target_compid = -1;

    if (msg_entry == nullptr) {
        log_debug("%s [%d]%s: No message entry for %u", _type.c_str(), fd, _name.c_str(), *msg_id);
    } else {
        if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) {
            // if target_system is 0, it may have been trimmed out on mavlink2
            if (msg_entry->target_system_ofs < payload_len) {
                *target_sysid = payload[msg_entry->target_system_ofs];
            } else {
                *target_sysid = 0;
            }
        }
        if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT) {
            // if target_system is 0, it may have been trimmed out on mavlink2
            if (msg_entry->target_component_ofs < payload_len) {
                *target_compid = payload[msg_entry->target_component_ofs];
            } else {
                *target_compid = 0;
            }
        }
        *msg_id = msg_entry->msgid;
    }

    // Check for sequence drops
    if (_stat.read.expected_seq != seq) {
        if (_stat.read.total > 1) {
            uint8_t diff;

            if (seq > _stat.read.expected_seq) {
                diff = (seq - _stat.read.expected_seq);
            } else {
                diff = (UINT8_MAX - _stat.read.expected_seq) + seq;
            }

            _stat.read.drop_seq_total += diff;
            _stat.read.total += diff;
        }
        _stat.read.expected_seq = seq;
    }
    _stat.read.expected_seq++;

    pbuf->data = rx_buf.data;
    pbuf->len = expected_size;

    return msg_entry != nullptr ? ReadOk : ReadUnkownMsg;
}

void Endpoint::_add_sys_comp_id(uint16_t sys_comp_id)
{
    if (has_sys_comp_id(sys_comp_id)) {
        return;
    }

    _sys_comp_ids.push_back(sys_comp_id);
}

bool Endpoint::has_sys_id(unsigned sysid) const
{
    for (const auto &id : _sys_comp_ids) {
        if (((id >> 8) | (sysid & 0xff)) == sysid) {
            return true;
        }
    }
    return false;
}

bool Endpoint::has_sys_comp_id(unsigned sys_comp_id) const
{
    for (const auto &id : _sys_comp_ids) {
        if (sys_comp_id == id) {
            return true;
        }
    }

    return false;
}

Endpoint::AcceptState Endpoint::accept_msg(int target_sysid, int target_compid, uint8_t src_sysid,
                                           uint8_t src_compid, uint32_t msg_id) const
{
    if (Log::get_max_level() >= Log::Level::DEBUG) {
        log_debug("Endpoint [%d]%s: got message %u to %d/%d from %u/%u", fd, _name.c_str(), msg_id,
                  target_sysid, target_compid, src_sysid, src_compid);
        log_debug("\tKnown components:");
        for (const auto &id : _sys_comp_ids) {
            log_debug("\t\t%u/%u", (id >> 8), id & 0xff);
        }
    }

    // This endpoint sent the message, we don't want to send it back over the
    // same channel to avoid loops: reject
    if (has_sys_comp_id(src_sysid, src_compid)) {
        return Endpoint::AcceptState::Rejected;
    }

    // If filter is defined and message is not in the set: discard it
    if (msg_id != UINT32_MAX && !_allowed_msg_ids.empty()
        && !vector_contains(_allowed_msg_ids, msg_id)) {
        return Endpoint::AcceptState::Filtered;
    }

    // Message is broadcast on sysid or sysid is non-existent: accept msg
    if (target_sysid == 0 || target_sysid == -1) {
        return Endpoint::AcceptState::Accepted;
    }

    // This endpoint has the target of message (sys and comp id): accept
    if (target_compid > 0 && has_sys_comp_id(target_sysid, target_compid)) {
        return Endpoint::AcceptState::Accepted;
    }

    // This endpoint has the target of message (sysid, but compid is broadcast or non-existent):
    // accept
    if ((target_compid == 0 || target_compid == -1) && has_sys_id(target_sysid)) {
        return Endpoint::AcceptState::Accepted;
    }

    // Reject everything else
    return Endpoint::AcceptState::Rejected;
}

bool Endpoint::_check_crc(const mavlink_msg_entry_t *msg_entry) const
{
    const bool mavlink2 = rx_buf.data[0] == MAVLINK_STX;
    uint16_t crc_msg, crc_calc;
    uint8_t payload_len, header_len, *payload;

    if (mavlink2) {
        auto *hdr = (struct mavlink_router_mavlink2_header *)rx_buf.data;
        payload = rx_buf.data + sizeof(*hdr);
        header_len = sizeof(*hdr);
        payload_len = hdr->payload_len;
    } else {
        auto *hdr = (struct mavlink_router_mavlink1_header *)rx_buf.data;
        payload = rx_buf.data + sizeof(*hdr);
        header_len = sizeof(*hdr);
        payload_len = hdr->payload_len;
    }

    crc_msg = payload[payload_len] | (payload[payload_len + 1] << 8);
    crc_calc = crc_calculate(&rx_buf.data[1], header_len + payload_len - 1);
    crc_accumulate(msg_entry->crc_extra, &crc_calc);
    return crc_calc == crc_msg;
}

void Endpoint::print_statistics()
{
    const uint32_t read_total = _stat.read.total == 0 ? 1 : _stat.read.total;

    printf("%s Endpoint [%d]%s {", _type.c_str(), fd, _name.c_str());
    printf("\n\tReceived messages {");
    printf("\n\t\tCRC error: %u %u%% %luKBytes", _stat.read.crc_error,
           (_stat.read.crc_error * 100) / read_total, _stat.read.crc_error_bytes / 1000);
    printf("\n\t\tSequence lost: %u %u%%", _stat.read.drop_seq_total,
           (_stat.read.drop_seq_total * 100) / read_total);
    printf("\n\t\tHandled: %u %luKBytes", _stat.read.handled, _stat.read.handled_bytes / 1000);
    printf("\n\t\tTotal: %u", _stat.read.total);
    printf("\n\t}");
    printf("\n\tTransmitted messages {");
    printf("\n\t\tTotal: %u %luKBytes", _stat.write.total, _stat.write.bytes / 1000);
    printf("\n\t}");
    printf("\n}\n");
}

uint8_t Endpoint::get_trimmed_zeros(const mavlink_msg_entry_t *msg_entry,
                                    const struct buffer *buffer)
{
    auto *msg = (struct mavlink_router_mavlink2_header *)buffer->data;

    /* Only MAVLink 2 trim zeros */
    if (buffer->data[0] != MAVLINK_STX) {
        return 0;
    }

    /* Should never happen but if happens it will cause stack overflow */
    if (msg->payload_len > msg_entry->max_msg_len) {
        return 0;
    }

    return msg_entry->max_msg_len - msg->payload_len;
}

void Endpoint::log_aggregate(unsigned int interval_sec)
{
    if (_incomplete_msgs > 0) {
        log_warning("%s Endpoint [%d]%s: %u incomplete messages in the last %d seconds",
                    _type.c_str(), fd, _name.c_str(), _incomplete_msgs, interval_sec);
        _incomplete_msgs = 0;
    }
}

UartEndpoint::~UartEndpoint()
{
    if (fd > 0) {
        reset_uart(fd);
    }
}

int UartEndpoint::set_speed(speed_t baudrate)
{
    struct termios2 tc;

    if (fd < 0) {
        return -1;
    }

    bzero(&tc, sizeof(tc));
    if (ioctl(fd, TCGETS2, &tc) == -1) {
        log_error("UART [%d]%s: Could not get termios2 (%m)", fd, _name.c_str());
        return -1;
    }

    /* speed is configured by c_[io]speed */
    tc.c_cflag &= ~CBAUD;
    tc.c_cflag |= BOTHER;
    tc.c_ispeed = baudrate;
    tc.c_ospeed = baudrate;

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        log_error("Could not set terminal attributes (%m)");
        return -1;
    }

    log_info("UART [%d]%s: speed = %u", fd, _name.c_str(), baudrate);

    if (ioctl(fd, TCFLSH, TCIOFLUSH) == -1) {
        log_error("UART [%d]%s: Could not flush terminal (%m)", fd, _name.c_str());
        return -1;
    }

    return 0;
}

int UartEndpoint::set_flow_control(bool enabled)
{
    struct termios2 tc;

    if (fd < 0) {
        return -1;
    }

    bzero(&tc, sizeof(tc));
    if (ioctl(fd, TCGETS2, &tc) == -1) {
        log_error("UART [%d]%s: Could not get termios2 (%m)", fd, _name.c_str());
        return -1;
    }

    if (enabled) {
        tc.c_cflag |= CRTSCTS;
    } else {
        tc.c_cflag &= ~CRTSCTS;
    }

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        log_error("UART [%d]%s: Could not set terminal attributes (%m)", fd, _name.c_str());
        return -1;
    }

    log_info("UART [%d]%s: flowcontrol = %s", fd, _name.c_str(), enabled ? "enabled" : "disabled");

    return 0;
}

int UartEndpoint::open(const char *path)
{
    struct termios2 tc;

    fd = ::open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC | O_NOCTTY);
    if (fd < 0) {
        log_error("Could not open %s (%m)", path);
        return -1;
    }

    if (reset_uart(fd) < 0) {
        log_error("Could not reset uart on %s", path);
        goto fail;
    }

    bzero(&tc, sizeof(tc));

    if (ioctl(fd, TCGETS2, &tc) == -1) {
        log_error("Could not get termios2 on %s (%m)", path);
        goto fail;
    }

    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

    tc.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | ECHONL | ICANON | IEXTEN | ISIG);

    /* never send SIGTTOU*/
    tc.c_lflag &= ~(TOSTOP);

    /* disable flow control */
    tc.c_cflag &= ~(CRTSCTS);
    tc.c_cflag &= ~(CSIZE | PARENB);

    /* ignore modem control lines */
    tc.c_cflag |= CLOCAL;

    /* 8 bits */
    tc.c_cflag |= CS8;

    /* we use epoll to get notification of available bytes */
    tc.c_cc[VMIN] = 0;
    tc.c_cc[VTIME] = 0;

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        log_error("Could not set terminal attributes on %s (%m)", path);
        goto fail;
    }

    // For Linux, set high speed polling at the chip
    // level. Since this routine relies on a USB latency
    // change at the chip level it may fail on certain
    // chip sets if their driver does not support this
    // configuration request

    {
        struct serial_struct serial_ctl;

        int result = ioctl(fd, TIOCGSERIAL, &serial_ctl);
        if (result < 0) {
            log_warning("Error while trying to read serial port configuration on %s: %m", path);
            goto set_latency_failed;
        }

        serial_ctl.flags |= ASYNC_LOW_LATENCY;

        result = ioctl(fd, TIOCSSERIAL, &serial_ctl);
        if (result < 0) {
            if (errno != ENODEV && errno != ENOTTY) {
                log_warning("Error while trying to write serial port latency on %s: %m", path);
            }
        }
    }

set_latency_failed:
    if (ioctl(fd, TCFLSH, TCIOFLUSH) == -1) {
        log_error("Could not flush terminal on %s (%m)", path);
        goto fail;
    }

    log_info("Opened UART [%d]%s: %s", fd, _name.c_str(), path);

    return fd;

fail:
    ::close(fd);
    fd = -1;
    return -1;
}

bool UartEndpoint::_change_baud_cb(void *data)
{
    _current_baud_idx = (_current_baud_idx + 1) % _baudrates.size();

    log_info("Retrying UART [%d]%s on new baudrate: %u", fd, _name.c_str(),
             _baudrates[_current_baud_idx]);

    set_speed(_baudrates[_current_baud_idx]);

    return true;
}

int UartEndpoint::read_msg(struct buffer *pbuf, int *target_sysid, int *target_compid,
                           uint8_t *src_sysid, uint8_t *src_compid, uint32_t *msg_id)
{
    int ret = Endpoint::read_msg(pbuf, target_sysid, target_compid, src_sysid, src_compid, msg_id);

    if (_change_baud_timeout != nullptr && ret == ReadOk) {
        log_info("%s [%d]%s: Baudrate %u responded, keeping it", _type.c_str(), fd, _name.c_str(),
                 _baudrates[_current_baud_idx]);
        Mainloop::get_instance().del_timeout(_change_baud_timeout);
        _change_baud_timeout = nullptr;
    }

    return ret;
}

ssize_t UartEndpoint::_read_msg(uint8_t *buf, size_t len)
{
    ssize_t r = ::read(fd, buf, len);
    if ((r == -1 && errno == EAGAIN) || r == 0) {
        return 0;
    }
    if (r == -1) {
        return -errno;
    }

    return r;
}

int UartEndpoint::write_msg(const struct buffer *pbuf)
{
    if (fd < 0) {
        log_error("UART %s: Trying to write invalid fd", _name.c_str());
        return -EINVAL;
    }

    /* TODO: send any pending data */
    if (tx_buf.len > 0) {
        ;
    }

    ssize_t r = ::write(fd, pbuf->data, pbuf->len);
    if (r == -1 && errno == EAGAIN) {
        return -EAGAIN;
    }

    _stat.write.total++;
    _stat.write.bytes += pbuf->len;

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t)pbuf->len) {
        _incomplete_msgs++;
        log_debug("UART %s: Discarding packet, incomplete write %zd but len=%u", _name.c_str(), r,
                  pbuf->len);
    }

    log_debug("UART [%d]%s: Wrote %zd bytes", fd, _name.c_str(), r);

    return r;
}

int UartEndpoint::add_speeds(const std::vector<speed_t> &bauds)
{
    if (bauds.empty()) {
        return -EINVAL;
    }

    _baudrates = bauds;

    set_speed(_baudrates[0]);

    _change_baud_timeout = Mainloop::get_instance().add_timeout(
        MSEC_PER_SEC * UART_BAUD_RETRY_SEC,
        std::bind(&UartEndpoint::_change_baud_cb, this, std::placeholders::_1), this);

    return 0;
}

UdpEndpoint::UdpEndpoint(std::string name)
    : Endpoint{ENDPOINT_TYPE_UDP, std::move(name)}
{
    bzero(&sockaddr, sizeof(sockaddr));
    bzero(&sockaddr6, sizeof(sockaddr6));
}

int UdpEndpoint::open_ipv6(const char *ip, unsigned long port, UdpEndpointConfig::Mode mode)
{
    fd = socket(AF_INET6, SOCK_DGRAM, 0);
    if (fd < 0) {
        log_error("Could not create IPv6 socket for %s:%lu (%m)", ip, port);
        return -errno;
    }

    /* strip square brackets from ip string */
    char *ip_str = strdup(&ip[1]);
    ip_str[strlen(ip_str) - 1] = '\0';

    sockaddr6.sin6_family = AF_INET6;
    sockaddr6.sin6_port = htons(port);

    /* multicast address needs to listen to all, but "filter" incoming packets */
    if (mode == UdpEndpointConfig::Mode::Server && ipv6_is_multicast(ip_str)) {
        sockaddr6.sin6_addr = in6addr_any;

        struct ipv6_mreq group;
        inet_pton(AF_INET6, ip_str, &group.ipv6mr_multiaddr);
        if (setsockopt(fd, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, &group, sizeof(group)) < 0) {
            log_error("Error setting IPv6 multicast socket options for %s:%lu (%m)", ip, port);
            goto fail;
        }
    } else {
        inet_pton(AF_INET6, ip_str, &sockaddr6.sin6_addr);
    }

    /* link-local address needs a scope ID */
    if (ipv6_is_linklocal(ip_str)) {
        sockaddr6.sin6_scope_id = ipv6_get_scope_id(ip_str);
    }

    if (mode == UdpEndpointConfig::Mode::Server) {
        if (bind(fd, (struct sockaddr *)&sockaddr6, sizeof(sockaddr6)) < 0) {
            log_error("Error binding IPv6 socket for %s:%lu (%m)", ip, port);
            goto fail;
        }
        sockaddr6.sin6_port = 0;
    }

    free(ip_str);
    return fd;

fail:
    free(ip_str);
    ::close(fd);
    fd = -1;
    return -EINVAL;
}

int UdpEndpoint::open_ipv4(const char *ip, unsigned long port, UdpEndpointConfig::Mode mode)
{
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        log_error("Could not create IPv4 socket for %s:%lu (%m)", ip, port);
        return -errno;
    }

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(ip);
    sockaddr.sin_port = htons(port);

    if (mode == UdpEndpointConfig::Mode::Server) {
        if (bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
            log_error("Error binding IPv4 socket for %s:%lu (%m)", ip, port);
            goto fail;
        }
        sockaddr.sin_port = 0;
    }

    return fd;

fail:
    ::close(fd);
    fd = -1;
    return -EINVAL;
}

int UdpEndpoint::open(const char *ip, unsigned long port, UdpEndpointConfig::Mode mode)
{
    const int broadcast_val = 1;

    this->ipv6 = is_ipv6(ip);

    // setup the special ipv6/ipv4 part
    if (this->ipv6) {
        open_ipv6(ip, port, mode);
    } else {
        open_ipv4(ip, port, mode);
    }

    if (fd < 0) {
        return -1;
    }

    // common setup
    if (mode == UdpEndpointConfig::Mode::Client) {
        if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast_val, sizeof(broadcast_val))) {
            log_error("Error enabling broadcast in socket for %s:%lu (%m)", ip, port);
            goto fail;
        }
    }

    if (fcntl(fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        log_error("Error setting socket fd as non-blocking for %s:%lu (%m)", ip, port);
        goto fail;
    }

    if (mode == UdpEndpointConfig::Mode::Server) {
        log_info("Opened UDP Server [%d]%s: %s:%lu", fd, _name.c_str(), ip, port);
    } else {
        log_info("Opened UDP Client [%d]%s: %s:%lu", fd, _name.c_str(), ip, port);
    }

    return fd;

fail:
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
    return -1;
}

ssize_t UdpEndpoint::_read_msg(uint8_t *buf, size_t len)
{
    socklen_t addrlen;
    struct sockaddr *sock;
    ssize_t r = 0;

    if (this->ipv6) {
        addrlen = sizeof(sockaddr6);
        sock = (struct sockaddr *)&sockaddr6;
    } else {
        addrlen = sizeof(sockaddr);
        sock = (struct sockaddr *)&sockaddr;
    }

    r = ::recvfrom(fd, buf, len, 0, sock, &addrlen);
    if (r == -1 && errno == EAGAIN) {
        return 0;
    }
    if (r == -1) {
        return -errno;
    }

    return r;
}

int UdpEndpoint::write_msg(const struct buffer *pbuf)
{
    struct sockaddr *sock;
    socklen_t addrlen;

    if (fd < 0) {
        log_error("UDP %s: Trying to write invalid fd", _name.c_str());
        return -EINVAL;
    }

    /* TODO: send any pending data */
    if (tx_buf.len > 0) {
        ;
    }

    bool sock_connected = false;
    if (this->ipv6) {
        addrlen = sizeof(sockaddr6);
        sock = (struct sockaddr *)&sockaddr6;
        sock_connected = sockaddr6.sin6_port != 0;
    } else {
        addrlen = sizeof(sockaddr);
        sock = (struct sockaddr *)&sockaddr;
        sock_connected = sockaddr.sin_port != 0;
    }

    if (!sock_connected) {
        log_debug("UDP %s: No one ever connected to us. No one to write for", _name.c_str());
        return 0;
    }

    ssize_t r = ::sendto(fd, pbuf->data, pbuf->len, 0, sock, addrlen);
    if (r == -1) {
        if (errno != EAGAIN && errno != ECONNREFUSED && errno != ENETUNREACH) {
            log_error("UDP %s: Error sending udp packet (%m)", _name.c_str());
        }
        return -errno;
    };

    _stat.write.total++;
    _stat.write.bytes += pbuf->len;

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t)pbuf->len) {
        _incomplete_msgs++;
        log_debug("UDP %s: Discarding packet, incomplete write %zd but len=%u", _name.c_str(), r,
                  pbuf->len);
    }

    log_debug("UDP [%d]%s: Wrote %zd bytes", fd, _name.c_str(), r);

    return r;
}

TcpEndpoint::TcpEndpoint(std::string name)
    : Endpoint{ENDPOINT_TYPE_TCP, std::move(name)}
{
    bzero(&sockaddr, sizeof(sockaddr));
    bzero(&sockaddr6, sizeof(sockaddr6));
}

TcpEndpoint::~TcpEndpoint()
{
    close();
}

int TcpEndpoint::accept(int listener_fd)
{
    struct sockaddr *sock;
    socklen_t addrlen;

    if (this->ipv6) {
        addrlen = sizeof(sockaddr6);
        sock = (struct sockaddr *)&sockaddr6;
    } else {
        addrlen = sizeof(sockaddr);
        sock = (struct sockaddr *)&sockaddr;
    }

    fd = accept4(listener_fd, sock, &addrlen, SOCK_NONBLOCK);
    if (fd == -1) {
        return -1;
    }

    log_info("TCP [%d]%s: Connection accepted", fd, _name.c_str());

    int tcp_nodelay_state = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (char *)&tcp_nodelay_state, sizeof(int)) < 0) {
        log_error("Error setting TCP_NODELAY on [%d]%s", fd, _name.c_str());
        return -1;
    }

    return fd;
}

int TcpEndpoint::open_ipv6(const char *ip, unsigned long port)
{
    fd = socket(AF_INET6, SOCK_STREAM, 0);
    if (fd == -1) {
        log_error("Could not create IPv6 socket for %s:%lu (%m)", ip, port);
        return -1;
    }

    /* strip square brackets from ip string */
    char *ip_str = strdup(&_ip[1]);
    ip_str[strlen(ip_str) - 1] = '\0';

    /* multicast address is not allowed for TCP sockets */
    if (ipv6_is_multicast(ip_str)) {
        log_error("TCP endpoints do not support multicast address");
        goto fail;
    }

    sockaddr6.sin6_family = AF_INET6;
    sockaddr6.sin6_port = htons(port);
    inet_pton(AF_INET6, ip_str, &sockaddr6.sin6_addr);

    /* link-local address needs a scope ID */
    if (ipv6_is_linklocal(ip_str)) {
        sockaddr6.sin6_scope_id = ipv6_get_scope_id(ip_str);
    }

    free(ip_str);

    return fd;

fail:
    free(ip_str);
    fd = -1;
    return fd;
}

int TcpEndpoint::open_ipv4(const char *ip, unsigned long port)
{
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == -1) {
        log_error("Could not create IPv4 socket for %s:%lu (%m)", ip, port);
        return -1;
    }

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(ip);
    sockaddr.sin_port = htons(port);

    return fd;
}

int TcpEndpoint::open(const std::string &ip, unsigned long port)
{
    // save IP and port for retry callback
    _ip = ip;
    _port = port;

    this->ipv6 = is_ipv6(ip.c_str());

    // setup the special ipv6/ipv4 part
    struct sockaddr *sock;
    socklen_t addrlen;
    if (this->ipv6) {
        open_ipv6(ip.c_str(), port);
        sock = (struct sockaddr *)&this->sockaddr6;
        addrlen = sizeof(sockaddr6);
    } else {
        open_ipv4(ip.c_str(), port);
        sock = (struct sockaddr *)&this->sockaddr;
        addrlen = sizeof(sockaddr);
    }

    if (fd < 0) {
        return -1;
    }

    // common setup
    if (connect(fd, sock, addrlen) < 0) {
        log_error("Error connecting to %s:%lu (%m)", ip.c_str(), port);
        goto fail;
    }

    if (fcntl(fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        log_error("Error setting socket fd as non-blocking for %s:%lu (%m)", ip.c_str(), port);
        goto fail;
    }

    log_info("Opened TCP Client [%d]%s: %s:%lu", fd, _name.c_str(), ip.c_str(), port);

    _valid = true;
    return fd;

fail:
    ::close(fd);
    return -1;
}

ssize_t TcpEndpoint::_read_msg(uint8_t *buf, size_t len)
{
    struct sockaddr *sock;
    socklen_t addrlen;
    ssize_t r;

    if (this->ipv6) {
        sock = (struct sockaddr *)&sockaddr6;
        addrlen = sizeof(sockaddr6);
    } else {
        sock = (struct sockaddr *)&sockaddr;
        addrlen = sizeof(sockaddr);
    }

    r = ::recvfrom(fd, buf, len, 0, sock, &addrlen);
    if (r == -1 && errno == EAGAIN) {
        return 0;
    }
    if (r == -1) {
        return -errno;
    }

    // a read of zero on a stream socket means that other side shut down
    if (r == 0 && len != 0) {
        _valid = false;
        return EOF; // TODO is EOF always negative?
    }

    return r;
}

int TcpEndpoint::write_msg(const struct buffer *pbuf)
{
    struct sockaddr *sock;
    socklen_t addrlen;

    if (fd < 0) {
        log_error("TCP %s: Trying to write invalid fd", _name.c_str());
        return -EINVAL;
    }

    /* TODO: send any pending data */
    if (tx_buf.len > 0) {
        ;
    }

    if (this->ipv6) {
        sock = (struct sockaddr *)&sockaddr6;
        addrlen = sizeof(sockaddr6);
    } else {
        sock = (struct sockaddr *)&sockaddr;
        addrlen = sizeof(sockaddr);
    }

    ssize_t r = ::sendto(fd, pbuf->data, pbuf->len, 0, sock, addrlen);
    if (r == -1) {
        if (errno != EAGAIN && errno != ECONNREFUSED) {
            log_error("TCP %s: Error sending tcp packet (%m)", _name.c_str());
        }
        if (errno == EPIPE) {
            _valid = false;
        }
        return -errno;
    };

    _stat.write.total++;
    _stat.write.bytes += pbuf->len;

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t)pbuf->len) {
        _incomplete_msgs++;
        log_debug("TCP %s: Discarding packet, incomplete write %zd but len=%u", _name.c_str(), r,
                  pbuf->len);
    }

    log_debug("TCP [%d]%s: Wrote %zd bytes", fd, _name.c_str(), r);

    return r;
}

void TcpEndpoint::close()
{
    if (fd > -1) {
        ::close(fd);

        log_info("TCP [%d]%s: Connection closed", fd, _name.c_str());
    }

    fd = -1;
}
