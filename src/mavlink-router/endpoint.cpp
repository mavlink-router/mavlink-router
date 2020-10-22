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

#include <arpa/inet.h>
#include <algorithm>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <netdb.h>
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

#include <linux/serial.h>

#include "mainloop.h"

#define RX_BUF_MAX_SIZE (MAVLINK_MAX_PACKET_LEN * 4)
#define TX_BUF_MAX_SIZE (8U * 1024U)

#define UART_BAUD_RETRY_SEC 5

Endpoint::Endpoint(const char *name)
    : _name{name}
{
    rx_buf.data = (uint8_t *) malloc(RX_BUF_MAX_SIZE);
    rx_buf.len = 0;
    tx_buf.data = (uint8_t *) malloc(TX_BUF_MAX_SIZE);
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
    struct buffer buf{};

    while ((r = read_msg(&buf, &target_sysid, &target_compid, &src_sysid, &src_compid, &msg_id)) > 0)
        Mainloop::get_instance().route_msg(&buf, target_sysid, target_compid, src_sysid,
                                           src_compid, msg_id, this);

    return r;
}

int Endpoint::read_msg(struct buffer *pbuf, int *target_sysid, int *target_compid,
                       uint8_t *src_sysid, uint8_t *src_compid, uint32_t *msg_id)
{
    bool should_read_more = true;
    const mavlink_msg_entry_t *msg_entry;
    uint8_t *payload, seq, payload_len;

    if (fd < 0) {
        log_error("Trying to read invalid fd");
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
        if (r <= 0)
            return r;

        log_debug("%s [%d] got %zd bytes", _name, fd, r);
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

        for (unsigned int i = 1; i < (unsigned int) rx_buf.len; i++) {
            if (rx_buf.data[i] == MAVLINK_STX)
                mavlink2 = true;
            else if (rx_buf.data[i] == MAVLINK_STX_MAVLINK1)
                mavlink1 = true;

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
        struct mavlink_router_mavlink2_header *hdr =
                (struct mavlink_router_mavlink2_header *)rx_buf.data;

        if (rx_buf.len < sizeof(*hdr))
            return 0;

        *msg_id = hdr->msgid;
        payload = rx_buf.data + sizeof(*hdr);
        seq = hdr->seq;
        *src_sysid = hdr->sysid;
        *src_compid = hdr->compid;
        payload_len = hdr->payload_len;

        expected_size = sizeof(*hdr);
        expected_size += hdr->payload_len;
        expected_size += checksum_len;
        if (hdr->incompat_flags & MAVLINK_IFLAG_SIGNED)
            expected_size += MAVLINK_SIGNATURE_BLOCK_LEN;
    } else {
        struct mavlink_router_mavlink1_header *hdr =
                (struct mavlink_router_mavlink1_header *)rx_buf.data;

        if (rx_buf.len < sizeof(*hdr))
            return 0;

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
    if (rx_buf.len < expected_size)
        return 0;

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
        log_debug("No message entry for %u", *msg_id);
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

            if (seq > _stat.read.expected_seq)
                diff = (seq - _stat.read.expected_seq);
            else
                diff = (UINT8_MAX - _stat.read.expected_seq) + seq;

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
    if (has_sys_comp_id(sys_comp_id))
        return;

    _sys_comp_ids.push_back(sys_comp_id);
}

bool Endpoint::has_sys_id(unsigned sysid)
{
    for (auto it = _sys_comp_ids.begin(); it != _sys_comp_ids.end(); it++) {
        if (((*it >> 8) | (sysid & 0xff)) == sysid)
            return true;
    }
    return false;
}

bool Endpoint::has_sys_comp_id(unsigned sys_comp_id)
{
    for (auto it = _sys_comp_ids.begin(); it != _sys_comp_ids.end(); it++) {
        if (sys_comp_id == *it)
            return true;
    }

    return false;
}

bool Endpoint::accept_msg(int target_sysid, int target_compid, uint8_t src_sysid,
                          uint8_t src_compid, uint32_t msg_id)
{
    if (Log::get_max_level() >= Log::Level::DEBUG) {
        log_debug("Endpoint [%d] got message %u to %d/%d from %u/%u", fd, msg_id, target_sysid, target_compid,
                  src_sysid, src_compid);
        log_debug("\tKnown components:");
        for (auto it = _sys_comp_ids.begin(); it != _sys_comp_ids.end(); it++) {
            log_debug("\t\t%u/%u", (*it >> 8), *it & 0xff);
        }
    }

    // This endpoint sent the message, we don't want to send it back over the
    // same channel to avoid loops: reject
    if (has_sys_comp_id(src_sysid, src_compid))
        return false;

    if (msg_id != UINT32_MAX && 
        _message_filter.size() > 0 && 
        std::find(_message_filter.begin(), _message_filter.end(), msg_id) == _message_filter.end()) {

        // if filter is defined and message is not in the set then discard it
        return false;
    }

    // Message is broadcast on sysid or sysid is non-existent: accept msg
    if (target_sysid == 0 || target_sysid == -1)
        return true;

    // This endpoint has the target of message (sys and comp id): accept
    if (target_compid > 0 && has_sys_comp_id(target_sysid, target_compid))
        return true;

    // This endpoint has the target of message (sysid, but compid is broadcast or non-existent): accept
    if ((target_compid == 0 || target_compid == -1) && has_sys_id(target_sysid))
        return true;

    // Reject everything else
    return false;
}

bool Endpoint::_check_crc(const mavlink_msg_entry_t *msg_entry)
{
    const bool mavlink2 = rx_buf.data[0] == MAVLINK_STX;
    uint16_t crc_msg, crc_calc;
    uint8_t payload_len, header_len, *payload;

    if (mavlink2) {
        struct mavlink_router_mavlink2_header *hdr =
                    (struct mavlink_router_mavlink2_header *)rx_buf.data;
        payload = rx_buf.data + sizeof(*hdr);
        header_len = sizeof(*hdr);
        payload_len = hdr->payload_len;
    } else {
        struct mavlink_router_mavlink1_header *hdr =
                    (struct mavlink_router_mavlink1_header *)rx_buf.data;
        payload = rx_buf.data + sizeof(*hdr);
        header_len = sizeof(*hdr);
        payload_len = hdr->payload_len;
    }

    crc_msg = payload[payload_len] | (payload[payload_len + 1] << 8);
    crc_calc = crc_calculate(&rx_buf.data[1], header_len + payload_len - 1);
    crc_accumulate(msg_entry->crc_extra, &crc_calc);
    if (crc_calc != crc_msg) {
        return false;
    }

    return true;
}

void Endpoint::print_statistics()
{
    const uint32_t read_total = _stat.read.total == 0 ? 1 : _stat.read.total;

    printf("Endpoint %s [%d] {", _name, fd);
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

uint8_t Endpoint::get_trimmed_zeros(const mavlink_msg_entry_t *msg_entry, const struct buffer *buffer)
{
    struct mavlink_router_mavlink2_header *msg
        = (struct mavlink_router_mavlink2_header *)buffer->data;

    /* Only MAVLink 2 trim zeros */
    if (buffer->data[0] != MAVLINK_STX)
        return 0;

    /* Should never happen but if happens it will cause stack overflow */
    if (msg->payload_len > msg_entry->max_msg_len)
        return 0;

    return msg_entry->max_msg_len - msg->payload_len;
}

void Endpoint::log_aggregate(unsigned int interval_sec)
{
    if (_incomplete_msgs > 0) {
        log_warning("Endpoint %s [%d]: %u incomplete messages in the last %d seconds", _name, fd,
                    _incomplete_msgs, interval_sec);
        _incomplete_msgs = 0;
    }
}

bool Endpoint::ipv4_is_multicast(const char *ip)
{
    /* multicast addresses start with 224 to 239 */
    int ipFirstByte = atoi(ip);
    if (224 <= ipFirstByte && ipFirstByte <= 239) {
        return true;
    }
    return false;
}

#ifdef ENABLE_IPV6
bool Endpoint::is_ipv6(const char *ip)
{
    /* Square brackets always exist on IPv6 addresses b/c of input validation */
    if (strchr(ip, '[') != nullptr) {
        return true;
    }
    return false;
}

bool Endpoint::ipv6_is_linklocal(const char *ip)
{
    /* link-local addresses start with fe80, ULA addresses are in fc::/7 range */
    if (strncmp(ip, "fe80", 4) == 0 || strncmp(ip, "fc", 2) == 0 || strncmp(ip, "fd", 2) == 0) {
        return true;
    }
    return false;
}

bool Endpoint::ipv6_is_multicast(const char *ip)
{
    /* multicast addresses start with ff0x */
    if (strncmp(ip, "ff0", 3) == 0) {
        return true;
    }
    return false;
}

unsigned int Endpoint::ipv6_get_scope_id(const char *ip)
{
    struct ifaddrs *addrs;
    char ipAddress[NI_MAXHOST];
    unsigned scope=0;
    getifaddrs(&addrs);

    /* search for our address in all interface addresses */
    for(ifaddrs *addr = addrs; addr; addr = addr->ifa_next){
        if (addr->ifa_addr && addr->ifa_addr->sa_family == AF_INET6){
            getnameinfo(addr->ifa_addr, sizeof(struct sockaddr_in6), ipAddress, sizeof(ipAddress), NULL, 0, NI_NUMERICHOST);

            /* cut the interface name from the end of a link-local address */
            auto search = strrchr(ipAddress, '%');
            if (search != nullptr) {
                *search = '\0';
            }

            /* convert to a scope ID, if it's our interface */
            if(strcmp(ipAddress, ip) == 0){
                scope = if_nametoindex(addr->ifa_name);
                break;
            }
        }
    }

    freeifaddrs(addrs);
    return scope;
}
#endif

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
        log_error("Could not get termios2 (%m)");
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

    log_info("UART [%d] speed = %u", fd, baudrate);

    if (ioctl(fd, TCFLSH, TCIOFLUSH) == -1) {
        log_error("Could not flush terminal (%m)");
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
        log_error("Could not get termios2 (%m)");
        return -1;
    }

    if (enabled)
        tc.c_cflag |= CRTSCTS;
    else
        tc.c_cflag &= ~CRTSCTS;

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        log_error("Could not set terminal attributes (%m)");
        return -1;
    }

    log_info("UART [%d] flowcontrol = %s", fd, enabled ? "enabled" : "disabled");

    return 0;
}

int UartEndpoint::open(const char *path)
{
    struct termios2 tc;

    fd = ::open(path, O_RDWR|O_NONBLOCK|O_CLOEXEC|O_NOCTTY);
    if (fd < 0) {
        log_error("Could not open %s (%m)", path);
        return -1;
    }

    if (reset_uart(fd) < 0) {
        log_error("Could not reset uart");
        goto fail;
    }

    bzero(&tc, sizeof(tc));

    if (ioctl(fd, TCGETS2, &tc) == -1) {
        log_error("Could not get termios2 (%m)");
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
        log_error("Could not set terminal attributes (%m)");
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
            log_warning("Error while trying to read serial port configuration: %s", strerror(result));
            goto set_latency_failed;
        }

        serial_ctl.flags |= ASYNC_LOW_LATENCY;

        result =  ioctl(fd, TIOCSSERIAL, &serial_ctl);
        if (result < 0) {
            log_warning("Error while trying to write serial port latency: %s", strerror(result));
        }
    }

set_latency_failed:
    if (ioctl(fd, TCFLSH, TCIOFLUSH) == -1) {
        log_error("Could not flush terminal (%m)");
        goto fail;
    }

    log_info("Open UART [%d] %s *", fd, path);

    return fd;

fail:
    ::close(fd);
    fd = -1;
    return -1;
}

bool UartEndpoint::_change_baud_cb(void *data)
{
    _current_baud_idx = (_current_baud_idx + 1) % _baudrates.size();

    log_info("Retrying UART [%d] on new baudrate: %lu", fd, _baudrates[_current_baud_idx]);

    set_speed(_baudrates[_current_baud_idx]);

    return true;
}

int UartEndpoint::read_msg(struct buffer *pbuf, int *target_sysid, int *target_compid,
                           uint8_t *src_sysid, uint8_t *src_compid, uint32_t *msg_id)
{
    int ret = Endpoint::read_msg(pbuf, target_sysid, target_compid, src_sysid, src_compid, msg_id);

    if (_change_baud_timeout != nullptr && ret == ReadOk) {
        log_info("Baudrate %lu responded, keeping it", _baudrates[_current_baud_idx]);
        Mainloop::get_instance().del_timeout(_change_baud_timeout);
        _change_baud_timeout = nullptr;
    }

    return ret;
}

ssize_t UartEndpoint::_read_msg(uint8_t *buf, size_t len)
{
    ssize_t r = ::read(fd, buf, len);
    if ((r == -1 && errno == EAGAIN) || r == 0)
        return 0;
    if (r == -1)
        return -errno;

    return r;
}

int UartEndpoint::write_msg(const struct buffer *pbuf)
{
    if (fd < 0) {
        log_error("Trying to write invalid fd");
        return -EINVAL;
    }

    /* TODO: send any pending data */
    if (tx_buf.len > 0) {
        ;
    }

    ssize_t r = ::write(fd, pbuf->data, pbuf->len);
    if (r == -1 && errno == EAGAIN)
        return -EAGAIN;

    _stat.write.total++;
    _stat.write.bytes += pbuf->len;

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t) pbuf->len) {
        _incomplete_msgs++;
        log_debug("Discarding packet, incomplete write %zd but len=%u", r, pbuf->len);
    }

    log_debug("UART [%d] wrote %zd bytes", fd, r);

    return r;
}

int UartEndpoint::add_speeds(std::vector<unsigned long> bauds)
{
    if (!bauds.size())
        return -EINVAL;

    _baudrates = bauds;

    set_speed(_baudrates[0]);

    _change_baud_timeout = Mainloop::get_instance().add_timeout(
        MSEC_PER_SEC * UART_BAUD_RETRY_SEC,
        std::bind(&UartEndpoint::_change_baud_cb, this, std::placeholders::_1), this);

    return 0;
}

UdpEndpoint::UdpEndpoint()
    : Endpoint{"UDP"}
{
    bzero(&sockaddr, sizeof(sockaddr));
    bzero(&sockaddr_out, sizeof(sockaddr_out));
#ifdef ENABLE_IPV6
    bzero(&sockaddr6, sizeof(sockaddr6));
    bzero(&sockaddr6_out, sizeof(sockaddr6_out));
#endif
}

int UdpEndpoint::open(const char *ip, unsigned long port, UdpEndpoint::UdpMode mode, const char *target_ip)
{
    this->_mode = mode;
    const int broadcast_val = 1;
    char *target_ip_str = nullptr;

#ifdef ENABLE_IPV6
    this->is_ipv6 = Endpoint::is_ipv6(ip);
    if (this->is_ipv6) {
        fd = socket(AF_INET6, SOCK_DGRAM, 0);
    } else {
#endif
    fd = socket(AF_INET, SOCK_DGRAM, 0);

#ifdef ENABLE_IPV6
    }
#endif

    if (fd == -1) {
        log_error("Could not create socket (%m)");
        return -1;
    }

#ifdef ENABLE_IPV6
    if (this->is_ipv6) {
        /* strip square brackets from ip string(s) */
        char *ip_str = strdup(&ip[1]);
        ip_str[strlen(ip_str) - 1] = '\0';

        /* remove omittable zeros from IPv6 address */
        sockaddr_in6 ip_addr;
        inet_pton(AF_INET6, ip_str, &ip_addr.sin6_addr);
        inet_ntop(AF_INET6, &(ip_addr.sin6_addr), ip_str, strlen(ip));

        /* do some more input validation for multicast mode */
        if (_mode == UdpEndpoint::UdpMode::Multicast) {
            if (nullptr == target_ip) {
                log_error("Multicast mode needs a target IP!");
                goto fail;
            }

            target_ip_str = strdup(&target_ip[1]);
            target_ip_str[strlen(target_ip_str) - 1] = '\0';

            if ((Endpoint::ipv6_is_multicast(target_ip_str) || Endpoint::ipv6_is_linklocal(target_ip_str))
                && !Endpoint::ipv6_is_linklocal(ip_str)) {
                log_error("Address must be link local, if targetAddress is multicast or link local!");
                goto fail;
            }
        }

        sockaddr6.sin6_family = AF_INET6;
        sockaddr6.sin6_port = htons(port);

        /* multicast address needs to listen to all, but "filter" incoming packets */
        if ((_mode == UdpEndpoint::UdpMode::Eavesdropping && Endpoint::ipv6_is_multicast(ip_str)) ||
            (_mode == UdpEndpoint::UdpMode::Multicast && Endpoint::ipv6_is_multicast(target_ip_str))) {
            sockaddr6.sin6_addr = in6addr_any;

            struct ipv6_mreq group{};
            if (_mode == UdpEndpoint::UdpMode::Multicast) {
                inet_pton(AF_INET6, target_ip_str, &group.ipv6mr_multiaddr);
            } else {
                inet_pton(AF_INET6, ip_str, &group.ipv6mr_multiaddr);
            }
            if (setsockopt(fd, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, &group, sizeof(group)) < 0){
                log_error("Error setting IPv6 multicast socket options (%m)");
                goto fail;
            }

            /* disable multicast loopback */
            bool loopch = false;
            if (setsockopt(fd, IPPROTO_IP, IPV6_MULTICAST_LOOP, &loopch, sizeof(loopch)) < 0){
                log_error("Error disabling IPv6 multicast loop (%m)");
                goto fail;
            }
        } else {
            inet_pton(AF_INET6, ip_str, &sockaddr6.sin6_addr);
        }

        /* link-local address needs a scope ID */
        if (Endpoint::ipv6_is_linklocal(ip_str)) {
            sockaddr6.sin6_scope_id = ipv6_get_scope_id(ip_str);
        }

        /* for multicast mode: set fixed outgoing address */
        if (_mode == UdpEndpoint::UdpMode::Multicast) {
            memcpy(&sockaddr6_out, &sockaddr6, sizeof(sockaddr6_out));
        }

        free(ip_str);
    } else {
#endif
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(port);

    if ((_mode == UdpEndpoint::UdpMode::Eavesdropping && Endpoint::ipv4_is_multicast(ip)) ||
        (_mode == UdpEndpoint::UdpMode::Multicast && Endpoint::ipv4_is_multicast(target_ip))) {
        sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

        struct ip_mreq group{};
        if (_mode == UdpMode::Multicast) {
            inet_pton(AF_INET, target_ip, &group.imr_multiaddr);
        } else {
            inet_pton(AF_INET, ip, &group.imr_multiaddr);
        }
        group.imr_interface.s_addr = htonl(INADDR_ANY);
        if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &group, sizeof(group)) < 0){
            log_error("Error setting IPv4 multicast socket options (%m)");
            goto fail;
        }

        /* disable multicast loopback */
        bool loopch = false;
        if (setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loopch, sizeof(loopch)) < 0){
            log_error("Error disabling IPv4 multicast loop (%m)");
            goto fail;
        }
    } else {
        sockaddr.sin_addr.s_addr = inet_addr(ip);
    }

    /* for multicast mode: set fixed outgoing address */
    if (_mode == UdpEndpoint::UdpMode::Multicast) {
        memcpy(&sockaddr_out, &sockaddr, sizeof(sockaddr_out));
    }
#ifdef ENABLE_IPV6
    }
#endif

    if (_mode == UdpEndpoint::UdpMode::Eavesdropping || _mode == UdpEndpoint::UdpMode::Multicast) {
#ifdef ENABLE_IPV6
        if (this->is_ipv6) {
            if (bind(fd, (struct sockaddr *)&sockaddr6, sizeof(sockaddr6)) < 0) {
                log_error("Error binding IPv6 socket (%m)");
                goto fail;
            }
        } else {
#endif
        if (bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
            log_error("Error binding socket (%m)");
            goto fail;
        }
#ifdef ENABLE_IPV6
        }
#endif
    } else {
        if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast_val, sizeof(broadcast_val))) {
            log_error("Error enabling broadcast in socket (%m)");
            goto fail;
        }
    }

    if (fcntl(fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        log_error("Error setting socket fd as non-blocking (%m)");
        goto fail;
    }

    if (_mode == UdpEndpoint::UdpMode::Eavesdropping) {
#ifdef ENABLE_IPV6
        if (this->is_ipv6) {
            sockaddr6.sin6_port = 0;
        } else {
#endif
        sockaddr.sin_port = 0;
#ifdef ENABLE_IPV6
        }
#endif
    } else {
        if (_mode == UdpEndpoint::UdpMode::Multicast) {
#ifdef ENABLE_IPV6
            if (this->is_ipv6) {
                inet_pton(AF_INET6, target_ip_str, &sockaddr6_out.sin6_addr);
            } else {
#endif
            inet_pton(AF_INET, target_ip, &sockaddr_out.sin_addr);
#ifdef ENABLE_IPV6
            }
#endif
        }
    }

    if (_mode == UdpEndpoint::UdpMode::Multicast) {
        log_info("Open UDP [%d] %s:%lu target %s", fd, ip, port, target_ip);
    } else {
        log_info("Open UDP [%d] %s:%lu %c", fd, ip, port, _mode != UdpEndpoint::UdpMode::Normal ? '*' : ' ');
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
    socklen_t addrlen = sizeof(sockaddr);
    ssize_t r = 0;
#ifdef ENABLE_IPV6
    if (this->is_ipv6) {
        addrlen = sizeof(sockaddr6);
        r = ::recvfrom(fd, buf, len, 0, (struct sockaddr *)&sockaddr6, &addrlen);
    } else {
#endif
    r = ::recvfrom(fd, buf, len, 0, (struct sockaddr *)&sockaddr, &addrlen);
#ifdef ENABLE_IPV6
    }
#endif

    if (r == -1 && errno == EAGAIN)
        return 0;
    if (r == -1)
        return -errno;

    return r;
}

int UdpEndpoint::write_msg(const struct buffer *pbuf)
{
    if (fd < 0) {
        log_error("Trying to write invalid fd");
        return -EINVAL;
    }

    /* TODO: send any pending data */
    if (tx_buf.len > 0) {
        ;
    }

#ifdef ENABLE_IPV6
    bool sock_connected = false;
    if (is_ipv6) {
        sock_connected = sockaddr6.sin6_port != 0;
    } else {
        sock_connected = sockaddr.sin_port != 0;
    }
    if (!sock_connected) {
#else
    if (!sockaddr.sin_port) {
#endif
        log_debug("No one ever connected to %d. No one to write for", fd);
        return 0;
    }

    ssize_t r = 0;
#ifdef ENABLE_IPV6
    if (this->is_ipv6) {
        if (_mode == UdpEndpoint::UdpMode::Multicast) {
            r = ::sendto(fd, pbuf->data, pbuf->len, 0, (struct sockaddr *)&sockaddr6_out, sizeof(sockaddr6_out));
        } else {
            r = ::sendto(fd, pbuf->data, pbuf->len, 0, (struct sockaddr *)&sockaddr6, sizeof(sockaddr6));
        }
    } else {
#endif
    if (_mode == UdpEndpoint::UdpMode::Multicast) {
        r = ::sendto(fd, pbuf->data, pbuf->len, 0, (struct sockaddr *)&sockaddr_out, sizeof(sockaddr_out));
    } else {
        r = ::sendto(fd, pbuf->data, pbuf->len, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    }
#ifdef ENABLE_IPV6
    }
#endif

    if (r == -1) {
        if (errno != EAGAIN && errno != ECONNREFUSED && errno != ENETUNREACH)
            log_error("Error sending udp packet (%m)");
        return -errno;
    };

    _stat.write.total++;
    _stat.write.bytes += pbuf->len;

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t) pbuf->len) {
        _incomplete_msgs++;
        log_debug("Discarding packet, incomplete write %zd but len=%u", r, pbuf->len);
    }

    log_debug("UDP [%d] wrote %zd bytes", fd, r);

    return r;
}

TcpEndpoint::TcpEndpoint()
    : Endpoint{"TCP"}
{
    bzero(&sockaddr, sizeof(sockaddr));
#ifdef ENABLE_IPV6
    bzero(&sockaddr6, sizeof(sockaddr6));
#endif
}

TcpEndpoint::~TcpEndpoint()
{
    close();
    free(_ip);
}

int TcpEndpoint::accept(int listener_fd)
{
    socklen_t addrlen = sizeof(sockaddr);
#ifdef ENABLE_IPV6
    if (this->is_ipv6) {
        addrlen = sizeof(sockaddr6);
        fd = accept4(listener_fd, (struct sockaddr *)&sockaddr6, &addrlen, SOCK_NONBLOCK);
    } else {
#endif
    fd = accept4(listener_fd, (struct sockaddr *)&sockaddr, &addrlen, SOCK_NONBLOCK);
#ifdef ENABLE_IPV6
    }
#endif

    if (fd == -1)
        return -1;

    log_info("TCP connection [%d] accepted", fd);

    return fd;
}

int TcpEndpoint::open(const char *ip, unsigned long port)
{
    if (!_ip || strcmp(ip, _ip)) {
        free(_ip);
        _ip = strdup(ip);
        _port = port;
    }

    assert_or_return(_ip, -ENOMEM);

#ifdef ENABLE_IPV6
    this->is_ipv6 = Endpoint::is_ipv6(ip);
    if (this->is_ipv6) {
        fd = socket(AF_INET6, SOCK_STREAM, 0);
    } else {
#endif
    fd = socket(AF_INET, SOCK_STREAM, 0);

#ifdef ENABLE_IPV6
    }
#endif

    if (fd == -1) {
        log_error("Could not create socket (%m)");
        return -1;
    }

#ifdef ENABLE_IPV6
    if (this->is_ipv6) {
        /* strip square brackets from ip string */
        char *ip_str = strdup(&_ip[1]);
        ip_str[strlen(ip_str) - 1] = '\0';

        /* multicast address is not allowed for TCP sockets */
        if (Endpoint::ipv6_is_multicast(ip_str)) {
            log_error("TCP socket does not support multicast address");
            goto fail;
        }

        sockaddr6.sin6_family = AF_INET6;
        sockaddr6.sin6_port = htons(port);
        inet_pton(AF_INET6, ip_str, &sockaddr6.sin6_addr);

        /* link-local address needs a scope ID */
        if (Endpoint::ipv6_is_linklocal(ip_str)) {
            sockaddr6.sin6_scope_id = ipv6_get_scope_id(ip_str);
        }

        free(ip_str);
    } else {
#endif
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(ip);
    sockaddr.sin_port = htons(port);
#ifdef ENABLE_IPV6
    }
#endif

#ifdef ENABLE_IPV6
    if (this->is_ipv6) {
        if (connect(fd, (struct sockaddr *)&sockaddr6, sizeof(sockaddr6)) < 0) {
            log_error("Error connecting to IPv6 socket (%m)");
            goto fail;
        }
    } else {
#endif
    if (connect(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        log_error("Error connecting to socket (%m)");
        goto fail;
    }
#ifdef ENABLE_IPV6
    }
#endif

    if (fcntl(fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        log_error("Error setting socket fd as non-blocking (%m)");
        goto fail;
    }

    log_info("Open TCP [%d] %s:%lu", fd, ip, port);

    _valid = true;
    return fd;

fail:
    ::close(fd);
    return -1;
}

ssize_t TcpEndpoint::_read_msg(uint8_t *buf, size_t len)
{
    socklen_t addrlen = sizeof(sockaddr);
    errno = 0;
    ssize_t r = 0;
#ifdef ENABLE_IPV6
    if (this->is_ipv6) {
        addrlen = sizeof(sockaddr6);
        r = ::recvfrom(fd, buf, len, 0, (struct sockaddr *)&sockaddr6, &addrlen);
    } else {
#endif
    r = ::recvfrom(fd, buf, len, 0, (struct sockaddr *)&sockaddr, &addrlen);
#ifdef ENABLE_IPV6
    }
#endif

    if (r == -1 && errno == EAGAIN)
        return 0;
    if (r == -1)
        return -errno;

    // a read of zero on a stream socket means that other side shut down
    if (r == 0 && len != 0) {
        _valid = false;
        return EOF; // TODO is EOF always negative?
    }

    return r;
}

int TcpEndpoint::write_msg(const struct buffer *pbuf)
{
    if (fd < 0) {
        log_error("Trying to write invalid fd");
        return -EINVAL;
    }

    /* TODO: send any pending data */
    if (tx_buf.len > 0) {
        ;
    }

    ssize_t r = 0;
#ifdef ENABLE_IPV6
    if (this->is_ipv6) {
        r = ::sendto(fd, pbuf->data, pbuf->len, 0, (struct sockaddr *)&sockaddr6, sizeof(sockaddr6));
    } else {
#endif
    r = ::sendto(fd, pbuf->data, pbuf->len, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
#ifdef ENABLE_IPV6
    }
#endif

    if (r == -1) {
        if (errno != EAGAIN && errno != ECONNREFUSED)
            log_error("Error sending tcp packet (%m)");
        if (errno == EPIPE)
            _valid = false;
        return -errno;
    };

    _stat.write.total++;
    _stat.write.bytes += pbuf->len;

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t) pbuf->len) {
        _incomplete_msgs++;
        log_debug("Discarding packet, incomplete write %zd but len=%u", r, pbuf->len);
    }

    log_debug("TCP [%d] wrote %zd bytes", fd, r);

    return r;
}

void TcpEndpoint::close()
{
    if (fd > -1) {
        ::close(fd);

        log_info("TCP Connection [%d] closed", fd);
    }

    fd = -1;
}
