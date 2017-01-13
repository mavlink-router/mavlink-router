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
#include "comm.h"

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <mavlink.h>

#include "log.h"
#include "util.h"

#define RX_BUF_MAX_SIZE (MAVLINK_MAX_PACKET_LEN * 4)
#define TX_BUF_MAX_SIZE (8U * 1024U)

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

Endpoint::Endpoint(const char *name, bool check_crc)
    : _name{name},
      _check_crc{check_crc}
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
    if (fd >= 0) {
        ::close(fd);
    }
}

int Endpoint::read_msg(struct buffer *pbuf)
{
    bool should_read_more = true;

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

        log_debug("%s: Got %zd bytes", _name, r);
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
    _read_total++;

    if (_check_crc && !check_crc())
        return 0;

    pbuf->data = rx_buf.data;
    pbuf->len = expected_size;

    return 1;
}

bool Endpoint::check_crc()
{
    const bool mavlink2 = rx_buf.data[0] == MAVLINK_STX;
    uint32_t msg_id;
    uint16_t crc_msg, crc_calc;
    uint8_t payload_len, header_len, *payload;
    const mavlink_msg_entry_t *msg_entry;

    if (mavlink2) {
        struct mavlink_router_mavlink2_header *hdr =
                    (struct mavlink_router_mavlink2_header *)rx_buf.data;
        payload = rx_buf.data + sizeof(*hdr);
        msg_id = hdr->msgid;
        header_len = sizeof(*hdr);
        payload_len = hdr->payload_len;
    } else {
        struct mavlink_router_mavlink1_header *hdr =
                    (struct mavlink_router_mavlink1_header *)rx_buf.data;
        payload = rx_buf.data + sizeof(*hdr);
        msg_id = hdr->msgid;
        header_len = sizeof(*hdr);
        payload_len = hdr->payload_len;
    }

    msg_entry = mavlink_get_msg_entry(msg_id);
    if (!msg_entry) {
        /*
         * It is accepting and forwarding unknown messages ids because
         * it can be a new MAVLink message implemented only in
         * Ground Station and Flight Stack. Although it can also be a
         * corrupted message is better forward than silent drop it.
         */
        return true;
    }

    crc_msg = payload[payload_len] | (payload[payload_len + 1] << 8);
    crc_calc = crc_calculate(&rx_buf.data[1], header_len + payload_len - 1);
    crc_accumulate(msg_entry->crc_extra, &crc_calc);
    if (crc_calc != crc_msg) {
        _read_crc_errors++;
        return false;
    }

    return true;
}

void Endpoint::print_statistics()
{
    printf("Endpoint {"
           "\n\tname: %s" \
           "\n\tmessages read: %u" \
           "\n\tmessages read with CRC error: %u %f%%" \
           "\n\tmessages written: %u" \
           "\n}" \
           "\n",
           _name, _read_total, _read_crc_errors,
           (_read_crc_errors * 100.0f) / (_read_total == 0 ? 1 : _read_total),
           _write_total);
}

int UartEndpoint::open(const char *path, speed_t baudrate)
{
    struct termios2 tc;

    fd = ::open(path, O_RDWR|O_NONBLOCK|O_CLOEXEC|O_NOCTTY);
    if (fd < 0) {
        log_error_errno(errno, "Could not open %s (%m)", path);
        return -1;
    }

    bzero(&tc, sizeof(tc));

    if (ioctl(fd, TCGETS2, &tc) == -1) {
        log_error_errno(errno, "Could not get termios2 (%m)");
        goto fail;
    }

    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    tc.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
    tc.c_cflag &= ~(CSIZE | PARENB | CBAUD | CRTSCTS);
    tc.c_cflag |= CS8 | BOTHER;

    tc.c_cc[VMIN] = 0;
    tc.c_cc[VTIME] = 0;
    tc.c_ispeed = baudrate;
    tc.c_ospeed = baudrate;

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        log_error_errno(errno, "Could not set terminal attributes (%m)");
        goto fail;
    }

    if (ioctl(fd, TCFLSH, TCIOFLUSH) == -1) {
        log_error_errno(errno, "Could not flush terminal (%m)");
        goto fail;
    }

    return fd;

fail:
    ::close(fd);
    fd = -1;
    return -1;
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

    _write_total++;

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t) pbuf->len) {
        log_warning("Discarding packet, incomplete write %zd but len=%u",
                    r, pbuf->len);
    }

    log_debug("UART: wrote %zd bytes", r);

    return r;
}

UdpEndpoint::UdpEndpoint()
    : Endpoint{"UDP", false}
{
    bzero(&sockaddr, sizeof(sockaddr));
}

int UdpEndpoint::open(const char *ip, unsigned long port)
{
    const int broadcast_val = 1;
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == -1) {
        log_error_errno(errno, "Could not create socket (%m)");
        return -1;
    }

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(ip);
    sockaddr.sin_port = htons(port);

    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast_val, sizeof(broadcast_val))) {
        log_error_errno(errno, "Error enabling broadcast in socket (%m)");
        goto fail;
    }

    if (fcntl(fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        log_error_errno(errno, "Error setting socket fd as non-blocking (%m)");
        goto fail;
    }

    log_info("Open %s:%lu", ip, port);

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
    ssize_t r = ::recvfrom(fd, buf, len, 0,
                           (struct sockaddr *)&sockaddr, &addrlen);
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

    ssize_t r = ::sendto(fd, pbuf->data, pbuf->len, 0,
                         (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (r == -1) {
        if (errno != EAGAIN && errno != ECONNREFUSED)
            log_error_errno(errno, "Error sending udp packet (%m)");
        return -errno;
    };

    _write_total++;

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t) pbuf->len) {
        log_warning("Discarding packet, incomplete write %zd but len=%u",
                    r, pbuf->len);
    }

    log_debug("UDP: wrote %zd bytes", r);

    return r;
}
