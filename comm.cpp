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

#include "log.h"
#include "util.h"

#define RX_BUF_MAX_SIZE (8U * 1024U)
#define TX_BUF_MAX_SIZE (8U * 1024U)

Endpoint::Endpoint()
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

    /* disable LF -> CR/LF */
    tc.c_iflag &= ~(BRKINT | IGNBRK | ICRNL | IXON | IXOFF);
    tc.c_oflag &= ~(OPOST | ONLCR);
    tc.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | TOSTOP);
    tc.c_cflag &= ~(CRTSCTS);

    tc.c_cc[VMIN] = 0;
    tc.c_cc[VTIME] = 0;
    tc.c_ispeed = baudrate;
    tc.c_ospeed = baudrate;

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        log_error_errno(errno, "Could not set terminal attributes (%m)");
        goto fail;
    }

    return fd;

fail:
    ::close(fd);
    fd = -1;
    return -1;
}

int UartEndpoint::read_msg(struct buffer *pbuf)
{
    if (fd < 0) {
        log_error("Trying to read invalid fd");
        return -EINVAL;
    }

    ssize_t r = ::read(fd, rx_buf.data, RX_BUF_MAX_SIZE);
    if ((r == -1 && errno == EAGAIN) || r == 0)
        return 0;
    if (r == -1)
        return -errno;

    log_debug("UART: Got %zd bytes", r > 0 ? r : 0);

    pbuf->data = rx_buf.data;
    pbuf->len = r;

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

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t) pbuf->len) {
        log_warning("Discarding packet, incomplete write %zd but len=%u",
                    r, pbuf->len);
    }

    log_debug("UART: wrote %zd bytes", r);

    return r;
}

int UdpEndpoint::open(const char *addr)
{
    struct sockaddr_in sockaddr;
    unsigned long port = 14550U;
    char *addrbuf = nullptr;
    char *ip, *portstr;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == -1) {
        log_error_errno(errno, "Could not create socket (%m)");
        return -1;
    }

    addrbuf = strdup(addr);
    if (!addrbuf)
        goto fail;

    ip = addrbuf;
    portstr = strchrnul(addrbuf, ':');
    if (portstr && *portstr && safe_atoul(portstr + 1, &port) < 0) {
        log_error("Invalid port in argument: %s", addr);
        goto fail;
    }

    bzero(&sockaddr, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(ip);
    sockaddr.sin_port = htons(port);

    log_info("Connecting to %.*s port %lu", (int)(portstr - ip), ip, port);

    if (connect(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        log_error_errno(errno, "Error connecting socket (%m)");
        goto fail;
    }

    if (fcntl(fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        log_error_errno(errno, "Error setting socket fd as non-blocking (%m)");
        goto fail;
    }

    free(addrbuf);

    return fd;

fail:
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
    free(addrbuf);
    return -1;
}

int UdpEndpoint::read_msg(struct buffer *pbuf)
{
    if (fd < 0) {
        log_error("Trying to read invalid fd");
        return -EINVAL;
    }

    ssize_t r = ::recv(fd, rx_buf.data, RX_BUF_MAX_SIZE, 0);
    if (r == -1 && errno == EAGAIN)
        return 0;
    if (r == -1)
        return -errno;

    log_debug("UDP: Got %zd bytes", r > 0 ? r : 0);

    pbuf->data = rx_buf.data;
    pbuf->len = r;

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

    ssize_t r = ::send(fd, pbuf->data, pbuf->len, 0);
    if (r == -1) {
        if (errno != EAGAIN && errno != ECONNREFUSED)
            log_error_errno(errno, "Error sending udp packet (%m)");
        return -errno;
    };

    /* Incomplete packet, we warn and discard the rest */
    if (r != (ssize_t) pbuf->len) {
        log_warning("Discarding packet, incomplete write %zd but len=%u",
                    r, pbuf->len);
    }

    log_debug("UDP: wrote %zd bytes", r);

    return r;
}
