// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "RtpsTopics.h"
#include "UART_node.h"

#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>
#include <fastrtps/Domain.h>
#include <fastrtps/utils/eClock.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "log.h"
#include "util.h"

#define BUFFER_SIZE 1024
#define USLEEP_TIME 500

using namespace eprosima;
using namespace eprosima::fastrtps;

struct options {
    const char *device;
    unsigned long baudrate;
    unsigned long port;
    bool report_stats;
};

static struct options _options
    = {.device = "/dev/ttyS1", .baudrate = 460800, .port = 13800, .report_stats = false};

static struct sockaddr_in _sockaddr;

static void help(FILE *fp)
{
    fprintf(fp, "%s [OPTIONS]\n\n"
                "  -d <device>              UART device to get RTPS (and MAVLink) messages.\n"
                "                           Default '/dev/ttyS1'\n"
                "  -b <baudrate>            Baudrate of UART device.\n"
                "                           Default '460800'\n"
                "  -p <port>                UDP port where to send (or receive) MAVLink packets.\n"
                "                           Default '13800'\n"
                "  -r                       Report stats each 10 seconds\n",
            program_invocation_short_name);
}

static int parse_options(int argc, char *argv[])
{
    int c;

    while ((c = getopt(argc, argv, "d:b:p:r")) >= 0) {
        switch (c) {
        case 'd':
            _options.device = optarg;
            break;
        case 'b':
            if (safe_atoul(optarg, &_options.baudrate)) {
                log_error("Invalid argument for baudrate = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            break;
        case 'p':
            if (safe_atoul(optarg, &_options.port)) {
                log_error("Invalid argument for port = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            break;
        case 'r':
            _options.report_stats = true;
            break;
        case '?':
        default:
            help(stderr);
            return -EINVAL;
        }
    }

    return 0;
}

static int setup_udp(unsigned port)
{
    int fd;
    const int broadcast_val = 1;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == -1) {
        log_error("Could not create socket %d", errno);
        return -1;
    }

    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    _sockaddr.sin_port = htons(port);

    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast_val, sizeof(broadcast_val))) {
        log_error("Error enabling broadcast %d", errno);
        return -1;
    }

    if (fcntl(fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        log_error("Error setting socket fd as non-blocking %d", errno);
        return -1;
    }

    return fd;
}

static void loop(int epollfd, int uart_fd, int udp_fd, UART_node uart_node, RtpsTopics &topics)
{
    unsigned bytes_read = 0, bytes_sent = 0;
    char topic_ID = 255;

    struct timespec begin;
    bool measuring = false;
    uint8_t seq;

    struct epoll_event events[8];
    char data_buffer[BUFFER_SIZE] = {};

    // TODO use mavlink-router mainloop (which need to be extracted from it)
    do {
        if (_options.report_stats && !measuring) {
            clock_gettime(0, &begin);
            measuring = true;
        }

        // TODO check if it's possible to add RTPS subscribers on epoll
        // or use a thread for them
        int r = epoll_wait(epollfd, events, 8, USLEEP_TIME);
        if (r < 0 && errno == EINTR)
            continue;

        bool udp = false;
        bool uart = false;
        for (int i = 0; i < r; i++) {
            if (events[i].data.fd == udp_fd)
                udp = true;
            if (events[i].data.fd == uart_fd)
                uart = true;
        }

        if (uart) {
            // Publish messages received from UART
            r = uart_node.read();
            if (r > 0)
                bytes_read += r;
            while (0 < uart_node.parseRTPSfromUART(&topic_ID, &seq, data_buffer, BUFFER_SIZE)) {
                topics.publish(topic_ID, data_buffer, sizeof(data_buffer));
            }

            int len;
            while ((len = uart_node.parseMavlinkFromUART(data_buffer, sizeof(data_buffer))) > 0) {
                // Just send it, as is, to mavlink-router
                r = sendto(udp_fd, data_buffer, len, 0, (struct sockaddr *)&_sockaddr,
                           sizeof(_sockaddr));
                if (r < 0) {
                    log_error("Failed to write mavlink to UDP %d", r);
                }
            }
        }

        // Send mavlink over UART
        if (udp) {
            // Assumption: we are running side by side with mavlink-router
            // So, UDP won't fail.
            // In the future, mavlink-router and rtps-router should become one
            // (or at least, share memory for this)
            socklen_t addrlen = sizeof(sockaddr);
            ssize_t len = recvfrom(udp_fd, data_buffer, sizeof(data_buffer), 0,
                                   (struct sockaddr *)&_sockaddr, &addrlen);

            if (r > 0) {
                r = uart_node.writeMavlinkToUART(data_buffer, len);

                if (r < 0) {
                    log_error("Failed to write mavlink to UART %d", r);
                } else {
                    bytes_sent += r;
                }
            }
        }

        // Send subscribed topics over UART
        eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, sizeof(data_buffer));
        eprosima::fastcdr::Cdr scdr(cdrbuffer);
        while (topics.nextMsg(&topic_ID, scdr)) {
            size_t len = scdr.getSerializedDataLength();
            r = uart_node.writeRTPStoUART(topic_ID, scdr.getBufferPointer(), len);
            if (r > 0)
                bytes_sent += r;
        }

        if (measuring) {
            struct timespec end;
            clock_gettime(0, &end);
            double elapsed_secs = double(end.tv_sec - begin.tv_sec)
                + double(end.tv_nsec - begin.tv_nsec) / double(1000000000);
            if (elapsed_secs > 10) {
                log_info("\nIn the last %.02f seconds on UART: %u bytes read. %u bytes sent\n"
                         "Write speed: %.02fkB/s Read speed: %.02fkB/s",
                         elapsed_secs, bytes_read, bytes_sent,
                         (double)bytes_sent / (1024 * elapsed_secs),
                         (double)bytes_read / (1024 * elapsed_secs));
                bytes_read = bytes_sent = 0;
                measuring = false;
            }
        }
    } while (true);
}

int main(int argc, char **argv)
{
    int udp_fd, uart_fd, epollfd;
    UART_node uart_node;
    RtpsTopics topics;
    struct epoll_event epev = {};

    Log::open();

    // TODO use conf_file as well?
    if (parse_options(argc, argv) < 0)
        goto err;

    epollfd = epoll_create1(EPOLL_CLOEXEC);
    if (epollfd == -1) {
        log_error("Epoll create error: %d", errno);
        goto err;
    }

    uart_fd = uart_node.init_uart(_options.device, _options.baudrate);
    if (uart_fd < 0) {
        goto err;
    }

    if ((udp_fd = setup_udp(_options.port)) < 0)
        goto err;

    topics.init();

    // Add fds on epoll
    epev.events = EPOLLIN;
    epev.data.fd = uart_fd;

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, uart_fd, &epev) < 0) {
        log_error("Epoll add UART fd error %d", errno);
        goto err;
    }

    epev.data.fd = udp_fd;

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, udp_fd, &epev) < 0) {
        log_error("Epoll add UDP fd error %d", errno);
        goto err;
    }

    loop(epollfd, uart_fd, udp_fd, uart_node, topics);

    Log::close();

    return 0;

err:
    Log::close();

    return EXIT_FAILURE;
}
