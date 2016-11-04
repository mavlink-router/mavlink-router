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

#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "comm.h"
#include "log.h"
#include "util.h"

#define MAX_PORTS 10


class Mainloop {
public:
    int open();
    int add_fd(int fd, void *data, int events);
    int mod_fd(int fd, void *data, int events);
    void loop();
    void handle_read(Endpoint *e);
    void handle_canwrite(Endpoint *e);
    void write_msg(Endpoint *e, const struct buffer *buf);

    int epollfd = -1;
};

static struct opt {
    long unsigned baudrate;
} opt = {
    .baudrate = 115200U,
};

static Endpoint *g_master;
static Endpoint *g_endpoints[MAX_PORTS];
static bool g_should_exit;

static void help(FILE *fp) {
    fprintf(fp,
            "%s [OPTIONS...] <uart> <ip:port>\n\n"
            "  -h --help                    Print this message\n"
            "  -b --baudrate                Use baudrate for UART\n"
            , program_invocation_short_name);
}

static int parse_argv(int argc, char *argv[], const char **uart, const char **addr)
{
    static const struct option options[] = {
        { "baudrate",   required_argument,  NULL,   'b' },
        { }
    };
    int c;

    assert(argc >= 0);
    assert(argv);
    assert(uart);
    assert(addr);

    while ((c = getopt_long(argc, argv, "hb:", options, NULL)) >= 0) {
        switch (c) {
        case 'h':
            help(stdout);
            return 0;
        case 'b':
            if (safe_atoul(optarg, &opt.baudrate) < 0) {
                log_error("Invalid argument for baudrate = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            break;
        case '?':
        default:
            help(stderr);
            return -EINVAL;
        }
    }

    /* positional arguments */
    if (optind + 2 != argc) {
        log_error("Error parsing required argument %d %d", optind, argc);
        help(stderr);
        return -EINVAL;
    }

    *uart = argv[optind++];
    *addr = argv[optind];

    return 2;
}

int Mainloop::open()
{
    if (epollfd != -1)
        return -EBUSY;

    epollfd = epoll_create1(EPOLL_CLOEXEC);

    if (epollfd == -1) {
        log_error_errno(errno, "%m");
        return -1;
    }

    return 0;
}

int Mainloop::mod_fd(int fd, void *data, int events)
{
    struct epoll_event epev = { };

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_MOD, fd, &epev) < 0) {
        log_error_errno(errno, "Could not mod fd (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::add_fd(int fd, void *data, int events)
{
    struct epoll_event epev = { };

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &epev) < 0) {
        log_error_errno(errno, "Could not add fd to epoll (%m)");
        return -1;
    }

    return 0;
}

void Mainloop::write_msg(Endpoint *e, const struct buffer *buf)
{
    int r = e->write_msg(buf);

    /*
     * If endpoint would block, add EPOLLOUT event to get notified when it's
     * possible to write again
     */
    if (r == -EAGAIN)
        mod_fd(e->fd, e, EPOLLIN | EPOLLOUT);
}

void Mainloop::handle_read(Endpoint *endpoint)
{
    assert(endpoint);

    struct buffer buf{};

    /*
     * We read from this endpoint and forward to the other endpoints.
     * Currently this makes the flight stack endpoint (master) as a special
     * one: packets from master goes to the other connected endpoints and
     * packets from endpoints go to master.
     *
     * This logic should be replaced with a routing logic so each endpoint
     * can talk to each one without involving the flight stack.
     */
    while (endpoint->read_msg(&buf) > 0) {
        if (endpoint == g_master) {
            for (Endpoint **e = g_endpoints; *e != nullptr; e++) {
                write_msg(*e, &buf);
            }
        } else {
            write_msg(g_master, &buf);
        }
    }
}

void Mainloop::handle_canwrite(Endpoint *e)
{
    int r = e->flush_pending_msgs();

    /*
     * If we could flush everything without triggering another block write,
     * remove EPOLLOUT from flags so we don't get called again
     */
    if (r != -EAGAIN)
        mod_fd(e->fd, e, EPOLLIN);
}

void Mainloop::loop()
{
    const int max_events = 8;
    struct epoll_event events[max_events];
    int r;

    if (epollfd < 0)
        return;

    while (!g_should_exit) {
        int i;

        r = epoll_wait(epollfd, events, max_events, -1);
        if (r < 0 && errno == EINTR)
            continue;

        for (i = 0; i < r; i++) {
            Endpoint *e = static_cast<Endpoint*>(events[i].data.ptr);

            if (events[i].events & EPOLLIN)
                handle_read(e);

            if (events[i].events & EPOLLOUT)
                handle_canwrite(e);
        }
    }
}

static void exit_signal_handler(int signum)
{
    g_should_exit = true;
}

static void setup_signal_handlers()
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
}

int main(int argc, char *argv[])
{
    const char *uartstr = NULL, *addrstr = NULL;
    UdpEndpoint udp;
    UartEndpoint uart{};
    Mainloop mainloop{};

    setup_signal_handlers();

    log_open();

    if (parse_argv(argc, argv, &uartstr, &addrstr) != 2)
        goto close_log;

    if (mainloop.open() < 0)
        goto close_log;

    if (uart.open(uartstr, opt.baudrate) < 0)
        goto close_log;

    if (udp.open(addrstr) < 0)
        goto close_log;

    g_master = &uart;
    mainloop.add_fd(uart.fd, &uart, EPOLLIN);

    g_endpoints[0] = &udp;
    mainloop.add_fd(udp.fd, &udp, EPOLLIN);

    mainloop.loop();

    log_close();

    return 0;

close_log:
    log_close();
    return EXIT_FAILURE;
}
