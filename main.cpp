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

struct endpoint_address {
    struct endpoint_address *next;
    const char *ip;
    unsigned long port;
};

static struct opt {
    long unsigned baudrate;
    struct endpoint_address *ep_addrs;
} opt = {
    .baudrate = 115200U,
    .ep_addrs = nullptr,
};

static Endpoint *g_master;
static Endpoint **g_endpoints;
static bool g_should_exit;

static void help(FILE *fp) {
    fprintf(fp,
            "%s [OPTIONS...] <uart>\n\n"
            "  -h --help                    Print this message\n"
            "  -b --baudrate                Use baudrate for UART\n"
            "  -e --endpoint <ip[:port]>    Add UDP endpoint to communicate port is optional\n"
            "                               and in case it's not given it starts in 14550 and\n"
            "                               continues increasing not to collide with previous\n"
            "                               ports\n"
            , program_invocation_short_name);
}

static unsigned long find_next_endpoint_port(const char *ip)
{
    unsigned long port = 14550U;

    while (true) {
        endpoint_address *e;

        for (e = opt.ep_addrs; e; e = e->next) {
            if (streq(e->ip, ip) && e->port == port) {
                port++;
                break;
            }
        }
        if (!e)
            break;
    }

    return port;
}

static int parse_argv(int argc, char *argv[], const char **uart)
{
    static const struct option options[] = {
        { "baudrate",   required_argument,  NULL,   'b' },
        { "endpoints",  required_argument,  NULL,   'e' },
        { }
    };
    int c;

    assert(argc >= 0);
    assert(argv);
    assert(uart);

    while ((c = getopt_long(argc, argv, "hb:e:", options, NULL)) >= 0) {
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
        case 'e': {
            char *ip = strdup(optarg);

            char *portstr = strchrnul(ip, ':');
            unsigned long port;
            if (*portstr == '\0') {
                port = find_next_endpoint_port(ip);
            } else {
                *portstr = '\0';
                if (safe_atoul(portstr + 1, &port) < 0) {
                    log_error("Invalid port in argument: %s", optarg);
                    free(ip);
                    help(stderr);
                    return -EINVAL;
                }
            }

            struct endpoint_address *e = (struct endpoint_address*) malloc(sizeof(*e));
            e->next = opt.ep_addrs;
            e->ip = ip;
            e->port = port;
            opt.ep_addrs = e;
            break;
        }
        case '?':
        default:
            help(stderr);
            return -EINVAL;
        }
    }

    /* positional arguments */
    if (optind + 1 != argc) {
        log_error("Error parsing required argument %d %d", optind, argc);
        help(stderr);
        return -EINVAL;
    }

    *uart = argv[optind++];

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

static void free_endpoints()
{
    for (Endpoint **e = g_endpoints; *e; e++) {
        delete *e;
    }

    for (auto e = opt.ep_addrs; e;) {
        auto next = e->next;
        free(e);
        e = next;
    }
}

static bool add_endpoints(Mainloop &mainloop)
{
    unsigned n_endpoints = 0, i = 0;
    struct endpoint_address *e;

    for (e = opt.ep_addrs; e; e = e->next)
        n_endpoints++;

    g_endpoints = (Endpoint**) calloc(n_endpoints + 1, sizeof(Endpoint*));

    for (e = opt.ep_addrs; e; e = e->next) {
        UdpEndpoint *udp = new UdpEndpoint{};
        if (udp->open(e->ip, e->port) < 0) {
            log_error("Could not open %s:%ld", e->ip, e->port);
            return false;
        }

        g_endpoints[i++] = udp;
        mainloop.add_fd(udp->fd, udp, EPOLLIN);
    }

    return true;
}

int main(int argc, char *argv[])
{
    const char *uartstr = NULL;
    UartEndpoint uart{};
    Mainloop mainloop{};

    setup_signal_handlers();

    log_open();

    if (parse_argv(argc, argv, &uartstr) != 2)
        goto close_log;

    if (mainloop.open() < 0)
        goto close_log;

    if (uart.open(uartstr, opt.baudrate) < 0)
        goto close_log;

    g_master = &uart;
    mainloop.add_fd(uart.fd, &uart, EPOLLIN);

    if (!add_endpoints(mainloop))
        goto close_log;

    mainloop.loop();

    free_endpoints();

    log_close();

    return 0;

close_log:
    log_close();
    return EXIT_FAILURE;
}
