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
#include <memory>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <unistd.h>

#include "comm.h"
#include "log.h"
#include "util.h"

#define MAVLINK_TCP_PORT 5760

#define MAX_TIMEOUT 5

struct endpoint_entry {
    struct endpoint_entry *next;
    Endpoint *endpoint;
    bool remove;
};

class Mainloop {
public:
    int open();
    int add_fd(int fd, void *data, int events);
    int mod_fd(int fd, void *data, int events);
    int remove_fd(int fd);
    void loop();
    void handle_read(Endpoint *e);
    void handle_canwrite(Endpoint *e);
    void handle_tcp_connection();
    int write_msg(Endpoint *e, const struct buffer *buf);
    void process_tcp_hangups();
    Timeout *timeout_add(uint32_t timeout_ms, bool (*cb)(void *data), const void *data);
    void timeout_del(Timeout *t);

    int epollfd = -1;
    bool report_msg_statistics = false;
    bool should_process_tcp_hangups = false;

private:
    Timeout *_timeout_list[MAX_TIMEOUT];
    void _timeout_process_del(bool del_all);
};

struct endpoint_address {
    struct endpoint_address *next;
    const char *ip;
    unsigned long port;
};

static struct opt {
    long unsigned baudrate;
    struct endpoint_address *ep_addrs;
    unsigned long tcp_port;
    bool report_msg_statistics;
} opt = {
    .baudrate = 115200U,
    .ep_addrs = nullptr,
    .tcp_port = MAVLINK_TCP_PORT,
    .report_msg_statistics = false,
};

static Endpoint *g_master;
static Endpoint **g_endpoints;
static volatile bool g_should_exit;
static int g_tcp_fd;
static endpoint_entry *g_tcp_endpoints;

static void help(FILE *fp) {
    fprintf(fp,
            "%s [OPTIONS...] [<uart>|<udp_address>]\n\n"
            "  <uart>                       UART device that will be routed\n"
            "  <udp_address>                UDP address (<ip>:<port>) that will be routed\n"
            "  -b --baudrate <baudrate>     Use baudrate for UART\n"
            "  -e --endpoint <ip[:port]>    Add UDP endpoint to communicate port is optional\n"
            "                               and in case it's not given it starts in 14550 and\n"
            "                               continues increasing not to collide with previous\n"
            "                               ports\n"
            "  -r --report_msg_statistics   Report message statistics\n"
            "  -t --tcp-port                Port in which mavlink-router will listen for TCP\n"
            "                               connections. Pass 0 to disable TCP listening.\n"
            "                               Default port 5760\n"
            "  -h --help                    Print this message\n"
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

static int parse_argv(int argc, char *argv[], const char **uart, char **udp_addr, unsigned long *udp_port)
{
    static const struct option options[] = {
        { "baudrate",               required_argument,  NULL,   'b' },
        { "endpoints",              required_argument,  NULL,   'e' },
        { "report_msg_statistics",  no_argument,        NULL,   'r' },
        { "tcp-port",               required_argument,  NULL,   't' },
        { }
    };
    int c;
    struct stat st;

    assert(argc >= 0);
    assert(argv);
    assert(uart);
    assert(udp_port);

    *uart = NULL;
    *udp_port = 0;

    while ((c = getopt_long(argc, argv, "hb:e:rt:", options, NULL)) >= 0) {
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
        case 'r': {
            opt.report_msg_statistics = true;
            break;
        }
        case 't': {
            if (safe_atoul(optarg, &opt.tcp_port) < 0) {
                log_error("Invalid argument for tcp-port = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
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

    if (stat(argv[optind], &st) == -1 || !S_ISCHR(st.st_mode)) {
        char *ip = strdup(argv[optind]);

        char *portstr = strchrnul(ip, ':');
        if (*portstr == '\0') {
            log_error("Invalid argument for UDP address = %s. Please inform <address>:<port>", argv[optind]);
            help(stderr);
            free(ip);
            return -EINVAL;
        } else {
            *portstr = '\0';
            if (safe_atoul(portstr + 1, udp_port) < 0) {
                log_error("Invalid argument for UDP port = %s", argv[optind]);
                help(stderr);
                free(ip);
                return -EINVAL;
            }
        }
        *udp_addr = ip;
    } else {
        *uart = argv[optind];
    }

    /* Will route from UART xor UDP */
    if (*uart && *udp_port) {
        log_error("Cannot route from UART and UDP at same time!");
        help(stderr);
        return -EINVAL;
    } else if (!*uart && !*udp_port) {
        log_error("Please, inform a UART device or UDP port to route!");
        help(stderr);
        return -EINVAL;
    }

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

int Mainloop::remove_fd(int fd) {
    if (epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, NULL) < 0) {
        log_error_errno(errno, "Could not remove fd from epoll (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::write_msg(Endpoint *e, const struct buffer *buf)
{
    int r = e->write_msg(buf);

    /*
     * If endpoint would block, add EPOLLOUT event to get notified when it's
     * possible to write again
     */
    if (r == -EAGAIN)
        mod_fd(e->fd, e, EPOLLIN | EPOLLOUT);

    return r;
}

void Mainloop::handle_read(Endpoint *endpoint)
{
    assert(endpoint);

    struct buffer buf{};
    should_process_tcp_hangups = false;

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
            for (struct endpoint_entry *e = g_tcp_endpoints; e; e = e->next) {
                int r = write_msg(e->endpoint, &buf);
                if (r == -EPIPE) {
                    e->remove = true;
                    should_process_tcp_hangups = true;
                }
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

void Mainloop::process_tcp_hangups() {
    // First, remove entries from the beginning of list, ensuring `g_tcp_endpoints` still
    // points to list beginning
    struct endpoint_entry **first = &g_tcp_endpoints;
    while (*first && (*first)->remove) {
        struct endpoint_entry *next = (*first)->next;
        remove_fd((*first)->endpoint->fd);
        delete (*first)->endpoint;
        free(*first);
        *first = next;
    }

    // Remove other entries
    if (*first) {
        struct endpoint_entry *prev = *first;
        struct endpoint_entry *current = prev->next;
        while (current) {
            if (current->remove) {
                prev->next = current->next;
                remove_fd(current->endpoint->fd);
                delete current->endpoint;
                free(current);
                current = prev->next;
            } else {
                prev = current;
                current = current->next;
            }
        }
    }
}

void Mainloop::handle_tcp_connection()
{
    struct endpoint_entry *tcp_entry;
    TcpEndpoint *tcp = new TcpEndpoint{};
    int fd;

    fd = tcp->accept(g_tcp_fd);

    if (fd == -1) {
        log_error_errno(errno, "Could not accept tcp connection (%m)");
        delete tcp;
    }

    tcp_entry = (struct endpoint_entry*)calloc(1, sizeof(struct endpoint_entry));
    if (!tcp_entry) {
        log_error_errno(errno, "Could not accept tcp connection (%m)");
        delete tcp;
    }

    tcp_entry->next = g_tcp_endpoints;
    tcp_entry->endpoint = tcp;
    g_tcp_endpoints = tcp_entry;

    add_fd(fd, tcp, EPOLLIN);

    log_debug("Received tcp connection on fd %d", fd);
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
            if (events[i].data.ptr == &g_tcp_fd) {
                handle_tcp_connection();
                continue;
            }
            Pollable *p = static_cast<Pollable *>(events[i].data.ptr);

            if (Endpoint *e = dynamic_cast<Endpoint *>(p)) {
                if (events[i].events & EPOLLIN)
                    handle_read(e);

                if (events[i].events & EPOLLOUT)
                    handle_canwrite(e);

                continue;
            }

            if (Timeout *t = dynamic_cast<Timeout *>(p)) {
                uint64_t val = 0;
                int ret = read(t->fd, &val, sizeof(val));

                if (ret < 0 || val == 0 || t->remove_me)
                    continue;

                if (!t->cb((void *)t->data)) {
                    timeout_del(t);
                }

                continue;
            }
        }

        if (should_process_tcp_hangups) {
            process_tcp_hangups();
        }

        if (report_msg_statistics) {
            g_master->print_statistics();
            for (Endpoint **e = g_endpoints; *e != nullptr; e++) {
                (*e)->print_statistics();
            }
        }

        _timeout_process_del(false);
    }

    _timeout_process_del(true);
}

void Mainloop::_timeout_process_del(bool del_all)
{
    for (uint8_t i = 0; i < MAX_TIMEOUT; i++) {
        Timeout *t = _timeout_list[i];

        if (t == NULL) {
            continue;
        }
        if (!del_all && !t->remove_me) {
            continue;
        }

        _timeout_list[i] = NULL;
        remove_fd(t->fd);
        delete t;
    }
}

Timeout *Mainloop::timeout_add(uint32_t timeout_ms, bool (*cb)(void *data), const void *data)
{
    struct itimerspec ts;
    Timeout *t = new Timeout();
    bool placed = false;
    uint8_t index_placed;

    null_check(t, NULL);

    for (index_placed = 0; index_placed < MAX_TIMEOUT; index_placed++) {
        if (_timeout_list[index_placed] == NULL) {
            _timeout_list[index_placed] = t;
            placed = true;
            break;
        }
    }

    if (!placed) {
        log_error("Maximum limit of timeouts reached");
        goto error1;
    }

    t->fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (t->fd < 0) {
        log_error_errno(errno, "Unable to create timerfd: %m");
        goto error2;
    }

    ts.it_interval.tv_sec = timeout_ms / MSEC_PER_SEC;
    ts.it_interval.tv_nsec = (timeout_ms % MSEC_PER_SEC) * NSEC_PER_MSEC;
    ts.it_value.tv_sec = ts.it_interval.tv_sec;
    ts.it_value.tv_nsec = ts.it_interval.tv_nsec;
    timerfd_settime(t->fd, 0, &ts, NULL);

    if (add_fd(t->fd, t, EPOLLIN) < 0) {
        goto error2;
    }

    t->cb = cb;
    t->data = data;

    return t;

error2:
    _timeout_list[index_placed] = NULL;
error1:
    delete t;
    return NULL;
}

void Mainloop::timeout_del(Timeout *t)
{
    t->remove_me = true;
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

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
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
        std::unique_ptr<UdpEndpoint> udp{new UdpEndpoint{}};
        if (udp->open(e->ip, e->port) < 0) {
            log_error("Could not open %s:%ld", e->ip, e->port);
            return false;
        }

        g_endpoints[i] = udp.release();
        mainloop.add_fd(g_endpoints[i]->fd, g_endpoints[i], EPOLLIN);
        i++;
    }

    return true;
}

static int tcp_open(Mainloop &mainloop) {
    int fd;
    struct sockaddr_in sockaddr = { };

    fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd == -1) {
        log_error_errno(errno, "Could not create tcp socket (%m)");
        return -1;
    }

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(opt.tcp_port);
    sockaddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        log_error_errno(errno, "Could not bind to tcp socket (%m)");
        close(fd);
        return -1;
    }

    if (listen(fd, SOMAXCONN) < 0) {
        log_error_errno(errno, "Could not listen on tcp socket (%m)");
        close(fd);
        return -1;
    }

    mainloop.add_fd(fd, &g_tcp_fd, EPOLLIN);

    log_info("Open TCP 0.0.0.0:%lu", opt.tcp_port);

    return fd;
}

int main(int argc, char *argv[])
{
    unsigned long udp_port = 0;
    const char *uartstr = NULL;
    char *udp_addr = NULL;
    Mainloop mainloop{};

    setup_signal_handlers();

    log_open();

    if (parse_argv(argc, argv, &uartstr, &udp_addr, &udp_port) != 2)
        goto close_log;

    if (mainloop.open() < 0)
        goto close_log;

    if (uartstr) {
        UartEndpoint *uart = new UartEndpoint{};
        if (uart->open(uartstr, opt.baudrate) < 0)
            goto close_log;

        g_master = uart;
        mainloop.add_fd(uart->fd, uart, EPOLLIN);
    } else {
        UdpEndpoint *udp = new UdpEndpoint{};
        if (udp->open(udp_addr, udp_port, true) < 0)
            goto close_log;

        g_master = udp;
        mainloop.add_fd(udp->fd, udp, EPOLLIN);
    }

    if (!add_endpoints(mainloop))
        goto close_log;

    mainloop.report_msg_statistics = opt.report_msg_statistics;

    if (opt.tcp_port)
        g_tcp_fd = tcp_open(mainloop);

    mainloop.loop();

    free_endpoints();

    delete g_master;
    free(udp_addr);

    log_close();

    return 0;

close_log:
    delete g_master;
    free(udp_addr);
    log_close();
    return EXIT_FAILURE;
}
