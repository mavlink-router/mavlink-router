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
#include <sys/types.h>
#include <unistd.h>

#include "comm.h"
#include "log.h"
#include "util.h"

#define MAVLINK_TCP_PORT 5760

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

    int epollfd = -1;
    bool report_msg_statistics = false;
    bool should_process_tcp_hangups = false;
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
    int target_sysid, target_compid;
    while (endpoint->read_msg(&buf, &target_sysid, &target_compid) > 0) {
        // Forward according to target_sysid and target_compid
        if (target_sysid > 0) {
            // target is a match sysid and compid. weak_target is a match sysid only
            Endpoint *target = nullptr, *weak_target = nullptr;
            struct endpoint_entry *tcp_target = nullptr;

            // First, check if master is the one
            if (g_master->get_system_id() == target_sysid) {
                weak_target = g_master;
                if (g_master->get_component_id() == target_compid)
                    target = g_master;
            }

            // Then, check udp endpoints
            if (!target) {
                for (Endpoint **e = g_endpoints; *e != nullptr; e++) {
                    if ((*e)->get_system_id() == target_sysid) {
                        weak_target = *e;
                        if ((*e)->get_component_id() == target_compid) {
                            target = *e;
                            break;
                        }
                    }
                }
            }

            // Last, check tcp endpoints
            if (!target) {
                for (struct endpoint_entry *e = g_tcp_endpoints; e; e = e->next) {
                    if (e->endpoint->get_system_id() == target_sysid) {
                        weak_target = e->endpoint;
                        tcp_target = e;
                        if (e->endpoint->get_component_id() == target_compid) {
                            target = e->endpoint;
                            tcp_target = e;
                            break;
                        }
                    }
                }
            }

            if (!target)
                target = weak_target;

            if (target) {
                log_debug("Routing message from %u/%u to endpoint %u/%u(%u/%u)",
                          endpoint->get_system_id(), endpoint->get_component_id(),
                          target->get_system_id(), target->get_component_id(), target_sysid,
                          target_compid);

                int r = write_msg(target, &buf);
                if (r == -EPIPE && tcp_target) {
                    tcp_target->remove = true;
                    should_process_tcp_hangups = true;
                }
            } else {
                log_error("Message to unknown sysid/compid: %u/%u", target_sysid, target_compid);
            }
        } else {
            log_debug("Routing message from %u/%u to all other known endpoints",
                      endpoint->get_system_id(), endpoint->get_component_id());
            // No target_sysid, forward to all (taking care to not forward to source)
            if (endpoint != g_master)
                write_msg(g_master, &buf);

            for (Endpoint **e = g_endpoints; *e != nullptr; e++) {
                if (endpoint != *e)
                    write_msg(*e, &buf);
            }

            for (struct endpoint_entry *e = g_tcp_endpoints; e; e = e->next) {
                if (endpoint != e->endpoint) {
                    int r = write_msg(e->endpoint, &buf);
                    if (r == -EPIPE) {
                        e->remove = true;
                        should_process_tcp_hangups = true;
                    }
                }
            }
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
    int errno_copy;

    fd = tcp->accept(g_tcp_fd);

    if (fd == -1)
        goto accept_error;

    tcp_entry = (struct endpoint_entry*)calloc(1, sizeof(struct endpoint_entry));
    if (!tcp_entry)
        goto calloc_error;

    tcp_entry->next = g_tcp_endpoints;
    tcp_entry->endpoint = tcp;
    g_tcp_endpoints = tcp_entry;

    add_fd(fd, tcp, EPOLLIN);

    log_debug("Received tcp connection on fd %d", fd);
    return;

calloc_error:
    errno_copy = errno;
    close(fd);
    errno = errno_copy;
accept_error:
    log_error_errno(errno, "Could not accept tcp connection (%m)");
    delete tcp;
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
            Endpoint *e = static_cast<Endpoint*>(events[i].data.ptr);

            if (events[i].events & EPOLLIN)
                handle_read(e);

            if (events[i].events & EPOLLOUT)
                handle_canwrite(e);
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
        free((void *)e->ip);
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
