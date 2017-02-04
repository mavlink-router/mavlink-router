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
#include "mainloop.h"

#include <assert.h>
#include <memory>
#include <signal.h>
#include <sys/epoll.h>
#include <unistd.h>

#include "log.h"

static volatile bool should_exit = false;

static void exit_signal_handler(int signum)
{
    should_exit = true;
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

int Mainloop::remove_fd(int fd)
{
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

            // First, check udp endpoints
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
                log_info("Routing message from %u/%u[%d] to endpoint %u/%u(%u/%u)[%d]",
                         endpoint->get_system_id(), endpoint->get_component_id(), endpoint->fd,
                         target->get_system_id(), target->get_component_id(), target_sysid,
                         target_compid, target->fd);

                int r = write_msg(target, &buf);
                if (r == -EPIPE && tcp_target) {
                    tcp_target->remove = true;
                    should_process_tcp_hangups = true;
                }
            } else {
                log_error("Message to unknown sysid/compid: %u/%u", target_sysid, target_compid);
            }
        } else {
            log_info("Routing message from %u/%u[%d] to all other known endpoints",
                     endpoint->get_system_id(), endpoint->get_component_id(), endpoint->fd);
            // No target_sysid, forward to all (taking care to not forward to source)
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

void Mainloop::process_tcp_hangups()
{
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

    log_debug("Accepted TCP connection on [%d]", fd);
    return;

calloc_error:
    errno_copy = errno;
    close(fd);
    errno = errno_copy;
accept_error:
    log_error_errno(errno, "Could not accept TCP connection (%m)");
    delete tcp;
}

void Mainloop::loop()
{
    const int max_events = 8;
    struct epoll_event events[max_events];
    int r;

    if (epollfd < 0)
        return;

    setup_signal_handlers();

    while (!should_exit) {
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
            for (Endpoint **e = g_endpoints; *e != nullptr; e++) {
                (*e)->print_statistics();
            }
        }
    }
}

bool Mainloop::add_endpoints(Mainloop &mainloop, const char *uartstr, struct opt *opt)
{
    unsigned n_endpoints = 0, i = 0;
    struct endpoint_address *e;

    if (uartstr)
        n_endpoints++;

    for (e = opt->master_addrs; e; e = e->next)
        n_endpoints++;

    for (e = opt->ep_addrs; e; e = e->next)
        n_endpoints++;

    g_endpoints = (Endpoint**) calloc(n_endpoints + 1, sizeof(Endpoint*));

    if (uartstr) {
        std::unique_ptr<UartEndpoint> uart{new UartEndpoint{}};
        if (uart->open(uartstr, opt->baudrate) < 0)
            return false;

        g_endpoints[i] = uart.release();
        mainloop.add_fd(g_endpoints[i]->fd, g_endpoints[i], EPOLLIN);
        i++;
    }

    for (e = opt->master_addrs; e; e = e->next) {
        std::unique_ptr<UdpEndpoint> udp{new UdpEndpoint{}};
        if (udp->open(e->ip, e->port, true) < 0) {
            log_error("Could not open %s:%ld", e->ip, e->port);
            return false;
        }

        g_endpoints[i] = udp.release();
        mainloop.add_fd(g_endpoints[i]->fd, g_endpoints[i], EPOLLIN);
        i++;
    }

    for (e = opt->ep_addrs; e; e = e->next) {
        std::unique_ptr<UdpEndpoint> udp{new UdpEndpoint{}};
        if (udp->open(e->ip, e->port) < 0) {
            log_error("Could not open %s:%ld", e->ip, e->port);
            return false;
        }

        g_endpoints[i] = udp.release();
        mainloop.add_fd(g_endpoints[i]->fd, g_endpoints[i], EPOLLIN);
        i++;
    }

    if (opt->tcp_port)
        g_tcp_fd = tcp_open(opt->tcp_port);

    return true;
}

void Mainloop::free_endpoints(struct opt *opt)
{
    for (Endpoint **e = g_endpoints; *e; e++) {
        delete *e;
    }
    free(g_endpoints);

    for (auto e = opt->ep_addrs; e;) {
        auto next = e->next;
        free((void *)e->ip);
        free(e);
        e = next;
    }
}

int Mainloop::tcp_open(unsigned long tcp_port)
{
    int fd;
    struct sockaddr_in sockaddr = { };

    fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd == -1) {
        log_error_errno(errno, "Could not create tcp socket (%m)");
        return -1;
    }

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(tcp_port);
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

    add_fd(fd, &g_tcp_fd, EPOLLIN);

    log_info("Open TCP [%d] 0.0.0.0:%lu", fd, tcp_port);

    return fd;
}
