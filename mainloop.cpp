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
#include <sys/timerfd.h>
#include <unistd.h>

#include "log.h"
#include "util.h"

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

void Mainloop::route_msg(struct buffer *buf, int target_sysid, int sender_sysid)
{
    if (target_sysid > 0) {
        Endpoint *target = nullptr;
        struct endpoint_entry *tcp_target = nullptr;

        // First, check UART and UDP endpoints
        if (!target) {
            for (Endpoint **e = g_endpoints; *e != nullptr; e++) {
                if ((*e)->get_system_id() == target_sysid) {
                    target = *e;
                    break;
                }
            }
        }

        // Last, check TCP endpoints
        if (!target) {
            for (struct endpoint_entry *e = g_tcp_endpoints; e; e = e->next) {
                if (e->endpoint->get_system_id() == target_sysid) {
                    target = e->endpoint;
                    tcp_target = e;
                    break;
                }
            }
        }

        if (target) {
            log_debug("Routing message from %u to endpoint %u[%d]", sender_sysid, target_sysid,
                      target->fd);

            int r = write_msg(target, buf);
            if (r == -EPIPE && tcp_target) {
                tcp_target->remove = true;
                should_process_tcp_hangups = true;
            }
        } else {
            log_error("Message to unknown sysid: %u", target_sysid);
        }
    } else {
        log_debug("Routing message from %u to all other known endpoints", sender_sysid);
        // No target_sysid, forward to all (taking care to not forward to source)
        for (Endpoint **e = g_endpoints; *e != nullptr; e++) {
            if (sender_sysid != (*e)->get_system_id())
                write_msg(*e, buf);
        }

        for (struct endpoint_entry *e = g_tcp_endpoints; e; e = e->next) {
            if (sender_sysid != e->endpoint->get_system_id()) {
                int r = write_msg(e->endpoint, buf);
                if (r == -EPIPE) {
                    e->remove = true;
                    should_process_tcp_hangups = true;
                }
            }
        }
    }
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

    should_process_tcp_hangups = false;
}

int Mainloop::_add_tcp_endpoint(TcpEndpoint *tcp)
{
    struct endpoint_entry *tcp_entry;

    tcp_entry = (struct endpoint_entry *)calloc(1, sizeof(struct endpoint_entry));
    if (!tcp_entry)
        return -ENOMEM;

    tcp_entry->next = g_tcp_endpoints;
    tcp_entry->endpoint = tcp;
    g_tcp_endpoints = tcp_entry;

    return 0;
}

void Mainloop::handle_tcp_connection()
{
    TcpEndpoint *tcp = new TcpEndpoint{};
    int fd;
    int errno_copy;

    fd = tcp->accept(g_tcp_fd);
    if (fd == -1)
        goto accept_error;

    if (_add_tcp_endpoint(tcp) < 0)
        goto add_error;

    add_fd(fd, tcp, EPOLLIN);

    log_debug("Accepted TCP connection on [%d]", fd);
    return;

add_error:
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
    Endpoint::set_Mainloop(this);

    if (_log_endpoint)
        _log_endpoint->start();

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
            Pollable *p = static_cast<Pollable *>(events[i].data.ptr);

            if (events[i].events & EPOLLIN)
                p->handle_read();

            if (events[i].events & EPOLLOUT) {
                if (!p->handle_canwrite()) {
                    mod_fd(p->fd, p, EPOLLIN);
                }
            }
        }

        if (should_process_tcp_hangups) {
            process_tcp_hangups();
        }

        _del_timeouts();
    }

    if (_log_endpoint)
        _log_endpoint->stop();

    // free all remaning Timeouts
    while (_timeouts) {
        Timeout *current = _timeouts;
        _timeouts = current->next;
        remove_fd(current->fd);
        delete current;
    }
}

void Mainloop::print_statistics()
{
    for (Endpoint **e = g_endpoints; *e != nullptr; e++)
        (*e)->print_statistics();
}

static bool _print_statistics_timeout_cb(void *data)
{
    Mainloop *mainloop = static_cast<Mainloop *>(data);
    mainloop->print_statistics();
    return true;
}

bool Mainloop::add_endpoints(Mainloop &mainloop, struct opt *opt)
{
    unsigned n_endpoints = 0, i = 0;
    struct endpoint_config *conf;

    for (conf = opt->endpoints; conf; conf = conf->next) {
        if (conf->type != Tcp) {
            // TCP endpoints are efemeral, that's why they don't
            // live on `g_endpoints` array, but on `g_tcp_endpoints` list
            n_endpoints++;
        }
    }

    if (opt->logs_dir)
        n_endpoints++;

    g_endpoints = (Endpoint**) calloc(n_endpoints + 1, sizeof(Endpoint*));
    assert_or_return(g_endpoints, false);

    for (conf = opt->endpoints; conf; conf = conf->next) {
        switch (conf->type) {
        case Uart: {
            std::unique_ptr<UartEndpoint> uart{new UartEndpoint{}};
            if (uart->open(conf->device, conf->baud) < 0)
                return false;

            g_endpoints[i] = uart.release();
            mainloop.add_fd(g_endpoints[i]->fd, g_endpoints[i], EPOLLIN);
            i++;
            break;
        }
        case Udp: {
            std::unique_ptr<UdpEndpoint> udp{new UdpEndpoint{}};
            if (udp->open(conf->address, conf->port, conf->eavesdropping) < 0) {
                log_error("Could not open %s:%ld", conf->address, conf->port);
                return false;
            }

            g_endpoints[i] = udp.release();
            mainloop.add_fd(g_endpoints[i]->fd, g_endpoints[i], EPOLLIN);
            i++;
            break;
        }
        case Tcp: {
            std::unique_ptr<TcpEndpoint> tcp{new TcpEndpoint{}};
            if (tcp->open(conf->address, conf->port) < 0) {
                log_error("Could not open %s:%ld", conf->address, conf->port);
                return false;
            }

            if (_add_tcp_endpoint(tcp.get()) < 0) {
                log_error("Could not open %s:%ld", conf->address, conf->port);
                return false;
            }
            mainloop.add_fd(tcp->fd, tcp.get(), EPOLLIN);
            tcp.release();
            break;
        }
        default:
            log_error("Unknow endpoint type!");
            return false;
        }
    }

    if (opt->tcp_port)
        g_tcp_fd = tcp_open(opt->tcp_port);

    if (opt->logs_dir) {
        if (streq(DIALECT, "common")) {
            _log_endpoint = new ULog(opt->logs_dir);
        } else {
            _log_endpoint = new BinLog(opt->logs_dir);
        }
        g_endpoints[i] = _log_endpoint;
    }

    if (opt->report_msg_statistics)
        add_timeout(MSEC_PER_SEC, _print_statistics_timeout_cb, this);

    return true;
}

void Mainloop::free_endpoints(struct opt *opt)
{
    for (Endpoint **e = g_endpoints; *e; e++) {
        delete *e;
    }
    free(g_endpoints);

    for (auto *t = g_tcp_endpoints; t;) {
        auto next = t->next;
        delete t->endpoint;
        free(t);
        t = next;
    }

    for (auto e = opt->endpoints; e;) {
        auto next = e->next;
        if (e->type == Udp || e->type == Tcp) {
            free(e->address);
        } else {
            free(e->device);
        }
        free(e->name);
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

    log_info("Open TCP [%d] 0.0.0.0:%lu *", fd, tcp_port);

    return fd;
}

Timeout *Mainloop::add_timeout(uint32_t timeout_msec, bool (*cb)(void *data), const void *data)
{
    struct itimerspec ts;
    Timeout *t = new Timeout(cb, data);

    assert_or_return(t, NULL);

    t->fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (t->fd < 0) {
        log_error_errno(errno, "Unable to create timerfd: %m");
        goto error;
    }

    ts.it_interval.tv_sec = timeout_msec / MSEC_PER_SEC;
    ts.it_interval.tv_nsec = (timeout_msec % MSEC_PER_SEC) * NSEC_PER_MSEC;
    ts.it_value.tv_sec = ts.it_interval.tv_sec;
    ts.it_value.tv_nsec = ts.it_interval.tv_nsec;
    timerfd_settime(t->fd, 0, &ts, NULL);

    if (add_fd(t->fd, t, EPOLLIN) < 0)
        goto error;

    t->next = _timeouts;
    _timeouts = t;

    return t;

error:
    delete t;
    return NULL;
}

void Mainloop::del_timeout(Timeout *t)
{
    t->remove_me = true;
}

void Mainloop::_del_timeouts()
{
    // Guarantee one valid Timeout on the beginning of the list
    while (_timeouts && _timeouts->remove_me) {
        Timeout *next = _timeouts->next;
        remove_fd(_timeouts->fd);
        delete _timeouts;
        _timeouts = next;
    }

    // Remove all other Timeouts
    if (_timeouts) {
        Timeout *prev = _timeouts;
        Timeout *current = _timeouts->next;
        while (current) {
            if (current->remove_me) {
                prev->next = current->next;
                remove_fd(current->fd);
                delete current;
                current = prev->next;
            } else {
                prev = current;
                current = current->next;
            }
        }
    }
}
