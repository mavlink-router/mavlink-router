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
#include <signal.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <atomic>
#include <memory>

#include <common/log.h>
#include <common/util.h>

#include "autolog.h"
#include "tlog.h"

static std::atomic<bool> should_exit{false};

Mainloop Mainloop::_instance{};
bool Mainloop::_initialized = false;

static void exit_signal_handler(int signum)
{
    Mainloop::instance().request_exit(0);
}

static void setup_signal_handlers()
{
    struct sigaction sa = {};

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGINT, &sa, nullptr);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, nullptr);
}

Mainloop &Mainloop::init()
{
    assert(_initialized == false);

    _initialized = true;

    return _instance;
}

void Mainloop::teardown()
{
    _initialized = false;
}

Mainloop &Mainloop::instance()
{
    return _instance;
}

void Mainloop::request_exit(int retcode)
{
    _retcode = retcode;
    should_exit.store(true, std::memory_order_relaxed);
}

int Mainloop::open()
{
    _retcode = -1;

    if (epollfd != -1) {
        return -EBUSY;
    }

    epollfd = epoll_create1(EPOLL_CLOEXEC);

    if (epollfd == -1) {
        log_error("%m");
        return -1;
    }

    _retcode = 0;

    return 0;
}

int Mainloop::mod_fd(int fd, void *data, int events) const
{
    struct epoll_event epev = {};

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_MOD, fd, &epev) < 0) {
        log_error("Could not mod fd (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::add_fd(int fd, void *data, int events) const
{
    struct epoll_event epev = {};

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &epev) < 0) {
        log_error("Could not add fd to epoll (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::remove_fd(int fd) const
{
    if (epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, nullptr) < 0) {
        log_error("Could not remove fd from epoll (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::write_msg(const std::shared_ptr<Endpoint> &e, const struct buffer *buf) const
{
    int r = e->write_msg(buf);

    /*
     * If endpoint would block, add EPOLLOUT event to get notified when it's
     * possible to write again
     */
    if (r == -EAGAIN) {
        mod_fd(e->fd, e.get(), EPOLLIN | EPOLLOUT);
    }

    return r;
}

void Mainloop::route_msg(struct buffer *buf)
{
    bool unknown = true;

    for (const auto &e : this->g_endpoints) {
        auto acceptState = e->accept_msg(buf);

        switch (acceptState) {
        case Endpoint::AcceptState::Accepted:
            log_trace("Endpoint [%d] accepted message %u to %d/%d from %u/%u",
                      e->fd,
                      buf->curr.msg_id,
                      buf->curr.target_sysid,
                      buf->curr.target_compid,
                      buf->curr.src_sysid,
                      buf->curr.src_compid);
            if (write_msg(e, buf) == -EPIPE) { // only TCP endpoints should return -EPIPE
                should_process_tcp_hangups = true;
            }
            unknown = false;
            break;
        case Endpoint::AcceptState::Filtered:
            log_trace("Endpoint [%d] filtered out message %u to %d/%d from %u/%u",
                      e->fd,
                      buf->curr.msg_id,
                      buf->curr.target_sysid,
                      buf->curr.target_compid,
                      buf->curr.src_sysid,
                      buf->curr.src_compid);
            unknown = false;
            break;
        case Endpoint::AcceptState::Rejected:
            log_debug("Endpoint [%d] rejected message %u to %d/%d from %u/%u",
                      e->fd,
                      buf->curr.msg_id,
                      buf->curr.target_sysid,
                      buf->curr.target_compid,
                      buf->curr.src_sysid,
                      buf->curr.src_compid);
            break;
        default:
            break; // do nothing (will count as unknown)
        }
    }

    if (unknown) {
        _errors_aggregate.msg_to_unknown++;
        log_trace("Message %u to unknown sysid/compid: %d/%d",
                  buf->curr.msg_id,
                  buf->curr.target_sysid,
                  buf->curr.target_compid);
    }
}

void Mainloop::process_tcp_hangups()
{
    // Remove endpoints, which are invalid
    for (auto it = g_endpoints.begin(); it != g_endpoints.end();) {
        if (it->get()->get_type() == ENDPOINT_TYPE_TCP) {
            auto *tcp_endpoint = static_cast<TcpEndpoint *>(it->get());
            if (!tcp_endpoint->is_valid()) {
                it = g_endpoints.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    should_process_tcp_hangups = false;
}

void Mainloop::handle_tcp_connection()
{
    log_debug("TCP Server: New client");

    auto *tcp = new TcpEndpoint{"dynamic"};

    int fd = tcp->accept(g_tcp_fd);
    if (fd == -1) {
        goto accept_error;
    }

    g_endpoints.emplace_back(tcp);
    this->add_fd(g_endpoints.back()->fd, g_endpoints.back().get(), EPOLLIN);

    return;

accept_error:
    log_error("TCP Server: Could not accept TCP connection (%m)");
    delete tcp;
}

int Mainloop::loop()
{
    const int max_events = 8;
    struct epoll_event events[max_events];
    int r;

    if (epollfd < 0) {
        return -EINVAL;
    }

    setup_signal_handlers();

    add_timeout(LOG_AGGREGATE_INTERVAL_SEC * MSEC_PER_SEC,
                std::bind(&Mainloop::_log_aggregate_timeout, this, std::placeholders::_1),
                this);

    while (!should_exit.load(std::memory_order_relaxed)) {
        int i;

        r = epoll_wait(epollfd, events, max_events, -1);
        if (r < 0 && errno == EINTR) {
            continue;
        }

        for (i = 0; i < r; i++) {
            if (events[i].data.ptr == &g_tcp_fd) {
                handle_tcp_connection();
                continue;
            }

            auto *p = static_cast<Pollable *>(events[i].data.ptr);

            if (events[i].events & EPOLLIN) {
                int rd = p->handle_read();
                if (rd < 0 && !p->is_valid()) {
                    // Only TcpEndpoint may become invalid after a read
                    should_process_tcp_hangups = true;
                }
            }

            if (events[i].events & EPOLLOUT) {
                if (!p->handle_canwrite()) {
                    mod_fd(p->fd, p, EPOLLIN);
                }
            }

            if (events[i].events & EPOLLERR) {
                log_error("poll error for fd %i", p->fd);

                if (p->is_critical()) {
                    log_error("Critical fd %i got error, exiting", p->fd);
                    request_exit(EXIT_FAILURE);
                } else {
                    log_debug("Non-critical fd %i, error is okay.", p->fd);
                }
            }
        }

        if (should_process_tcp_hangups) {
            process_tcp_hangups();
        }

        _del_timeouts();
    }

    if (_log_endpoint != nullptr) {
        _log_endpoint->stop();
    }

    clear_endpoints();

    // free all remaning Timeouts
    while (_timeouts != nullptr) {
        Timeout *current = _timeouts;
        _timeouts = current->next;
        remove_fd(current->fd);
        delete current;
    }

    return _retcode;
}

bool Mainloop::_log_aggregate_timeout(void *data)
{
    if (_errors_aggregate.msg_to_unknown > 0) {
        log_warning("%u messages to unknown endpoints in the last %d seconds",
                    _errors_aggregate.msg_to_unknown,
                    LOG_AGGREGATE_INTERVAL_SEC);
        _errors_aggregate.msg_to_unknown = 0;
    }

    for (const auto &e : g_endpoints) {
        e->log_aggregate(LOG_AGGREGATE_INTERVAL_SEC);
    }
    return true;
}

void Mainloop::print_statistics()
{
    for (const auto &e : g_endpoints) {
        e->print_statistics();
    }
}

static bool _print_statistics_timeout_cb(void *data)
{
    auto *mainloop = static_cast<Mainloop *>(data);
    mainloop->print_statistics();
    return true;
}

bool Mainloop::dedup_check_msg(const buffer *buf)
{
    return _msg_dedup.check_packet(buf->data, buf->len)
        == Dedup::PacketStatus::NEW_PACKET_OR_TIMED_OUT;
}

bool Mainloop::add_endpoints(const Configuration &config)
{
    // Create UART and UDP endpoints
    if (config.sniffer_sysid != 0) {
        Endpoint::sniffer_sysid = config.sniffer_sysid;
        log_info("An endpoint with sysid %u on it will sniff all messages",
                 Endpoint::sniffer_sysid);
    }
    for (const auto &conf : config.uart_configs) {
        auto uart = std::make_shared<UartEndpoint>(conf.name);

        if (!uart->setup(conf)) {
            return false;
        }

        g_endpoints.push_back(uart);
        auto endpoint = g_endpoints.back();
        this->add_fd(endpoint->fd, endpoint.get(), EPOLLIN);
    }

    for (const auto &conf : config.udp_configs) {
        auto udp = std::make_shared<UdpEndpoint>(conf.name);
        
        if (!udp->setup(conf)) {
            return false;
        }

        g_endpoints.emplace_back(udp);
        auto endpoint = g_endpoints.back();
        this->add_fd(endpoint->fd, endpoint.get(), EPOLLIN);
    }

    // Create TCP endpoints
    for (const auto &conf : config.tcp_configs) {
        auto tcp = std::make_shared<TcpEndpoint>(conf.name);

        if (!tcp->setup(conf)) { // handles reconnect and add_fd
            return false;        // only on fatal errors
        }

        g_endpoints.emplace_back(tcp);
    }

    // Link grouped endpoints together
    for (auto e : g_endpoints) {
        if (e->get_group_name().empty()) {
            continue;
        }

        for (auto other : g_endpoints) { // find other endpoints in group
            if (other != e && e->get_group_name() == e->get_group_name()) {
                e->link_group_member(other);
            }
        }
    }

    // Create TCP server
    if (config.tcp_port != 0u) {
        g_tcp_fd = tcp_open(config.tcp_port);
    }

    // Create Log endpoint
    auto conf = config.log_config;
    if (!conf.logs_dir.empty()) {
        switch (conf.mavlink_dialect) {
        case LogOptions::MavDialect::Ardupilotmega:
            this->_log_endpoint = std::make_shared<BinLog>(conf);
            break;

        case LogOptions::MavDialect::Common:
            this->_log_endpoint = std::make_shared<ULog>(conf);
            break;

        case LogOptions::MavDialect::Auto:
            this->_log_endpoint = std::make_shared<AutoLog>(conf);
            break;

            // no default case on purpose
        }
        this->_log_endpoint->mark_unfinished_logs();
        g_endpoints.emplace_back(this->_log_endpoint);

        if (conf.log_telemetry) {
            auto tlog_endpoint = std::make_shared<TLog>(conf);
            tlog_endpoint->mark_unfinished_logs();
            g_endpoints.emplace_back(tlog_endpoint);
        }
    }

    // Apply other options
    if (config.report_msg_statistics) {
        add_timeout(MSEC_PER_SEC, _print_statistics_timeout_cb, this);
    }

    if (config.dedup_period_ms > 0) {
        log_info("Message de-duplication enabled: %ld ms period", config.dedup_period_ms);
        _msg_dedup.set_dedup_period(config.dedup_period_ms);
    }

    return true;
}

void Mainloop::clear_endpoints()
{
    g_endpoints.clear();
}

int Mainloop::tcp_open(unsigned long tcp_port)
{
    int fd;
    struct sockaddr_in6 sockaddr = {};
    int val = 1;

    fd = socket(AF_INET6, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd == -1) {
        log_error("TCP Server: Could not create tcp socket (%m)");
        return -1;
    }

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));

    sockaddr.sin6_family = AF_INET6;
    sockaddr.sin6_port = htons(tcp_port);
    sockaddr.sin6_addr = in6addr_any;

    if (bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        log_error("TCP Server: Could not bind to tcp socket (%m)");
        close(fd);
        return -1;
    }

    if (listen(fd, SOMAXCONN) < 0) {
        log_error("TCP Server: Could not listen on tcp socket (%m)");
        close(fd);
        return -1;
    }

    add_fd(fd, &g_tcp_fd, EPOLLIN);

    log_info("Opened TCP Server [%d] [::]:%lu", fd, tcp_port);

    return fd;
}

Timeout *Mainloop::add_timeout(uint32_t timeout_msec, std::function<bool(void *)> cb,
                               const void *data)
{
    auto *t = new Timeout(cb, data);

    assert_or_return(t, nullptr);

    t->fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (t->fd < 0) {
        log_error("Unable to create timerfd: %m");
        goto error;
    }

    mod_timeout(t, timeout_msec);

    if (add_fd(t->fd, t, EPOLLIN) < 0) {
        goto error;
    }

    t->next = _timeouts;
    _timeouts = t;

    return t;

error:
    delete t;
    return nullptr;
}

void Mainloop::del_timeout(Timeout *t)
{
    t->remove_me = true;
}

void Mainloop::mod_timeout(Timeout *t, uint32_t timeout_msec)
{
    struct itimerspec ts;

    ts.it_interval.tv_sec = timeout_msec / MSEC_PER_SEC;
    ts.it_interval.tv_nsec = (timeout_msec % MSEC_PER_SEC) * NSEC_PER_MSEC;
    ts.it_value.tv_sec = ts.it_interval.tv_sec;
    ts.it_value.tv_nsec = ts.it_interval.tv_nsec;

    timerfd_settime(t->fd, 0, &ts, nullptr);
}

void Mainloop::_del_timeouts()
{
    // Guarantee one valid Timeout on the beginning of the list
    while ((_timeouts != nullptr) && _timeouts->remove_me) {
        Timeout *next = _timeouts->next;
        remove_fd(_timeouts->fd);
        delete _timeouts;
        _timeouts = next;
    }

    // Remove all other Timeouts
    if (_timeouts != nullptr) {
        Timeout *prev = _timeouts;
        Timeout *current = _timeouts->next;
        while (current != nullptr) {
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
