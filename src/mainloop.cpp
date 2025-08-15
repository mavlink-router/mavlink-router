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
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <memory>
#include <sstream>

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
            // fall through
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

void Mainloop::handle_command_pipe()
{
    char buf[1024];
    auto bytes = read(g_commands_fd, buf, sizeof(buf) - 1);
    char *cmd = buf;

    if (bytes < 0) {
        log_error("Command Server: Error");
    } else {
        buf[bytes] = '\0';
        log_debug("Command Server: Read %zd bytes: %s", bytes, buf);

        char *current_new_line = strchr(cmd, '\n');
        while (current_new_line != NULL) {
            *current_new_line = '\0';
            char *command = cmd;
            cmd = current_new_line + 1;
            current_new_line = strchr(cmd, '\n');

            // Parse command
            std::vector<std::string> a;
            char *pch = strtok(command, " ");
            while (pch != NULL) {
                a.push_back(std::string(pch));
                pch = strtok(NULL, " \n");
            }

            if (a[0] == "add") {
                // Add command
                // add UDP Name IP Port Mode Group CoalesceBytes CoalesceMs CoalesceNoDelay
                // a0  a1   a2  a3  a4   a5   a6        a7          a8           a9
                //  allow_msg_id_out block_msg_id_out allow_src_comp_out block_src_comp_out allow_src_sys_out block_src_sys_out allow_msg_id_in
                //        a10               a11              a12                  a13             a14                a15              a16
                //  block_msg_id_in allow_src_comp_in block_src_comp_in allow_src_sys_in block_src_sys_in
                //           a17           a18                a19             a20              a21

                std::set<unsigned> argc_options = {6, 7, 10, 22};

                // Sanity checks
                if (!argc_options.count(a.size()) || a[1] != "udp") {
                    log_debug("Command Server: add command usage:\n\tadd <protocol> "
                              "<endpoint_name> <IP> <port> <endpoint_mode>");
                    log_debug("Additional optional parameters for grouping and coalescing are: "
                              "<group> <coalesce_bytes> <coalesce_ms> <coalesce_no_delay>");
                    log_debug(
                        "Additional optional parameters for filtering are: <allow_msg_id_out> "
                        "<block_msg_id_out> <allow_src_comp_out> <block_src_comp_out>");
                    log_debug("<allow_src_sys_out> <block_src_sys_out> <allow_msg_id_in> "
                              "<block_msg_id_in> <allow_src_comp_in> <block_src_comp_in> "
                              "<allow_src_sys_in> <block_src_sys_in>");
                    log_debug("Set the optional fields you wish to leave unconfigured to \"NULL\"");
                    continue;
                }
                int port = atoi(a[4].c_str());
                if (port <= 0) {
                    log_trace("Malformed port in add command");
                    continue;
                }
                auto to_create = std::find_if(
                    g_endpoints.begin(),
                    g_endpoints.end(),
                    [&a](const std::shared_ptr<Endpoint> &e) { return e->get_name() == a[2]; });
                if (to_create != g_endpoints.end()) {
                    log_trace("Endpoint named \"%s\" already exists, please choose another name",
                              a[2].c_str());
                    continue;
                }

                // Command to UDP endpoint configuration
                UdpEndpointConfig conf{};

                conf.name = a[2];
                conf.address = a[3];
                conf.port = port;

                // UDP endpoint mode
                if (a[5] == "server" || a[5] == "Server") {
                    conf.mode = UdpEndpointConfig::Mode::Server;
                } else if (a[5] == "eavesdropping" || a[5] == "Eavesdropping") {
                    conf.mode = UdpEndpointConfig::Mode::Server;
                } else if (a[5] == "receiver" || a[5] == "Receiver") {
                    conf.mode = UdpEndpointConfig::Mode::Receiver;
                } else if (a[5] == "client" || a[5] == "Client") {
                    conf.mode = UdpEndpointConfig::Mode::Client;
                } else {
                    conf.mode = UdpEndpointConfig::Mode::Client;
                }

                if (a.size() > 6) { // group name provided
                    conf.group = a[6] == "NULL" ? "" : a[6];
                }

                if (a.size() > 7) { // coalescence config provided
                    conf.coalesce_bytes = a[7] == "NULL" ? 0 : atoi(a[7].c_str());
                    conf.coalesce_ms = a[8] == "NULL" ? 0 : atoi(a[8].c_str());
                    parse_into_vector(a[9], conf.coalesce_nodelay);
                }

                if (a.size() > 10) { // filtering config provided
                    parse_into_vector(a[10], conf.allow_msg_id_out);
                    parse_into_vector(a[11], conf.block_msg_id_out);
                    parse_into_vector(a[12], conf.allow_src_comp_out);
                    parse_into_vector(a[14], conf.block_src_comp_out);
                    parse_into_vector(a[14], conf.allow_src_sys_out);
                    parse_into_vector(a[15], conf.block_src_sys_out);
                    parse_into_vector(a[16], conf.allow_msg_id_in);
                    parse_into_vector(a[17], conf.block_msg_id_in);
                    parse_into_vector(a[18], conf.allow_src_comp_in);
                    parse_into_vector(a[19], conf.block_src_comp_in);
                    parse_into_vector(a[20], conf.allow_src_sys_in);
                    parse_into_vector(a[21], conf.block_src_sys_in);
                }

                // UDP endpoint configuration to instance
                auto dynamic_udp = std::make_shared<UdpEndpoint>(conf.name);
                if (!dynamic_udp->setup(conf)) {
                    log_debug("Command Server: Could not open dynamic endpoint on %s:%d",
                              a[3].c_str(),
                              port);
                    continue;
                }

                g_endpoints.emplace_back(dynamic_udp);
                auto endpoint = g_endpoints.back();
                this->add_fd(endpoint->fd, endpoint.get(), EPOLLIN);

                // Update endpoints groups
                if (!endpoint->get_group_name().empty()) {
                    for (auto other : g_endpoints) { // find other endpoints in group
                        if (other != endpoint
                            && other->get_group_name() == endpoint->get_group_name()) {
                            endpoint->link_group_member(other);
                            other->link_group_member(endpoint);
                        }
                    }
                }
            } else if (a[0] == "remove") {
                // Remove command
                // remove Name
                //   a0    a1

                // Sanity checks
                if (a.size() != 2) {
                    log_debug("Command Server: remove command usage: \n\tremove <endpoint_name>");
                    continue;
                }

                // Remove dynamic endpoint
                // Update groups
                for (auto e : g_endpoints) {
                    e->unlink_group_member(a[1]);
                }

                auto to_delete = std::find_if(
                    g_endpoints.begin(),
                    g_endpoints.end(),
                    [&a](const std::shared_ptr<Endpoint> e) { return e->get_name() == a[1]; });

                if (to_delete == g_endpoints.end()) {
                    log_debug("No endpoint named %s", a[1].c_str());
                } else {
                    // Delete fd
                    this->remove_fd(to_delete->get()->fd);
                    // Remove from endpoint list
                    g_endpoints.erase(to_delete);
                    log_info("Removed endpoint %s", a[1].c_str());
                }

            } else {
                log_debug("Command Server: Unsupported command \'%s\'", a[0].c_str());
            }
        }
    }
}

template <typename T>
void Mainloop::parse_into_vector(const std::string &command, std::vector<T> &vector)
{
    if (command == "NULL")
        return;

    std::istringstream arguments{command};
    for (std::string token; std::getline(arguments, token, ',');) {
        vector.push_back(atoi(token.c_str()));
    }
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

            if (events[i].data.ptr == &g_commands_fd) {
                if (events[i].events & (EPOLLERR | EPOLLHUP)) {
                    // Reopen the command pipe if there was an error in reading it
                    clean_command_pipe();
                    open_command_pipe(command_pipe_path);
                }

                handle_command_pipe();
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
                if (events[i].events & EPOLLHUP && !p->is_critical()) {
                    // EPOLLHUP is an expected error, in case the TCP connection
                    // drops. In this case, we'll just need to clean up the TCP
                    // connection later, no need to panic.
                    log_debug("Non-critical error for fd %i.", p->fd);
                    should_process_tcp_hangups = true;
                } else {
                    log_error("Critical error for fd %i, exiting", p->fd);
                    request_exit(EXIT_FAILURE);
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
    clean_command_pipe();

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
        log_info("%u messages to unknown endpoints in the last %d seconds",
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
            if (other != e && other->get_group_name() == e->get_group_name()) {
                e->link_group_member(other);
            }
        }
    }

    // Create TCP server
    if (config.tcp_port != 0u) {
        g_tcp_fd = tcp_open(config.tcp_port);
    }

    // Create command server endpoint
    if (!config.command_pipe_path.empty()) {
        command_pipe_path = config.command_pipe_path;
        g_commands_fd = open_command_pipe(command_pipe_path);
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

int Mainloop::open_command_pipe(const std::string &address)
{
    mkfifo(address.c_str(), 0600);

    int fd = ::open(address.c_str(), O_RDWR | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0) {
        log_error("Failed to open Command Server: %s", address.c_str());
    } else {
        add_fd(fd, &g_commands_fd, EPOLLIN);
        log_info("Opened Command Server [%d] at %s", fd, address.c_str());
    }

    return fd;
}

void Mainloop::clean_command_pipe()
{
    // clean command pipe after use
    if (g_commands_fd != -1) {
        ::close(g_commands_fd);
        g_commands_fd = -1;
    }
    ::remove(command_pipe_path.c_str());
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
