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
#include <fcntl.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/stat.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <memory>
#include <vector>

#include <common/log.h>
#include <common/util.h>

#include "autolog.h"

#define TIMEOUT_LOG_SHUTDOWN_US     5000000ULL // number of microseconds we wait until we give up trying to stop log streaming
                                        // after a shutdown of mavlink router was requested

static const char* pipe_path = "/tmp/mavlink_router_pipe";

Mainloop* Mainloop::instance = nullptr;

Mainloop::Mainloop()
{
    if (instance) {
        throw std::logic_error("Only one mainloop instance must exist at a given time");
    }

    epollfd = epoll_create1(EPOLL_CLOEXEC);

    if (epollfd == -1) {
        throw std::runtime_error(std::string("epoll_create: ") + strerror(errno));
    }

    // XXX: this is bad for testing/multi-instantiation.
    if (mkfifo(pipe_path, 0777) == -1 && errno != EEXIST) {
        throw std::runtime_error(std::string("mkfifo: ") + strerror(errno));
    }

    _pipefd = ::open(pipe_path, O_RDWR | O_CLOEXEC);
    if (_pipefd == -1) {
        throw std::runtime_error(std::string("open pipe: ") + strerror(errno));
    }
    add_fd(_pipefd, &_pipefd, EPOLLIN);

    instance = this;
}

Mainloop::~Mainloop()
{
    free_endpoints();
    _del_timeouts(); // needs to happen after endpoints are freed
    instance = nullptr;
}

Mainloop& Mainloop::get_instance()
{
    return *instance;
}

void Mainloop::request_exit()
{
    _should_exit.store(true, std::memory_order_relaxed);
}

int Mainloop::mod_fd(int fd, void *data, int events)
{
    struct epoll_event epev = { };

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_MOD, fd, &epev) < 0) {
        log_error("Could not mod fd (%m)");
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
        log_error("Could not add fd to epoll (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::remove_fd(int fd)
{
    if (epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, nullptr) < 0) {
        log_error("Could not remove fd from epoll (%m)");
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

void Mainloop::route_msg(struct buffer *buf, int target_sysid, int target_compid, int sender_sysid,
                         int sender_compid, uint32_t msg_id)
{
    bool unknown = true;

    for (const auto& e : _endpoints) {
        if (e->accept_msg(target_sysid, target_compid, sender_sysid, sender_compid, msg_id)) {
            log_debug("Endpoint [%d] accepted message to %d/%d from %u/%u", e->fd, target_sysid,
                      target_compid, sender_sysid, sender_compid);
            write_msg(e.get(), buf);
            unknown = false;
        }
    }

    for (auto i: _dynamic_endpoints) {
        if (i.second->accept_msg(target_sysid, target_compid, sender_sysid, sender_compid, msg_id)) {
            log_debug("Endpoint [%d] accepted message to %d/%d from %u/%u", i.second->fd, target_sysid,
                      target_compid, sender_sysid, sender_compid);
            write_msg(i.second, buf);
            unknown = false;
        }
    }

    for (struct endpoint_entry *e = g_tcp_endpoints; e; e = e->next) {
        if (e->endpoint->accept_msg(target_sysid, target_compid, sender_sysid, sender_compid, msg_id)) {
            log_debug("Endpoint [%d] accepted message to %d/%d from %u/%u", e->endpoint->fd,
                      target_sysid, target_compid, sender_sysid, sender_compid);
            int r = write_msg(e->endpoint, buf);
            if (r == -EPIPE) {
                should_process_tcp_hangups = true;
            }
            unknown = false;
            e->endpoint->postprocess_msg(target_sysid, target_compid, sender_sysid, sender_compid, msg_id);
        }
    }

    if (unknown) {
        _errors_aggregate.msg_to_unknown++;
        log_debug("Message to unknown sysid/compid: %u/%u", target_sysid, target_compid);
    }
}

void Mainloop::process_tcp_hangups()
{
    // First, remove entries from the beginning of list, ensuring `g_tcp_endpoints` still
    // points to list beginning
    struct endpoint_entry **first = &g_tcp_endpoints;
    while (*first && !(*first)->endpoint->is_valid()) {
        struct endpoint_entry *next = (*first)->next;
        remove_fd((*first)->endpoint->fd);
        if ((*first)->endpoint->retry_timeout > 0) {
            _add_tcp_retry((*first)->endpoint);
        } else {
            delete (*first)->endpoint;
        }
        free(*first);
        *first = next;
    }

    // Remove other entries
    if (*first) {
        struct endpoint_entry *prev = *first;
        struct endpoint_entry *current = prev->next;
        while (current) {
            if (!current->endpoint->is_valid()) {
                prev->next = current->next;
                remove_fd(current->endpoint->fd);
                if (current->endpoint->retry_timeout > 0) {
                    _add_tcp_retry(current->endpoint);
                } else {
                    delete current->endpoint;
                }
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

    add_fd(tcp->fd, tcp, EPOLLIN);

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

    log_debug("Accepted TCP connection on [%d]", fd);
    return;

add_error:
    errno_copy = errno;
    close(fd);
    errno = errno_copy;
accept_error:
    log_error("Could not accept TCP connection (%m)");
    delete tcp;
}

void Mainloop::loop()
{
    if (epollfd < 0)
        return;

    MainloopSignalHandlers handlers(this);

    add_timeout(LOG_AGGREGATE_INTERVAL_SEC * MSEC_PER_SEC,
                std::bind(&Mainloop::_log_aggregate_timeout, this, std::placeholders::_1), this);

    while (!_should_exit.load(std::memory_order_relaxed)) {
        run_single(-1);
    }

    // This is a bit weird, but models previous behavior: run event handling a
    // bit more to allow log end point to shutdown. This should instead be
    // handled in main taking care of log_endpoint *only*.
    if (_log_endpoint) {
        _log_endpoint->stop();

        usec_t now = now_usec();
        usec_t deadline = now + TIMEOUT_LOG_SHUTDOWN_US;

        while (now < deadline) {
            run_single((deadline - now) / 1000);
            now = now_usec();
        }

        _log_endpoint->stop();
    }

    // free all remaning Timeouts
    while (_timeouts) {
        Timeout *current = _timeouts;
        _timeouts = current->next;
        remove_fd(current->fd);
        delete current;
    }
}

int Mainloop::run_single(int timeout_msec)
{
    constexpr int max_events = 8;
    struct epoll_event events[max_events];

    int r = epoll_wait(epollfd, events, max_events, timeout_msec);
    if (r <= 0) {
        return 0;
    }
    for (int i = 0; i < r; i++) {
        if (events[i].data.ptr == &_pipefd) {
            _handle_pipe();
            continue;
        }
        else if (events[i].data.ptr == &g_tcp_fd) {
            handle_tcp_connection();
            continue;
        }
        Pollable *p = static_cast<Pollable *>(events[i].data.ptr);

        if (events[i].events & EPOLLIN) {
            r = p->handle_read();
            if (r < 0 && !p->is_valid()) {
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
            log_error("poll error for fd %i, closing it", p->fd);
            remove_fd(p->fd);
            // make poll errors fatal so that an external component can
            // restart mavlink-router
            request_exit();
        }
    }

    if (should_process_tcp_hangups) {
        process_tcp_hangups();
    }

    _del_timeouts();

    return r;
}

bool Mainloop::_log_aggregate_timeout(void *data)
{
    if (_errors_aggregate.msg_to_unknown > 0) {
        log_warning("%u messages to unknown endpoints in the last %d seconds",
                    _errors_aggregate.msg_to_unknown, LOG_AGGREGATE_INTERVAL_SEC);
        _errors_aggregate.msg_to_unknown = 0;
    }

    for (const auto& e : _endpoints) {
        e->log_aggregate(LOG_AGGREGATE_INTERVAL_SEC);
    }

    for (auto *t = g_tcp_endpoints; t; t = t->next) {
        t->endpoint->log_aggregate(LOG_AGGREGATE_INTERVAL_SEC);
    }
    return true;
}

void Mainloop::print_statistics()
{
    for (const auto & e : _endpoints) {
        e->print_statistics();
    }
    for (auto i: _dynamic_endpoints) {
        i.second->print_statistics();
    }
    for (auto *t = g_tcp_endpoints; t; t = t->next)
        t->endpoint->print_statistics();
}

static bool _print_statistics_timeout_cb(void *data)
{
    Mainloop *mainloop = static_cast<Mainloop *>(data);
    mainloop->print_statistics();
    return true;
}

bool Mainloop::remove_dynamic_endpoint(Endpoint *endpoint)
{
    if (!endpoint) {
        return false;
    }

    for (auto i = _dynamic_endpoints.begin(); i != _dynamic_endpoints.end(); i++) {
        if (i->second == endpoint) {
            log_info("Removing dynamic endpoint: %s", i->first.c_str());
            remove_fd(i->second->fd);
            delete i->second;
            _pipe_commands.erase(i->first);
            _dynamic_endpoints.erase(i);
            return true;
        }
    }

    return false;
}

bool Mainloop::_remove_dynamic_endpoint(std::string name)
{
    for (auto i = _dynamic_endpoints.begin(); i != _dynamic_endpoints.end(); i++) {
        if (i->first == name) {
            log_info("Removing dynamic endpoint: %s", i->first.c_str());
            remove_fd(i->second->fd);
            delete i->second;
            _pipe_commands.erase(i->first);
            _dynamic_endpoints.erase(i);
            return true;
        }
    }

    return false;
}

bool Mainloop::_add_dynamic_endpoint(std::string name, std::string command, Endpoint *endpoint)
{
    if (!endpoint) {
        return false;
    }
    log_info("Adding dynamic endpoint: %s", name.c_str());
    auto ep = _dynamic_endpoints.find(name);
    if (ep != _dynamic_endpoints.end()) {
        remove_fd(ep->second->fd);
        delete ep->second;
    }
    _pipe_commands[name] = command;
    _dynamic_endpoints[name] = endpoint;
    add_fd(endpoint->fd, endpoint, EPOLLIN);
    endpoint->start_expire_timer();

    return true;
}

bool Mainloop::add_endpoints(Mainloop &mainloop, struct options *opt)
{
    unsigned n_endpoints = 0;
    struct endpoint_config *conf;

    for (conf = opt->endpoints; conf; conf = conf->next) {
        if (conf->type != Tcp) {
            // TCP endpoints are ephemeral, that's why they don't
            // live on `_endpoints` array, but on `g_tcp_endpoints` list
            n_endpoints++;
        }
    }

    if (opt->logs_dir)
        n_endpoints++;

    for (conf = opt->endpoints; conf; conf = conf->next) {
        switch (conf->type) {
        case Uart: {
            std::unique_ptr<UartEndpoint> uart{new UartEndpoint{}};
            if (uart->open(conf->device) < 0)
                return false;

            if (conf->bauds->size() == 1) {
                if (uart->set_speed((*(conf->bauds))[0]) < 0)
                    return false;
            } else {
                if (uart->add_speeds(*conf->bauds) < 0)
                    return false;
            }

            if (conf->flowcontrol) {
                if (uart->set_flow_control(true) < 0)
                    return false;
            }

            mainloop.add_fd(uart->fd, uart.get(), EPOLLIN);
            _endpoints.push_back(std::move(uart));
            break;
        }
        case Udp: {
            std::unique_ptr<UdpEndpoint> udp{new UdpEndpoint{}};
            if (udp->open(conf->address, conf->port, conf->eavesdropping) < 0) {
                log_error("Could not open %s:%ld", conf->address, conf->port);
                return false;
            }

            udp->set_coalescing(conf->coalesce_bytes, conf->coalesce_ms);

            if (conf->filter) {
                char *local_filter = strdup(conf->filter);
                char *token = strtok(local_filter, ",");
                while (token != nullptr) {
                    udp->add_message_to_filter(atoi(token));
                    token = strtok(nullptr, ",");
                }
                free(local_filter);
            }

            if (conf->coalesce_nodelay) {
                char *local_nodelay = strdup(conf->coalesce_nodelay);
                char *token = strtok(local_nodelay, ",");
                while (token != nullptr) {
                    udp->add_message_to_nodelay(atoi(token));
                    token = strtok(nullptr, ",");
                }
                free(local_nodelay);
            }

            mainloop.add_fd(udp->fd, udp.get(), EPOLLIN);
            _endpoints.push_back(std::move(udp));
            break;
        }
        case Tcp: {
            std::unique_ptr<TcpEndpoint> tcp{new TcpEndpoint{}};
            tcp->retry_timeout = conf->retry_timeout;
            if (tcp->open(conf->address, conf->port) < 0) {
                log_error("Could not open %s:%ld.", conf->address, conf->port);
                if (tcp->retry_timeout > 0) {
                    _add_tcp_retry(tcp.release());
                }
                continue;
            }

            if (_add_tcp_endpoint(tcp.get()) < 0) {
                log_error("Could not open %s:%ld", conf->address, conf->port);
                return false;
            }
            tcp.release();
            break;
        }
        default:
            log_error("Unknow endpoint type!");
            return false;
        }
    }

    if (opt->tcp_port) {
        g_tcp_fd = tcp_open(opt->tcp_port);
    }


    if (opt->logs_dir) {
        std::unique_ptr<LogEndpoint> log_endpoint;
        if (opt->mavlink_dialect == Ardupilotmega) {
            log_endpoint.reset(
                new BinLog(opt->logs_dir, opt->log_mode, opt->min_free_space, opt->max_log_files, opt->heartbeat));
        } else if (opt->mavlink_dialect == Common) {
            log_endpoint.reset(
                new ULog(opt->logs_dir, opt->log_mode, opt->min_free_space, opt->max_log_files, opt->heartbeat));
        } else {
            log_endpoint.reset(new AutoLog(opt->logs_dir, opt->log_mode, opt->min_free_space,
                                        opt->max_log_files, opt->heartbeat));
        }
        _log_endpoint = log_endpoint.get();
        _log_endpoint->mark_unfinished_logs();
        _endpoints.push_back(std::move(log_endpoint));
    }

    if (opt->report_msg_statistics)
        add_timeout(MSEC_PER_SEC, _print_statistics_timeout_cb, this);

    return true;
}

void Mainloop::free_endpoints()
{
    // XXX not explicitly needed since only called from constructor; leaving
    // here until remainder clean.
    _endpoints.clear();

    for (auto *t = g_tcp_endpoints; t;) {
        auto next = t->next;
        delete t->endpoint;
        free(t);
        t = next;
    }

    for (auto ep = _dynamic_endpoints.begin(); ep != _dynamic_endpoints.end(); ep++) {
        delete ep->second;
    }
    _pipe_commands.clear();
    _dynamic_endpoints.clear();
}

int Mainloop::tcp_open(unsigned long tcp_port)
{
    int fd;
    struct sockaddr_in sockaddr = { };
    int val = 1;

    fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd == -1) {
        log_error("Could not create tcp socket (%m)");
        return -1;
    }

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(tcp_port);
    sockaddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        log_error("Could not bind to tcp socket (%m)");
        close(fd);
        return -1;
    }

    if (listen(fd, SOMAXCONN) < 0) {
        log_error("Could not listen on tcp socket (%m)");
        close(fd);
        return -1;
    }

    add_fd(fd, &g_tcp_fd, EPOLLIN);

    log_info("Open TCP [%d] 0.0.0.0:%lu *", fd, tcp_port);

    return fd;
}

Timeout *Mainloop::add_timeout(uint32_t timeout_msec, std::function<bool(void*)> cb, const void *data)
{
    Timeout *t = new Timeout(cb, data);

    assert_or_return(t, nullptr);

    t->fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (t->fd < 0) {
        log_error("Unable to create timerfd: %m");
        goto error;
    }

    set_timeout(t, timeout_msec);
    if (add_fd(t->fd, t, EPOLLIN) < 0)
        goto error;

    t->next = _timeouts;
    _timeouts = t;

    return t;

error:
    delete t;
    return nullptr;
}

void Mainloop::set_timeout(Timeout *t, uint32_t timeout_msec)
{
    if (!t) {
        return;
    }

    struct itimerspec ts;
    ts.it_interval.tv_sec = timeout_msec / MSEC_PER_SEC;
    ts.it_interval.tv_nsec = (timeout_msec % MSEC_PER_SEC) * NSEC_PER_MSEC;
    ts.it_value.tv_sec = ts.it_interval.tv_sec;
    ts.it_value.tv_nsec = ts.it_interval.tv_nsec;
    timerfd_settime(t->fd, 0, &ts, nullptr);
}

void Mainloop::del_timeout(Timeout *t)
{
    if (_timeouts && t) {
        t->remove_me = true;
    }
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

void Mainloop::_add_tcp_retry(TcpEndpoint *tcp)
{
    Timeout *t
        ;
    if (tcp->retry_timeout <= 0) {
        return;
    }

    tcp->close();
    t = add_timeout(MSEC_PER_SEC * tcp->retry_timeout,
            std::bind(&Mainloop::_retry_timeout_cb, this, std::placeholders::_1),
            tcp);

    if (t == nullptr) {
        log_warning("Could not create retry timeout for TCP endpoint %s:%lu\n"
                    "No attempts to reconnect will be made", tcp->get_ip(), tcp->get_port());
    }
}

bool Mainloop::_retry_timeout_cb(void *data)
{
    TcpEndpoint *tcp = (TcpEndpoint *)data;

    if (tcp->open(tcp->get_ip(), tcp->get_port()) < 0) {
        return true;
    }

    if (_add_tcp_endpoint(tcp) < 0) {
        tcp->close();
        return true;
    }

    return false;
}

void Mainloop::_handle_pipe()
{
    char cmd[1024];
    ssize_t num_read = read(_pipefd, cmd, sizeof(cmd) - 1);
    char* buffer = cmd;
    if (num_read > 0) {
        cmd[num_read] = 0;
        log_debug("Pipe read %ld bytes: %s", num_read, cmd);

        // If more than one command separated by an end of
        // line was written in the pipe, separate each command
        char* current_new_line = strchr(buffer, '\n');
        while (current_new_line != NULL) {
          *current_new_line = '\0';
          char *command = buffer;
          buffer = current_new_line+1;
          current_new_line = strchr(buffer, '\n');

          // Parse each command
          // Command Format:
          // Cmd UDP Name IP Port Eavesdropping
          // e.g. add udp application 127.0.0.1 14532 0
          std::vector<std::string> a;
          char *pch = strtok(command, " ");
          while (pch != nullptr) {
              a.emplace_back(pch);
              pch = strtok(nullptr, " \n");
          }

          if (a.size() == 2 && a[0] == "remove") {
              _remove_dynamic_endpoint(a[1]);
              continue;
          }

          if (a.size() != 6 || a[0] != "add" || a[1] != "udp") {
              continue;
          }
          std::string name = a[2];
          std::string address = a[3];
          int port = atoi(a[4].c_str());
          if (port <= 0) {
              continue;
          }

          auto pipecmd = _pipe_commands.find(name);
          if (pipecmd != _pipe_commands.end() && pipecmd->second == command) {
              auto ep = _dynamic_endpoints.find(name);
              if (ep != _dynamic_endpoints.end()) {
                  ep->second->reset_expire_timer();
                  continue;
              }
          }

          std::unique_ptr<UdpEndpoint> udp{new UdpEndpoint()};
          if (udp->open(address.c_str(), port, a[5]=="1") < 0) {
              log_error("Could not open %s:%d", address.c_str(), port);
              continue;
          }
          _add_dynamic_endpoint(name, command, udp.release());
        }
    }
}

MainloopSignalHandlers::MainloopSignalHandlers(Mainloop* mainloop)
{
    if (mainloop_instance) {
        throw std::logic_error("Only one MainloopSignalHandlers instance must exist at a given time");
    }

    mainloop_instance = mainloop;

    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = &signal_handler_function;
    sigaction(SIGTERM, &sa, &_old_sigterm);
    sigaction(SIGINT, &sa, &_old_sigint);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, &_old_sigpipe);
}

MainloopSignalHandlers::~MainloopSignalHandlers()
{
    sigaction(SIGTERM, &_old_sigterm, nullptr);
    sigaction(SIGINT, &_old_sigint, nullptr);
    sigaction(SIGPIPE, &_old_sigpipe, nullptr);

    mainloop_instance = nullptr;
}

void MainloopSignalHandlers::signal_handler_function(int signo)
{
    mainloop_instance->request_exit();
}

Mainloop* MainloopSignalHandlers::mainloop_instance{nullptr};
