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
#pragma once

#include <signal.h>

#include <atomic>
#include <string>
#include <map>

#include "binlog.h"
#include "comm.h"
#include "endpoint.h"
#include "timeout.h"
#include "ulog.h"

struct endpoint_entry {
    struct endpoint_entry *next;
    TcpEndpoint *endpoint;
};

class Mainloop {
public:
    /*
     * Sets up main event handling loop. There can be only a single instance
     * of it in the whole process at any given point in time.
     */
    Mainloop();
    Mainloop(const Mainloop &) = delete;
    Mainloop &operator=(const Mainloop &) = delete;

    ~Mainloop();

    int add_fd(int fd, void *data, int events);
    int mod_fd(int fd, void *data, int events);
    int remove_fd(int fd);
    void loop();

    /*
     * Runs single iteration of mainloop, i.e. single round of event handling.
     * This will wait for at most the given time (indefinitely if -1) and
     * within this time dispatch of all handling events (socket events &
     * timers.
     */
    int run_single(int timeout_msec);

    void route_msg(struct buffer *buf, int target_sysid, int target_compid, int sender_sysid,
                   int sender_compid, uint32_t msg_id = UINT32_MAX);
    void handle_read(Endpoint *e);
    void handle_canwrite(Endpoint *e);
    void handle_tcp_connection();
    int write_msg(Endpoint *e, const struct buffer *buf);
    void process_tcp_hangups();
    Timeout *add_timeout(uint32_t timeout_msec, std::function<bool(void*)> cb, const void *data);
    void set_timeout(Timeout *t, uint32_t timeout_msec);
    void del_timeout(Timeout *t);

    bool add_endpoints(Mainloop &mainloop, struct options *opt);
    bool remove_dynamic_endpoint(Endpoint *endpoint);

    void print_statistics();

    int epollfd = -1;
    bool should_process_tcp_hangups = false;

    /*
     * Return singleton for this class, tied to the main thread.
     */
    static Mainloop &get_instance();

    /*
     * Request that loop exits "eventually". This (and only this!) function
     * is async-signal safe.
     */
    void request_exit();

    /*
     * Expose list of registered endpoints (primarily for direct interaction
     * in tests).
     */
    inline const std::vector<std::unique_ptr<Endpoint>>& endpoints() const
    {
        return _endpoints;
    }

private:
    static const unsigned int LOG_AGGREGATE_INTERVAL_SEC = 5;

    endpoint_entry *g_tcp_endpoints = nullptr;
    std::vector<std::unique_ptr<Endpoint>> _endpoints;
    int g_tcp_fd = -1;
    LogEndpoint *_log_endpoint = nullptr;

    std::map<std::string, std::string> _pipe_commands;
    std::map<std::string, Endpoint *> _dynamic_endpoints;
    int _pipefd = -1;
    struct options* _options{nullptr};

    Timeout *_timeouts = nullptr;

    std::atomic<bool> _should_exit {false};

    struct {
        uint32_t msg_to_unknown = 0;
    } _errors_aggregate;

    void free_endpoints();
    int tcp_open(unsigned long tcp_port);
    void _del_timeouts();
    int _add_tcp_endpoint(TcpEndpoint *tcp);
    void _add_tcp_retry(TcpEndpoint *tcp);
    bool _retry_timeout_cb(void *data);
    bool _log_aggregate_timeout(void *data);
    bool _add_dynamic_endpoint(std::string name, std::string command, Endpoint *endpoint);
    void _handle_pipe();

    static Mainloop* instance;
};

class MainloopSignalHandlers {
public:
    explicit MainloopSignalHandlers(Mainloop* mainloop);
    ~MainloopSignalHandlers();

private:
    static void signal_handler_function(int signo);

    static Mainloop* mainloop_instance;

    struct sigaction _old_sigterm;
    struct sigaction _old_sigint;
    struct sigaction _old_sigpipe;
};

enum endpoint_type { Tcp, Uart, Udp, Unknown };
enum mavlink_dialect { Auto, Common, Ardupilotmega };

struct endpoint_config {
    struct endpoint_config *next;
    char *name;
    enum endpoint_type type;
    union {
        struct {
            char *address;
            long unsigned port;
            int retry_timeout;
            bool eavesdropping;     // bind to local port specified, instead of send to remote port
            int coalesce_ms;        // max time to hold data to try to send packets together
            int coalesce_bytes;     // never send packets larger than this size
            char *coalesce_nodelay; // immediately send if a mavlink msg_id is matching this
        };
        struct {
            char *device;
            std::vector<unsigned long> *bauds;
            bool flowcontrol;
        };
    };
    char *filter;
};

struct options {
    struct endpoint_config *endpoints;
    const char *conf_file_name;
    const char *conf_dir;
    unsigned long tcp_port;
    bool report_msg_statistics;
    char *logs_dir;
    LogMode log_mode;
    int debug_log_level;
    enum mavlink_dialect mavlink_dialect;
    unsigned long min_free_space;
    unsigned long max_log_files;
};
