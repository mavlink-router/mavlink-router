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
    int open();
    int add_fd(int fd, void *data, int events);
    int mod_fd(int fd, void *data, int events);
    int remove_fd(int fd);
    void loop();
    void route_msg(struct buffer *buf, int target_sysid, int target_compid, int sender_sysid,
                   int sender_compid);
    void handle_read(Endpoint *e);
    void handle_canwrite(Endpoint *e);
    void handle_tcp_connection();
    int write_msg(Endpoint *e, const struct buffer *buf);
    void process_tcp_hangups();
    Timeout *add_timeout(uint32_t timeout_msec, std::function<bool(void*)> cb, const void *data);
    void del_timeout(Timeout *t);

    void free_endpoints(struct options *opt);
    bool add_endpoints(Mainloop &mainloop, struct options *opt);

    void print_statistics();

    int epollfd = -1;
    bool should_process_tcp_hangups = false;

    /*
     * Return singleton for this class, tied to the main thread. It needds to
     * be called after a call to Mainloop::init().
     */
    static Mainloop &get_instance()
    {
        assert(_initialized);
        return _instance;
    }

    /*
     * Initialize and return singleton.
     */
    static Mainloop &init();

private:
    static const unsigned int LOG_AGGREGATE_INTERVAL_SEC = 5;

    endpoint_entry *g_tcp_endpoints = nullptr;
    Endpoint **g_endpoints = nullptr;
    int g_tcp_fd = -1;
    LogEndpoint *_log_endpoint = nullptr;

    Timeout *_timeouts = nullptr;

    struct {
        uint32_t msg_to_unknown = 0;
    } _errors_aggregate;

    int tcp_open(unsigned long tcp_port);
    void _del_timeouts();
    int _add_tcp_endpoint(TcpEndpoint *tcp);
    void _add_tcp_retry(TcpEndpoint *tcp);
    bool _retry_timeout_cb(void *data);
    bool _log_aggregate_timeout(void *data);

    Mainloop() { }
    Mainloop(const Mainloop &) = delete;
    Mainloop &operator=(const Mainloop &) = delete;

    static Mainloop _instance;
    static bool _initialized;
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
            bool eavesdropping;
        };
        struct {
            char *device;
            std::vector<unsigned long> *bauds;
            bool flowcontrol;
        };
    };
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
};
