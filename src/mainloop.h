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

#include <memory>
#include <string>
#include <vector>

#include "common/log.h"

#include "binlog.h"
#include "comm.h"
#include "endpoint.h"
#include "timeout.h"
#include "ulog.h"

struct Configuration {
    std::string conf_file_name; ///< CLI "conf-file" only!
    std::string conf_dir;       ///< CLI "conf-dir" only!
    unsigned long tcp_port;     ///< conf "TcpServerPort" or CLI "tcp-port"
    bool report_msg_statistics; ///< conf "ReportStats" or CLI "report_msg_statistics"
    Log::Level debug_log_level; ///< conf "DebugLogLevel" or CLI "debug-log-level"

    LogOptions log_config; ///< logging is in General config section, but internally an endpoint
    std::vector<UartEndpointConfig> uart_configs;
    std::vector<UdpEndpointConfig> udp_configs;
    std::vector<TcpEndpointConfig> tcp_configs;
};

struct endpoint_entry {
    struct endpoint_entry *next;
    TcpEndpoint *endpoint;
};

class Mainloop {
public:
    int open();
    int add_fd(int fd, void *data, int events) const;
    int mod_fd(int fd, void *data, int events) const;
    int remove_fd(int fd) const;
    int loop();
    void route_msg(struct buffer *buf, int target_sysid, int target_compid, int sender_sysid,
                   int sender_compid, uint32_t msg_id = UINT32_MAX);
    void handle_tcp_connection();
    int write_msg(const std::shared_ptr<Endpoint> &e, const struct buffer *buf) const;
    void process_tcp_hangups();
    Timeout *add_timeout(uint32_t timeout_msec, std::function<bool(void *)> cb, const void *data);
    void del_timeout(Timeout *t);

    bool add_endpoints(const Configuration &config);

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

    static Mainloop &instance();

    /*
     * Request that loop exits on next iteration.
     */
    void request_exit(int retcode);

    void add_tcp_retry(TcpEndpoint *tcp);

private:
    static const unsigned int LOG_AGGREGATE_INTERVAL_SEC = 5;

    std::vector<std::shared_ptr<Endpoint>> g_endpoints{};
    int g_tcp_fd = -1; ///< for TCP server
    std::shared_ptr<LogEndpoint> _log_endpoint{nullptr};

    Timeout *_timeouts = nullptr;

    struct {
        uint32_t msg_to_unknown = 0;
    } _errors_aggregate;

    int _retcode;

    int tcp_open(unsigned long tcp_port);
    void _del_timeouts();
    void _add_tcp_endpoint(TcpEndpoint *tcp);
    bool _retry_timeout_cb(void *data);
    bool _log_aggregate_timeout(void *data);

    Mainloop() = default;
    Mainloop(const Mainloop &) = delete;
    Mainloop &operator=(const Mainloop &) = delete;

    static Mainloop _instance;
    static bool _initialized;
};
