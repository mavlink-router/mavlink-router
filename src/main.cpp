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
#include <dirent.h>
#include <getopt.h>
#include <limits.h>
#include <stddef.h>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <sys/stat.h>

#include <common/conf_file.h>
#include <common/dbg.h>
#include <common/log.h>
#include <common/util.h>

#include "comm.h"
#include "endpoint.h"
#include "logendpoint.h"
#include "mainloop.h"

#define DEFAULT_CONFFILE "/etc/mavlink-router/main.conf"
#define DEFAULT_CONF_DIR "/etc/mavlink-router/config.d"

extern const char *BUILD_VERSION;

static const struct option long_options[] = {{"endpoints", required_argument, nullptr, 'e'},
                                             {"conf-file", required_argument, nullptr, 'c'},
                                             {"conf-dir", required_argument, nullptr, 'd'},
                                             {"report_msg_statistics", no_argument, nullptr, 'r'},
                                             {"tcp-port", required_argument, nullptr, 't'},
                                             {"tcp-endpoint", required_argument, nullptr, 'p'},
                                             {"log", required_argument, nullptr, 'l'},
                                             {"telemetry-log", no_argument, nullptr, 'T'},
                                             {"debug-log-level", required_argument, nullptr, 'g'},
                                             {"verbose", no_argument, nullptr, 'v'},
                                             {"version", no_argument, nullptr, 'V'},
                                             {"sniffer-sysid", required_argument, nullptr, 's'},
                                             {}};

static const char *short_options = "he:rt:c:d:l:p:g:vV:s:T:";

static void help(FILE *fp)
{
    fprintf(
        fp,
        "%s [OPTIONS...] [<uart>|<udp_address>]\n\n"
        "  <uart>                       UART device (<device>[:<baudrate>]) that will be routed\n"
        "  <udp_address>                UDP address (<ip>:<port>) that will be routed\n"
        "                               ('server' mode)\n"
        "  -e --endpoint <ip[:port]>    Add UDP endpoint to communicate. Port is optional\n"
        "                               and in case it's not given it starts in 14550 and\n"
        "                               continues increasing not to collide with previous\n"
        "                               ports. 'normal' mode\n"
        "  -p --tcp-endpoint <ip:port>  Add TCP endpoint client, which will connect to given\n"
        "                               address\n"
        "  -r --report_msg_statistics   Report message statistics\n"
        "  -t --tcp-port <port>         Port in which mavlink-router will listen for TCP\n"
        "                               connections. Pass 0 to disable TCP listening.\n"
        "                               Default port 5760\n"
        "  -c --conf-file <file>        .conf file with configurations for mavlink-router.\n"
        "  -d --conf-dir <dir>          Directory where to look for .conf files overriding\n"
        "                               default conf file.\n"
        "  -l --log <directory>         Enable Flight Stack logging\n"
        "  -T --telemetry-log           Enable Telemetry logging. Only works if Flight\n"
        "                               logging directoy is set.\n"
        "  -g --debug-log-level <level> Set debug log level. Levels are\n"
        "                               <error|warning|info|debug>\n"
        "  -v --verbose                 Verbose. Same as --debug-log-level=debug\n"
        "  -V --version                 Show version\n"
        "  -s --sniffer-sysid           Sysid that all messages are sent to.\n"
        "  -h --help                    Print this message\n",
        program_invocation_short_name);
}

static uint32_t find_next_udp_port(const std::string &ip, const Configuration &config)
{
    uint32_t port = 14550U;

    for (const auto &c : config.udp_configs) {
        if (ip == c.address && port == c.port) {
            port++;
            break;
        }
    }

    return port;
}

static int split_on_last_colon(const char *str, char **base, unsigned long *number)
{
    char *colonstr;

    *base = strdup(str);
    colonstr = strrchr(*base, ':');
    *number = ULONG_MAX;

    if (colonstr != nullptr) {
        *colonstr = '\0';
        if (safe_atoul(colonstr + 1, number) < 0) {
            free(*base);
            return -EINVAL;
        }
    }

    return 0;
}

static Log::Level log_level_from_str(const char *str)
{
    if (strcaseeq(str, "error")) {
        return Log::Level::ERROR;
    }
    if (strcaseeq(str, "warning")) {
        return Log::Level::WARNING;
    }
    if (strcaseeq(str, "info")) {
        return Log::Level::INFO;
    }
    if (strcaseeq(str, "debug")) {
        return Log::Level::DEBUG;
    }

    throw std::invalid_argument("log_level_from_str: unkown string value");
}

static bool pre_parse_argv(int argc, char *argv[], Configuration &config)
{
    // This function parses only conf-file and conf-dir from
    // command line, so we can read the conf files.
    // parse_argv will then parse all other options, overriding
    // config files definitions

    int c;

    while ((c = getopt_long(argc, argv, short_options, long_options, nullptr)) >= 0) {
        switch (c) {
        case 'c': {
            config.conf_file_name = optarg;
            break;
        }
        case 'd': {
            config.conf_dir = optarg;
            break;
        }
        case 'V':
            printf(PACKAGE " version %s\n", BUILD_VERSION);
            return false;
        }
    }

    // Reset getopt*
    optind = 1;

    return true;
}

static int parse_argv(int argc, char *argv[], Configuration &config)
{
    int c;
    struct stat st;

    assert(argc >= 0);
    assert(argv);

    while ((c = getopt_long(argc, argv, short_options, long_options, nullptr)) >= 0) {
        switch (c) {
        case 'h':
            help(stdout);
            return 0;
        case 'e': {
            char *ip;
            unsigned long port;
            UdpEndpointConfig opt_udp{};
            opt_udp.name = "CLI";
            opt_udp.mode = UdpEndpointConfig::Mode::Client;

            if (split_on_last_colon(optarg, &ip, &port) < 0) {
                log_error("Invalid port in argument: %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            if (ULONG_MAX != port) {
                opt_udp.port = port;
            } else {
                opt_udp.port = find_next_udp_port(opt_udp.address, config);
            }

            opt_udp.address = std::string(ip);

            if (!UdpEndpoint::validate_config(opt_udp)) {
                free(ip);
                help(stderr); // error message was already logged by validate_config()
                return -EINVAL;
            }

            config.udp_configs.push_back(opt_udp);

            free(ip);
            break;
        }
        case 'r': {
            config.report_msg_statistics = true;
            break;
        }
        case 't': {
            if (safe_atoul(optarg, &config.tcp_port) < 0) {
                log_error("Invalid argument for tcp-port = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            break;
        }
        case 'l': {
            config.log_config.logs_dir.assign((const char *)optarg);
            break;
        }
        case 'T': {
            config.log_config.log_telemetry = true;
            break;
        }
        case 'g': {
            try {
                config.debug_log_level = log_level_from_str(optarg);
            } catch (const std::exception &e) {
                log_error("Invalid argument for debug-log-level = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            break;
        }
        case 'v': {
            config.debug_log_level = Log::Level::DEBUG;
            break;
        }
        case 's': {
            uint16_t id = atoi(optarg);
            if ((id == 0) || (id > 255)) {
                log_error("Invalid sniffer sysid %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            config.sniffer_sysid = id;
            break;
        }
        case 'p': {
            char *ip;
            unsigned long port;

            TcpEndpointConfig opt_tcp{};
            opt_tcp.name = "CLI";

            if (split_on_last_colon(optarg, &ip, &port) < 0) {
                log_error("Invalid port in argument: %s", optarg);
                help(stderr);
                return -EINVAL;
            }

            opt_tcp.port = port;
            if (ULONG_MAX == opt_tcp.port) {
                log_error("Missing port in argument: %s", optarg);
                free(ip);
                help(stderr);
                return -EINVAL;
            }

            opt_tcp.address = std::string(ip);

            if (!TcpEndpoint::validate_config(opt_tcp)) {
                free(ip);
                help(stderr); // error message was already logged by validate_config()
                return -EINVAL;
            }

            config.tcp_configs.push_back(opt_tcp);

            free(ip);
            break;
        }
        case 'c':
        case 'd':
        case 'V':
            break; // These options were parsed on pre_parse_argv
        case '?':
        default:
            help(stderr);
            return -EINVAL;
        }
    }

    /* positional arguments */
    while (optind < argc) {
        // UDP and UART master endpoints are of the form:
        // UDP: <ip>:<port> UART: <device>[:<baudrate>]
        char *base;
        unsigned long number;

        if (split_on_last_colon(argv[optind], &base, &number) < 0) {
            log_error("Invalid argument %s", argv[optind]);
            help(stderr);
            return -EINVAL;
        }

        if (stat(base, &st) == -1 || !S_ISCHR(st.st_mode)) {
            UdpEndpointConfig opt_udp{};
            opt_udp.name = "CLI";
            opt_udp.mode = UdpEndpointConfig::Mode::Server;

            opt_udp.port = number;
            if (ULONG_MAX == opt_udp.port) {
                log_error("Invalid argument for UDP port = %s", argv[optind]);
                help(stderr);
                free(base);
                return -EINVAL;
            }

            opt_udp.address = std::string(base);

            if (!UdpEndpoint::validate_config(opt_udp)) {
                help(stderr); // error message was already logged by validate_config()
                return -EINVAL;
            }

            config.udp_configs.push_back(opt_udp);
        } else {
            UartEndpointConfig opt_uart{};
            opt_uart.name = "CLI";
            opt_uart.device = std::string(base);

            const char *bauds = number != ULONG_MAX ? base + strlen(base) + 1 : nullptr;
            if (bauds != nullptr) {
                opt_uart.baudrates.push_back(atoi(bauds));
            } else {
                opt_uart.baudrates.push_back(DEFAULT_BAUDRATE);
            }

            config.uart_configs.push_back(opt_uart);
        }
        free(base);
        optind++;
    }

    return 2;
}

static std::string get_conf_file_name(const Configuration &config)
{
    char *s;

    if (!config.conf_file_name.empty()) {
        return config.conf_file_name;
    }

    s = getenv("MAVLINK_ROUTERD_CONF_FILE");
    if (s != nullptr) {
        return std::string(s);
    }

    return DEFAULT_CONFFILE;
}

static std::string get_conf_dir(const Configuration &config)
{
    char *s;

    if (!config.conf_dir.empty()) {
        return config.conf_dir;
    }

    s = getenv("MAVLINK_ROUTERD_CONF_DIR");
    if (s != nullptr) {
        return std::string(s);
    }

    return DEFAULT_CONF_DIR;
}

#define MAX_LOG_LEVEL_SIZE 10
static int parse_log_level(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    assert(val);
    assert(storage);
    assert(val_len);

    if (storage_len < sizeof(Configuration::debug_log_level)) {
        return -ENOBUFS;
    }
    if (val_len > MAX_LOG_LEVEL_SIZE) {
        return -EINVAL;
    }

    const char *log_level = strndupa(val, val_len);
    try {
        auto *level = (Log::Level *)storage;
        *level = log_level_from_str(log_level);
    } catch (const std::exception &e) {
        log_error("Invalid argument for DebugLogLevel = %s", log_level);
        return -EINVAL;
    }

    return 0;
}
#undef MAX_LOG_LEVEL_SIZE

static int parse_confs(ConfFile &conffile, Configuration &config)
{
    int ret;
    size_t offset;
    struct ConfFile::section_iter iter;
    // clang-format off
    static const ConfFile::OptionsTable global_option_table[] = {
        {"TcpServerPort",       false, ConfFile::parse_ul,      OPTIONS_TABLE_STRUCT_FIELD(Configuration, tcp_port)},
        {"ReportStats",         false, ConfFile::parse_bool,    OPTIONS_TABLE_STRUCT_FIELD(Configuration, report_msg_statistics)},
        {"DebugLogLevel",       false, parse_log_level,         OPTIONS_TABLE_STRUCT_FIELD(Configuration, debug_log_level)},
        {"DeduplicationPeriod", false, ConfFile::parse_ul,      OPTIONS_TABLE_STRUCT_FIELD(Configuration, dedup_period_ms)},
        {"SnifferSysid",    false, ConfFile::parse_ul,      OPTIONS_TABLE_STRUCT_FIELD(Configuration, sniffer_sysid)},
        {}
    };
    // clang-format on

    ret = conffile.extract_options("General", global_option_table, &config);
    if (ret < 0) {
        return ret;
    }

    ret = conffile.extract_options("General", LogEndpoint::option_table, &config.log_config);
    if (ret < 0) {
        return ret;
    }

    iter = {};
    offset = strlen(UartEndpoint::section_pattern) - 1;
    while (conffile.get_sections(UartEndpoint::section_pattern, &iter) == 0) {
        UartEndpointConfig opt_uart{};
        opt_uart.name = std::string(iter.name + offset, iter.name_len - offset);

        ret = conffile.extract_options(&iter, UartEndpoint::option_table, &opt_uart);
        if (ret != 0) {
            return ret;
        }

        if (opt_uart.baudrates.empty()) {
            opt_uart.baudrates.push_back(DEFAULT_BAUDRATE);
        }

        if (!UartEndpoint::validate_config(opt_uart)) {
            return -EINVAL; // error message was already logged by validate_config()
        }
        config.uart_configs.push_back(opt_uart);
    }

    iter = {};
    offset = strlen(UdpEndpoint::section_pattern) - 1;
    while (conffile.get_sections(UdpEndpoint::section_pattern, &iter) == 0) {
        UdpEndpointConfig opt_udp{};
        opt_udp.name = std::string(iter.name + offset, iter.name_len - offset);
        opt_udp.port = ULONG_MAX; // unset port value to be checked later on

        ret = conffile.extract_options(&iter, UdpEndpoint::option_table, &opt_udp);
        if (ret != 0) {
            return ret;
        }

        if (UdpEndpointConfig::Mode::Client == opt_udp.mode && ULONG_MAX == opt_udp.port) {
            opt_udp.port = find_next_udp_port(opt_udp.address, config);
        }

        if (!UdpEndpoint::validate_config(opt_udp)) {
            return -EINVAL; // error message was already logged by validate_config()
        }
        config.udp_configs.push_back(opt_udp);
    }

    iter = {};
    offset = strlen(TcpEndpoint::section_pattern) - 1;
    while (conffile.get_sections(TcpEndpoint::section_pattern, &iter) == 0) {
        TcpEndpointConfig opt_tcp{};
        opt_tcp.name = std::string(iter.name + offset, iter.name_len - offset);
        opt_tcp.port = ULONG_MAX; // unset port value to be checked later on
        ret = conffile.extract_options(&iter, TcpEndpoint::option_table, &opt_tcp);
        if (ret != 0) {
            return ret;
        }

        if (!TcpEndpoint::validate_config(opt_tcp)) {
            return -EINVAL; // error message was already logged by validate_config()
        }
        config.tcp_configs.push_back(opt_tcp);
    }

    return 0;
}

static int cmpstr(const void *s1, const void *s2)
{
    return strcmp(*(const char **)s1, *(const char **)s2);
}

static int parse_conf_files(Configuration &config)
{
    DIR *dir;
    struct dirent *ent;
    int ret = 0;
    char *files[128] = {};
    int i = 0, j = 0;

    ConfFile conffile;

    // First, open default conf file
    auto filename = get_conf_file_name(config);
    ret = conffile.parse(filename);

    // If there's no default conf file, everything is good
    if (ret < 0 && ret != -ENOENT) {
        return ret;
    }

    // Then, parse all files on configuration directory
    auto dirname = get_conf_dir(config);
    dir = opendir(dirname.c_str());
    if (dir != nullptr) {
        while ((ent = readdir(dir)) != nullptr) {
            char path[PATH_MAX];
            struct stat st;

            ret = snprintf(path, sizeof(path), "%s/%s", dirname.c_str(), ent->d_name);
            if (ret >= (int)sizeof(path)) {
                log_error("Couldn't open directory %s", dirname.c_str());
                ret = -EINVAL;
                goto fail;
            }
            if (stat(path, &st) < 0 || !S_ISREG(st.st_mode)) {
                continue;
            }
            files[i] = strdup(path);
            if (files[i] == nullptr) {
                ret = -ENOMEM;
                goto fail;
            }
            i++;

            if ((size_t)i > sizeof(files) / sizeof(*files)) {
                log_warning("Too many files on %s. Not all of them will be considered",
                            dirname.c_str());
                break;
            }
        }

        qsort(files, (size_t)i, sizeof(char *), cmpstr);

        for (j = 0; j < i; j++) {
            ret = conffile.parse(files[j]);
            if (ret < 0) {
                goto fail;
            }
            free(files[j]);
        }

        closedir(dir);
    }

    // Finally get configuration values from all config files
    return parse_confs(conffile, config);

fail:
    while (j < i) {
        free(files[j++]);
    }

    closedir(dir);

    return ret;
}

int main(int argc, char *argv[])
{
    Mainloop &mainloop = Mainloop::init();
    int retcode;
    Configuration config{};

    Log::open();
    log_info(PACKAGE " version %s", BUILD_VERSION);

    if (!pre_parse_argv(argc, argv, config)) {
        Log::close();
        return 0;
    }

    // Build remaining config from config files and CLI parameters
    if (parse_conf_files(config) < 0) {
        goto close_log;
    }

    if (parse_argv(argc, argv, config) != 2) {
        goto close_log;
    }
    dbg("Cmd line and options parsed");

    Log::set_max_level(config.debug_log_level);

    // Create endpoint instances and run
    if (mainloop.open() < 0) {
        goto close_log;
    }

    if (!mainloop.add_endpoints(config)) {
        goto close_log;
    }

    retcode = mainloop.loop();

    Log::close();
    return retcode ? EXIT_FAILURE : 0;

close_log:
    Log::close();
    return EXIT_FAILURE;
}
