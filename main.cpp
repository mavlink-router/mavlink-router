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
#include <getopt.h>
#include <limits.h>
#include <stdio.h>
#include <sys/stat.h>

#include "comm.h"
#include "conf.h"
#include "endpoint.h"
#include "log.h"
#include "mainloop.h"
#include "util.h"

#define MAVLINK_TCP_PORT 5760
#define DEFAULT_BAUDRATE 115200U
#define DEFAULT_CONFFILE "/etc/mavlink-router/main.conf"

static struct opt opt = {
        .ep_addrs = nullptr,
        .master_addrs = nullptr,
        .uart_devices = nullptr,
        .conf_file_name = nullptr,
        .tcp_port = ULONG_MAX,
        .report_msg_statistics = false
};

static void help(FILE *fp) {
    fprintf(fp,
            "%s [OPTIONS...] [<uart>|<udp_address>]\n\n"
            "  <uart>                       UART device (<device>[:<baudrate>]) that will be routed\n"
            "  <udp_address>                UDP address (<ip>:<port>) that will be routed\n"
            "  -e --endpoint <ip[:port]>    Add UDP endpoint to communicate port is optional\n"
            "                               and in case it's not given it starts in 14550 and\n"
            "                               continues increasing not to collide with previous\n"
            "                               ports\n"
            "  -r --report_msg_statistics   Report message statistics\n"
            "  -t --tcp-port                Port in which mavlink-router will listen for TCP\n"
            "                               connections. Pass 0 to disable TCP listening.\n"
            "                               Default port 5760\n"
            "  -c --conf-file               .conf file with configurations for mavlink-router.\n"
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

static int split_on_colon(const char *str, char **base, unsigned long *number)
{
    char *colonstr;

    *base = strdup(str);
    colonstr = strchrnul(*base, ':');
    *number = ULONG_MAX;

    if (*colonstr != '\0') {
        *colonstr = '\0';
        if (safe_atoul(colonstr + 1, number) < 0) {
            free(*base);
            return -EINVAL;
        }
    }

    return 0;
}

static void add_endpoint_address(struct endpoint_address **list, char *ip, long int port)
{
    struct endpoint_address *e = (struct endpoint_address *)malloc(sizeof(*e));

    if (!e) {
        log_error_errno(errno, "Could not parse endpoint address: %m");
        return;
    }

    e->next = *list;
    e->ip = ip;
    e->port = port;
    *list = e;
}

static int add_uart_endpoint(struct uart_endpoint_device **list, char *uart_device,
                             unsigned long baudrate)
{
    struct uart_endpoint_device *d = (struct uart_endpoint_device *)malloc(sizeof(*d));

    if (!d) {
        log_error_errno(errno, "Could not parse endpoint device: %m");
        return -ENOMEM;
    }

    d->device = uart_device;
    d->baudrate = baudrate;
    d->next = *list;
    *list = d;

    return 0;
}

static int parse_argv(int argc, char *argv[])
{
    static const struct option options[] = {
        { "baudrate",               required_argument,  NULL,   'b' },
        { "endpoints",              required_argument,  NULL,   'e' },
        { "conf-file",               required_argument,  NULL,   'i' },
        { "report_msg_statistics",  no_argument,        NULL,   'r' },
        { "tcp-port",               required_argument,  NULL,   't' },
        { }
    };
    int c;
    struct stat st;

    assert(argc >= 0);
    assert(argv);

    while ((c = getopt_long(argc, argv, "he:rt:c:", options, NULL)) >= 0) {
        switch (c) {
        case 'h':
            help(stdout);
            return 0;
        case 'e': {
            char *ip;
            unsigned long port;

            if (split_on_colon(optarg, &ip, &port) < 0) {
                log_error("Invalid port in argument: %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            if (port == ULONG_MAX) {
                port = find_next_endpoint_port(ip);
            }

            add_endpoint_address(&opt.ep_addrs, ip, port);
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
        case 'c': {
            opt.conf_file_name = optarg;
            break;
        }
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

        if (split_on_colon(argv[optind], &base, &number) < 0) {
            log_error("Invalid argument %s", argv[optind]);
            help(stderr);
            return -EINVAL;
        }

        if (stat(base, &st) == -1 || !S_ISCHR(st.st_mode)) {
            if (number == ULONG_MAX) {
                log_error("Invalid argument for UDP port = %s", argv[optind]);
                help(stderr);
                free(base);
                return -EINVAL;
            }

            add_endpoint_address(&opt.master_addrs, base, number);
        } else {
            int ret = add_uart_endpoint(&opt.uart_devices, base, number);
            if (ret < 0)
                return ret;
        }
        optind++;
    }

    return 2;
}

static const char *get_conf_file_name()
{
    char *s;

    if (opt.conf_file_name)
        return opt.conf_file_name;

    s = getenv("MAVLINK_ROUTERD_CONF_FILE");
    if (s)
        return s;

    return DEFAULT_CONFFILE;
}

static int parse_conf()
{
    const char *conf_file_name = get_conf_file_name();
    const char *value, *section;
    int ret;

    ConfFile conf{conf_file_name};

    ret = conf.parse_file();

    if (ret < 0) {
        // If there's no default conf file, everything is good
        if (ret == -EIO && !strcmp(conf_file_name, DEFAULT_CONFFILE))
            return 0;
        return ret;
    }

    // This conflicting option is resolved by using command line one
    value = conf.next_from_section("General", "tcp");
    if (value && opt.tcp_port == ULONG_MAX && (safe_atoul(value, &opt.tcp_port) < 0)) {
        log_error("On file %s: invalid argument for tcp-port = '%s'", conf_file_name, value);
        return -EINVAL;
    }

    value = conf.next_from_section("General", "report-stats");
    if (value) {
        if (!strcasecmp(value, "true") || !strcmp(value, "1"))
            opt.report_msg_statistics = true;
    }

    section = conf.next_section(true);
    while (section) {
        const char *type;

        if (strncasecmp(section, "endpoint ", strlen("endpoint "))) {
            section = conf.next_section();
            continue;
        }

        type = conf.next_from_section(section, "type");
        if (!type) {
            log_error("On file %s: expected 'type' key for %s", conf_file_name, section);
            return -EINVAL;
        }

        if (!strcasecmp(type, "uart")) {
            // ********************* UART type ***********************
            const char *baudstr, *device;
            char *d;
            unsigned long baud;

            device = conf.next_from_section(section, "device");
            if (!device) {
                log_error("On file %s: expected 'device' key for %s", conf_file_name, section);
                return -EINVAL;
            }

            baudstr = conf.next_from_section(section, "baud");
            if (!baudstr) {
                baud = DEFAULT_BAUDRATE;
            } else if (safe_atoul(baudstr, &baud)) {
                log_error("On file %s: invalid baudrate for %s", conf_file_name, section);
                return -EINVAL;
            }

            // As mainloop frees this char*, we need to use something that can be freed
            d = strdup(device);
            if (!d) {
                log_error_errno(errno, "On file %s: could not parse %s (%m)", conf_file_name,
                                section);
                return -ENOMEM;
            }
            ret = add_uart_endpoint(&opt.uart_devices, d, baud);
            if (ret < 0) {
                return ret;
            }
        } else if (!strcasecmp(type, "udp")) {
            // ********************* UDP type ***********************
            const char *addr, *portstr, *mode;
            unsigned long port = ULONG_MAX;
            struct endpoint_address **list;
            char *c;

            addr = conf.next_from_section(section, "address");
            if (!addr) {
                log_error("On file %s: expected 'address' key for %s", conf_file_name, section);
                return -EINVAL;
            }

            mode = conf.next_from_section(section, "mode");
            if (!mode) {
                log_error("On file %s: expected 'mode' key for %s", conf_file_name, section);
                return -EINVAL;
            }

            portstr = conf.next_from_section(section, "port");
            if (portstr && safe_atoul(portstr, &port) < 0) {
                log_error("On file %s: invalid port for %s", conf_file_name, section);
                return -EINVAL;
            }

            if (!strcasecmp(mode, "normal")) {
                list = &opt.ep_addrs;
            } else if (!strcasecmp(mode, "eavesdropping")) {
                list = &opt.master_addrs;
            } else {
                log_error("On file %s: unknown 'mode' key for %s", conf_file_name, section);
                return -EINVAL;
            }

            // Eavesdroppoing (or master) udp endpoints need an explicit port
            if (port == ULONG_MAX) {
                if (list == &opt.master_addrs) {
                    log_error("On file %s: expected 'port' key for %s", conf_file_name, section);
                    return -EINVAL;
                }
                port = find_next_endpoint_port(addr);
            }

            // As mainloop frees this char*, we need to use something that can be freed
            c = strdup(addr);
            if (!c) {
                log_error_errno(errno, "On file %s: could not parse %s (%m)", conf_file_name,
                                section);
                return -ENOMEM;
            }
            add_endpoint_address(list, c, port);
        } else {
            log_info("On file %s: Unknown type for %s", conf_file_name, section);
        }

        section = conf.next_section();
    }

    return 0;
}

int main(int argc, char *argv[])
{
    Mainloop mainloop{};

    log_open();

    if (parse_argv(argc, argv) != 2)
        goto close_log;

    if (parse_conf() < 0)
        goto close_log;

    if (mainloop.open() < 0)
        goto close_log;

    if (opt.tcp_port == ULONG_MAX)
        opt.tcp_port = MAVLINK_TCP_PORT;

    if (!mainloop.add_endpoints(mainloop, &opt))
        goto close_log;

    mainloop.loop();

    mainloop.free_endpoints(&opt);

    log_close();

    return 0;

close_log:
    log_close();
    return EXIT_FAILURE;
}
