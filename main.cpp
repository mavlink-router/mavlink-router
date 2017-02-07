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
#include <stdio.h>
#include <sys/stat.h>

#include "comm.h"
#include "endpoint.h"
#include "log.h"
#include "mainloop.h"
#include "util.h"

#define MAVLINK_TCP_PORT 5760

static struct opt opt = {
        .baudrate = 115200U,
        .ep_addrs = nullptr,
        .master_addrs = nullptr,
        .tcp_port = MAVLINK_TCP_PORT,
        .report_msg_statistics = false,
};

static void help(FILE *fp) {
    fprintf(fp,
            "%s [OPTIONS...] [<uart>|<udp_address>]\n\n"
            "  <uart>                       UART device that will be routed\n"
            "  <udp_address>                UDP address (<ip>:<port>) that will be routed\n"
            "  -b --baudrate <baudrate>     Use baudrate for UART\n"
            "  -e --endpoint <ip[:port]>    Add UDP endpoint to communicate port is optional\n"
            "                               and in case it's not given it starts in 14550 and\n"
            "                               continues increasing not to collide with previous\n"
            "                               ports\n"
            "  -r --report_msg_statistics   Report message statistics\n"
            "  -t --tcp-port                Port in which mavlink-router will listen for TCP\n"
            "                               connections. Pass 0 to disable TCP listening.\n"
            "                               Default port 5760\n"
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

static int parse_argv(int argc, char *argv[], const char **uart, char **udp_addr, unsigned long *udp_port)
{
    static const struct option options[] = {
        { "baudrate",               required_argument,  NULL,   'b' },
        { "endpoints",              required_argument,  NULL,   'e' },
        { "report_msg_statistics",  no_argument,        NULL,   'r' },
        { "tcp-port",               required_argument,  NULL,   't' },
        { }
    };
    int c;
    struct stat st;

    assert(argc >= 0);
    assert(argv);
    assert(uart);

    *uart = NULL;

    while ((c = getopt_long(argc, argv, "hb:e:rt:", options, NULL)) >= 0) {
        switch (c) {
        case 'h':
            help(stdout);
            return 0;
        case 'b':
            if (safe_atoul(optarg, &opt.baudrate) < 0) {
                log_error("Invalid argument for baudrate = %s", optarg);
                help(stderr);
                return -EINVAL;
            }
            break;
        case 'e': {
            char *ip = strdup(optarg);

            char *portstr = strchrnul(ip, ':');
            unsigned long port;
            if (*portstr == '\0') {
                port = find_next_endpoint_port(ip);
            } else {
                *portstr = '\0';
                if (safe_atoul(portstr + 1, &port) < 0) {
                    log_error("Invalid port in argument: %s", optarg);
                    free(ip);
                    help(stderr);
                    return -EINVAL;
                }
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
        case '?':
        default:
            help(stderr);
            return -EINVAL;
        }
    }

    /* positional arguments */
    if (optind + 1 > argc) {
        log_error("Error parsing required argument %d %d", optind, argc);
        help(stderr);
        return -EINVAL;
    }

    while (optind < argc) {
        if (stat(argv[optind], &st) == -1 || !S_ISCHR(st.st_mode)) {
            unsigned long port;
            char *ip = strdup(argv[optind]);

            char *portstr = strchrnul(ip, ':');
            if (*portstr == '\0') {
                log_error("Invalid argument for UDP address = %s. Please inform <address>:<port>",
                          argv[optind]);
                help(stderr);
                free(ip);
                return -EINVAL;
            } else {
                *portstr = '\0';
                if (safe_atoul(portstr + 1, &port) < 0) {
                    log_error("Invalid argument for UDP port = %s", argv[optind]);
                    help(stderr);
                    free(ip);
                    return -EINVAL;
                }
            }

            add_endpoint_address(&opt.master_addrs, ip, port);
        } else {
            if (!*uart)
                *uart = argv[optind];
            else {
                log_error("Cannot route from more than one UART!");
                return -EINVAL;
            }
        }
        optind++;
    }

    return 2;
}

int main(int argc, char *argv[])
{
    unsigned long udp_port = 0;
    const char *uartstr = NULL;
    char *udp_addr = NULL;
    Mainloop mainloop{};

    log_open();

    if (parse_argv(argc, argv, &uartstr, &udp_addr, &udp_port) != 2)
        goto close_log;

    if (mainloop.open() < 0)
        goto close_log;

    if (!mainloop.add_endpoints(mainloop, uartstr, &opt))
        goto close_log;

    mainloop.loop();

    mainloop.free_endpoints(&opt);

    free(udp_addr);

    log_close();

    return 0;

close_log:
    free(udp_addr);
    log_close();
    return EXIT_FAILURE;
}
