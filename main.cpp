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
#define DEFAULT_CONF_DIR "/etc/mavlink-router/config.d"

static struct opt opt = {
        .endpoints = nullptr,
        .conf_file_name = nullptr,
        .conf_dir = nullptr,
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
            "  -d --conf-dir                Directory were to look for .conf files overriding\n"
            "                               default conf file.\n"
            "  -h --help                    Print this message\n"
            , program_invocation_short_name);
}

static unsigned long find_next_endpoint_port(const char *ip)
{
    unsigned long port = 14550U;

    while (true) {
        struct endpoint_config *conf;

        for (conf = opt.endpoints; conf; conf = conf->next) {
            if (conf->type == Udp && streq(conf->address, ip) && conf->port == port) {
                port++;
                break;
            }
        }
        if (!conf)
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

static int add_endpoint_address(struct endpoint_config *conf, const char *name, const char *ip,
                                long unsigned port, bool eavesdropping)
{
    bool new_conf = false;

    if (!conf) {
        conf = (struct endpoint_config *)calloc(1, sizeof(struct endpoint_config));
        assert_or_return(conf, -ENOMEM);
        conf->type = Udp;
        conf->port = ULONG_MAX;
        new_conf = true;
    }

    if (!conf->name && name) {
        conf->name = strdup(name);
        assert_or_return(conf->name, -ENOMEM);
    }

    if (ip) {
        free(conf->address);
        conf->address = strdup(ip);
        assert_or_return(conf->address, -ENOMEM);
    }

    if (!conf->address) {
        return -EINVAL;
    }

    if (port != ULONG_MAX) {
        conf->port = port;
    }

    if (conf->port == ULONG_MAX) {
        conf->port = find_next_endpoint_port(conf->address);
    }

    conf->eavesdropping = eavesdropping;

    if (new_conf) {
        conf->next = opt.endpoints;
        opt.endpoints = conf;
    }

    return 0;
}

static int add_uart_endpoint(struct endpoint_config *conf, const char *name,
                             const char *uart_device, unsigned long baudrate)
{
    bool new_conf = false;

    if (!conf) {
        conf = (struct endpoint_config *)calloc(1, sizeof(struct endpoint_config));
        assert_or_return(conf, -ENOMEM);
        conf->type = Uart;
        conf->baud = ULONG_MAX;
        new_conf = true;
    }

    // As name doesn't change, only update if there's no name yet
    if (!conf->name && name) {
        conf->name = strdup(name);
        assert_or_return(conf->name, -ENOMEM);
    }

    if (uart_device) {
        free(conf->device);
        conf->device = strdup(uart_device);
        assert_or_return(conf->device, -ENOMEM);
    }

    if (!conf->device) {
        return -EINVAL;
    }

    if (baudrate != ULONG_MAX) {
        conf->baud = baudrate;
    }

    if (conf->baud == ULONG_MAX) {
        conf->baud = DEFAULT_BAUDRATE;
    }

    if (new_conf) {
        conf->next = opt.endpoints;
        opt.endpoints = conf;
    }

    return 0;
}

static int parse_argv(int argc, char *argv[])
{
    static const struct option options[] = {
        { "baudrate",               required_argument,  NULL,   'b' },
        { "endpoints",              required_argument,  NULL,   'e' },
        { "conf-file",              required_argument,  NULL,   'i' },
        { "dir-file" ,              required_argument,  NULL,   'd' },
        { "report_msg_statistics",  no_argument,        NULL,   'r' },
        { "tcp-port",               required_argument,  NULL,   't' },
        { }
    };
    int c;
    struct stat st;

    assert(argc >= 0);
    assert(argv);

    while ((c = getopt_long(argc, argv, "he:rt:c:d:", options, NULL)) >= 0) {
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

            add_endpoint_address(NULL, NULL, ip, port, false);
            free(ip);
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
        case 'd': {
            opt.conf_dir = optarg;
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

            add_endpoint_address(NULL, NULL, base, number, true);
        } else {
            int ret = add_uart_endpoint(NULL, NULL, base, number);
            if (ret < 0)
                return ret;
        }
        free(base);
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

static const char *get_conf_dir()
{
    char *s;

    if (opt.conf_dir)
        return opt.conf_dir;

    s = getenv("MAVLINK_ROUTERD_CONF_DIR");
    if (s)
        return s;

    return DEFAULT_CONF_DIR;
}

static struct endpoint_config *search_endpoints(const char *name)
{
    for (struct endpoint_config *conf = opt.endpoints; conf; conf = conf->next) {
        if (conf->name && strcaseeq(conf->name, name)) {
            return conf;
        }
    }

    return nullptr;
}

static int parse_conf(const char *conf_file_name)
{
    const char *value, *section;
    int ret;

    ConfFile conf{conf_file_name};

    ret = conf.parse_file();

    if (ret < 0) {
        return ret;
    }

    value = conf.next_from_section("General", "tcp");
    if (value && (safe_atoul(value, &opt.tcp_port) < 0)) {
        log_error("On file %s: invalid argument for tcp-port = '%s'", conf_file_name, value);
        return -EINVAL;
    }

    value = conf.next_from_section("General", "report-stats");
    if (value) {
        if (strcaseeq(value, "true") || !strcmp(value, "1")) {
            opt.report_msg_statistics = true;
        } else {
            opt.report_msg_statistics = false;
        }
    }

    section = conf.first_section();
    while (section) {
        const char *typestr;
        struct endpoint_config *endpoint = nullptr;
        enum endpoint_type type = Unknown;

        if (strncasecmp(section, "endpoint ", strlen("endpoint "))) {
            section = conf.next_section();
            continue;
        }

        // If we've seen this endpoint, keep it here
        endpoint = search_endpoints(section);

        typestr = conf.next_from_section(section, "type");
        if (!typestr && !endpoint) {
            log_error("On file %s: expected 'type' key for %s", conf_file_name, section);
            return -EINVAL;
        }

        if (!typestr) {
            type = endpoint->type;
        } else {
            if (strcaseeq(typestr, "uart")) {
                type = Uart;
            } else if (strcaseeq(typestr, "udp")) {
                type = Udp;
            }

            if (endpoint && endpoint->type != type) {
                log_error("On file %s: redefining type for %s", conf_file_name, section);
                return -EINVAL;
            }
        }

        switch (type) {
        case Uart: {
            const char *baudstr, *device;
            unsigned long baud = ULONG_MAX;

            device = conf.next_from_section(section, "device");
            if (!device && !endpoint) {
                log_error("On file %s: expected 'device' key for %s", conf_file_name, section);
                return -EINVAL;
            }

            baudstr = conf.next_from_section(section, "baud");
            if (baudstr && safe_atoul(baudstr, &baud) < 0) {
                log_error("On file %s: invalid baudrate for %s", conf_file_name, section);
                return -EINVAL;
            }

            ret = add_uart_endpoint(endpoint, section, device, baud);
            if (ret < 0) {
                return ret;
            }
            break;
        }
        case Udp: {
            const char *addr, *portstr, *mode;
            unsigned long port = ULONG_MAX;
            bool eavesdropping;

            addr = conf.next_from_section(section, "address");
            if (!addr && !endpoint) {
                log_error("On file %s: expected key 'address' for %s", conf_file_name, section);
                return -EINVAL;
            }

            mode = conf.next_from_section(section, "mode");
            if (!mode && !endpoint) {
                log_error("On file %s: expected key 'mode' for %s", conf_file_name, section);
                return -EINVAL;
            }

            portstr = conf.next_from_section(section, "port");
            if (portstr && safe_atoul(portstr, &port) < 0) {
                log_error("On file %s: invalid port for %s", conf_file_name, section);
                return -EINVAL;
            }

            if (mode) {
                if (strcaseeq(mode, "normal")) {
                    eavesdropping = false;
                } else if (strcaseeq(mode, "eavesdropping")) {
                    eavesdropping = true;
                } else {
                    log_error("On file %s: unknown 'mode' key for %s", conf_file_name, section);
                    return -EINVAL;
                }
            } else {
                eavesdropping = endpoint->eavesdropping;
            }

            // Eavesdroppoing (or master) udp endpoints need an explicit port
            if (port == ULONG_MAX && !endpoint) {
                if (eavesdropping) {
                    log_error("On file %s: expected 'port' key for %s", conf_file_name, section);
                    return -EINVAL;
                }
                port = find_next_endpoint_port(addr);
            }

            ret = add_endpoint_address(endpoint, section, addr, port, eavesdropping);
            if (ret < 0) {
                return ret;
            }
            break;
        }
        default:
            log_info("On file %s: Unknown type for %s", conf_file_name, section);
            return -EINVAL;
        }

        section = conf.next_section();
    }

    return 0;
}

static int cmpstr(const void *s1, const void *s2)
{
    return strcmp(*(const char **)s1, *(const char **)s2);
}

static int parse_conf_files()
{
    DIR *dir;
    struct dirent *ent;
    const char *filename, *dirname;
    int ret = 0;
    char *files[128] = {};
    int i = 0, j = 0;

    // First, open default conf file
    filename = get_conf_file_name();
    ret = parse_conf(filename);

    // If there's no default conf file, everything is good
    if (ret < 0 && ret != -EIO) {
        return ret;
    }

    dirname = get_conf_dir();
    // Then, parse all files on configuration directory
    dir = opendir(dirname);
    if (!dir) {
        return 0;
    }

    while ((ent = readdir(dir))) {
        char path[PATH_MAX];
        struct stat st;

        ret = snprintf(path, sizeof(path), "%s/%s", dirname, ent->d_name);
        if (ret >= (int)sizeof(path)) {
            log_error("Couldn't open directory %s", dirname);
            ret = -EINVAL;
            goto fail;
        }
        if (stat(path, &st) < 0 || !S_ISREG(st.st_mode)) {
            continue;
        }
        files[i] = strdup(path);
        if (!files[i]) {
            ret = -ENOMEM;
            goto fail;
        }
        i++;

        if ((size_t)i > sizeof(files) / sizeof(*files)) {
            log_warning("Too many files on %s. Not all of them will be considered", dirname);
            break;
        }
    }

    qsort(files, (size_t)i, sizeof(char *), cmpstr);

    for (j = 0; j < i; j++) {
        ret = parse_conf(files[j]);
        if (ret < 0) {
            goto fail;
        }
        free(files[j]);
    }

    closedir(dir);

    return 0;

fail:
    while (j < i) {
        free(files[j++]);
    }

    closedir(dir);

    return ret;
}

int main(int argc, char *argv[])
{
    Mainloop mainloop{};

    log_open();

    if (parse_argv(argc, argv) != 2)
        goto close_log;

    if (parse_conf_files() < 0)
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
