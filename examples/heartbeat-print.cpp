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

#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>

#include <common/mavlink.h>

static volatile bool g_should_exit;

static void exit_signal_handler(int signum)
{
    g_should_exit = true;
}

static void setup_signal_handlers()
{
    struct sigaction sa = {};

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGINT, &sa, nullptr);
}

static void handle_new_message(const mavlink_message_t *msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t heartbeat{};
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        printf("HEARTBEAT:\n"
               "\tmavlink_version: %u\n"
               "\ttype: %u\n",
               heartbeat.mavlink_version,
               heartbeat.type);
    }
}

int main(int argc, char *argv[])
{
    struct sockaddr_in sockaddr;
    const socklen_t addrlen = sizeof(sockaddr);
    int port = 14550;
    int fd;

    if (argc < 2) {
        printf("Usage: heartbeat-print <ip>:<port>\n");
        return -1;
    }

    setup_signal_handlers();

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == -1) {
        fprintf(stderr, "Could not create socket (%m)\n");
        return 1;
    }

    char *ip = strdup(argv[1]);
    char *portstr = strchr(ip, ':');
    if (portstr && *portstr) {
        *portstr = '\0';
        port = atoi(portstr + 1);
    }
    printf("Connecting to: %s:%i\n", ip, port);

    bzero(&sockaddr, addrlen);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(ip);
    sockaddr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr *)&sockaddr, addrlen) == -1) {
        fprintf(stderr, "Could not bind to %s:%d (%m)", ip, port);
        free(ip);
        return 1;
    }

    mavlink_message_t msg{};
    mavlink_status_t status{};

    while (!g_should_exit) {
        uint8_t buf[1024];
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n == -1) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }

        for (int i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                handle_new_message(&msg);
            }
        }
    }

    free(ip);
    close(fd);

    return 0;
}
