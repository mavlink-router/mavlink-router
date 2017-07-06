/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
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

/*
 * This example shows how to change to offboard mode in PX4.
 *
 * PX4 will only change to offboard mode and keep it in this mode if it is
 * receiving one of this messages: mavlink_set_position_target_local_ned_t,
 * mavlink_set_actuator_control_target_t or mavlink_set_attitude_target_t
 * at least at 0.5 hz.
 *
 * At each 50msec this example will request to the vehicle to move
 * 0 meters in X, 0 meters in Y, 0 meters in Z and 0 rads in yaw
 * (position_set_send(0, 0, 0, 0)) from the actual position of the vehicle and
 * if the actual mode is not offboard it will request PX4 to change to this
 * mode.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/timerfd.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <mavlink.h>

#include "common/util.h"

#define SYSTEM_ID 11
#define TARGET_SYSTEM_ID 1

#define PX4_MAIN_MODE_CUSTOM 1
#define PX4_CUSTOM_MODE_OFFBOARD 6

#define POLL_ERROR_EVENTS (POLLERR | POLLHUP | POLLNVAL)

static volatile bool g_should_exit;

static int tcp_fd = -1;
static struct sockaddr_in sockaddr;

static int timeout_fd = -1;

enum px4_modes {
    PX4_MODE_UNKNOWN = 0,
    PX4_MODE_STABILIZED,
    PX4_MODE_OFFBOARD,
    PX4_MODE_LAST
};

static const char * mode_names[] = {
    "unknown",
    "stabilized",
    "offboard"
};

static enum px4_modes target_mode = PX4_MODE_UNKNOWN;
static enum px4_modes target_mode_requested = PX4_MODE_UNKNOWN;

static void exit_signal_handler(int signum)
{
    g_should_exit = true;
}

static int setup_signal_handlers()
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;

    if (sigaction(SIGTERM, &sa, NULL) != 0 ||
        sigaction(SIGINT, &sa, NULL) != 0)
        return -1;

    return 0;
}

static int msg_send(mavlink_message_t *msg)
{
    uint8_t data[MAVLINK_MAX_PACKET_LEN];

    uint16_t len = mavlink_msg_to_send_buffer(data, msg);
    int r = sendto(tcp_fd, data, len, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (r < 0) {
        fprintf(stderr, "Could not send to %d: r=%d (%m)\n", tcp_fd, r);
    }

    return r;
}

static void handle_command_ack(mavlink_command_ack_t *ack)
{
    switch (ack->command) {
    case MAV_CMD_DO_SET_MODE:
        if (ack->result == MAV_RESULT_ACCEPTED) {
            printf("Mode %s set\n", mode_names[target_mode_requested]);
            target_mode = target_mode_requested;
        } else {
            printf("Error setting vehicle to mode %s, actual mode is %s\n", mode_names[target_mode_requested], mode_names[target_mode]);
        }
        break;
    default:
        break;
    }
}

static void handle_new_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_ACK:
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);
        handle_command_ack(&ack);
        break;
    default:
        break;
    }
}

static void position_set_send(float x, float y, float z, float yaw)
{
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t set_position = {};
    const uint64_t now_msec = now_usec() / NSEC_PER_MSEC;
    static uint64_t initial_msec = 0;

    if (!initial_msec) {
        set_position.time_boot_ms = 0;
        initial_msec = now_msec;
    } else {
        set_position.time_boot_ms = now_msec - initial_msec;
    }

    set_position.target_system = TARGET_SYSTEM_ID;
    set_position.target_component = MAV_COMP_ID_ALL;
    set_position.coordinate_frame = MAV_FRAME_LOCAL_OFFSET_NED;
    /* only make X, Y, Z and yaw valid */
    set_position.type_mask = ~(1 << 0 | 1 << 1 | 1 << 2 | 1 << 10);
    set_position.x = x;
    set_position.y = y;
    set_position.z = z;
    set_position.yaw = yaw;

    mavlink_msg_set_position_target_local_ned_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, &set_position);
    msg_send(&msg);
}

static void set_mode_send(enum px4_modes mode)
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd = {};

    cmd.command = MAV_CMD_DO_SET_MODE;
    cmd.target_system = TARGET_SYSTEM_ID;
    cmd.target_component = MAV_COMP_ID_ALL;

    switch (mode) {
    case PX4_MODE_OFFBOARD:
        cmd.param1 = PX4_MAIN_MODE_CUSTOM;
        cmd.param2 = PX4_CUSTOM_MODE_OFFBOARD;
        target_mode_requested = mode;
        break;
    case PX4_MODE_STABILIZED:
    default:
        cmd.param1 = MAV_MODE_STABILIZE_ARMED;
        target_mode_requested = PX4_MODE_STABILIZED;
        break;
    }

    mavlink_msg_command_long_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, &cmd);
    msg_send(&msg);
}

static void timeout_callback()
{
    position_set_send(0, 0, 0, 0);

    if (target_mode != PX4_MODE_OFFBOARD) {
        printf("Requesting mode %s...\n", mode_names[PX4_MODE_OFFBOARD]);
        set_mode_send(PX4_MODE_OFFBOARD);
    }
}

static int tcp_fd_poll_handle(const struct pollfd *d)
{
    if (d->revents & POLLIN) {
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        socklen_t addrlen = sizeof(sockaddr);
        ssize_t n = recvfrom(tcp_fd, buf, sizeof(buf), 0, (struct sockaddr *)&sockaddr, &addrlen);

        if (n == -1) {
            if (errno == EINTR) {
                return 0;
            }

            fprintf(stderr, "Error reading socket (%m)\n");
            return -1;
        }

        for (int i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                handle_new_message(&msg);
            }
        }
    }

    return 0;
}

static int timeout_fd_poll_handle(const struct pollfd *d)
{
    if (d->revents & POLLIN) {
        uint64_t val = 0;
        int r = read(d->fd, &val, sizeof(val));

        if (r < 1 || val == 0) {
            return 0;
        }

        timeout_callback();
    }

    return 0;
}

static void loop()
{
    struct pollfd desc[2];

    desc[0].fd = tcp_fd;
    desc[0].events = POLLIN | POLL_ERROR_EVENTS;
    desc[0].revents = 0;

    desc[1].fd = timeout_fd;
    desc[1].events = POLLIN | POLL_ERROR_EVENTS;
    desc[1].revents = 0;

    while (!g_should_exit) {
        int ret = poll(desc, sizeof(desc) / sizeof(struct pollfd), -1);

        if (ret < 1) {
            fprintf(stderr, "poll() returned a error (%m)\n");
            break;
        }

        for (unsigned i = 0; ret && i < (sizeof(desc) / sizeof(struct pollfd)); i++, ret--) {
            const struct pollfd *d = &desc[i];

            if (d->fd == tcp_fd) {
                tcp_fd_poll_handle(d);
            } else if (d->fd == timeout_fd) {
                timeout_fd_poll_handle(d);
            }
        }
    }
}

static int setup_connection()
{
    const char *ip = "127.0.0.1";
    const int port = 5760;

    tcp_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_fd == -1) {
        fprintf(stderr, "Could not create socket (%m)\n");
        return -1;
    }

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(ip);
    sockaddr.sin_port = htons(port);

    if (connect(tcp_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
        fprintf(stderr, "Could not connect to socket (%m)\n");
        return -1;
    }

    if (fcntl(tcp_fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0) {
        fprintf(stderr, "Error setting socket tcp_fd as non-blocking");
        return -1;
    }

    printf("Connected to TCP:%s:%u\n", ip, port);

    return 0;
}

static int setup_timeout()
{
    struct itimerspec ts = {};

    timeout_fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (timeout_fd < 0) {
        fprintf(stderr, "Unable to create timerfd (%m)\n");
        return -1;
    }

    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = NSEC_PER_MSEC * 50;
    ts.it_value.tv_sec = ts.it_interval.tv_sec;
    ts.it_value.tv_nsec = ts.it_interval.tv_nsec;

    timerfd_settime(timeout_fd, 0, &ts, NULL);

    return 0;
}

int main(int argc, char *argv[])
{
    if (setup_connection() < 0 ||
        setup_timeout() < 0 ||
        setup_signal_handlers() < 0)
        goto fail;

    loop();

    close(timeout_fd);
    close(tcp_fd);

    return 0;

fail:
    if (tcp_fd >= 0)
        close(tcp_fd);
    if (timeout_fd >=0)
        close(timeout_fd);

    return EXIT_FAILURE;
}
