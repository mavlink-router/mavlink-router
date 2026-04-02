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
 * This example shows how to fly in offboard mode in PX4.
 *
 * PX4 will only change to offboard mode and keep it in this mode if it is
 * receiving one of this messages: mavlink_set_position_target_local_ned_t,
 * mavlink_set_actuator_control_target_t or mavlink_set_attitude_target_t
 * at least at 0.5 hz.
 *
 * At each 50msec this example will send to PX4 a
 * mavlink_set_position_target_local_ned_t, also in the same callback it will
 * handle each item of the task list, changing the parameters of
 * mavlink_set_position_target_local_ned_t and sending another messages when
 * needed.
 *
 * The task list bellow will wait for the basic information need and check
 * if the state is good, then change to offboard mode, arm the vehicle, takeoff,
 * fly the vehicle in a square shaped path then land and disarm.
 */

#include <arpa/inet.h>
#include <common/time_compat.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <common/mavlink.h>

#include "common/util.h"

#define SYSTEM_ID        11
#define TARGET_SYSTEM_ID 1

#define PX4_MAIN_MODE_CUSTOM     1
#define PX4_CUSTOM_MODE_OFFBOARD 6

#define POLL_ERROR_EVENTS (POLLERR | POLLHUP | POLLNVAL)

#define SETPOINT_RANGE 0.25f

static volatile bool g_should_exit;

static int tcp_fd = -1;
static struct sockaddr_in sockaddr;

static int timeout_fd = -1;

enum px4_modes { PX4_MODE_UNKNOWN = 0, PX4_MODE_OFFBOARD, PX4_MODE_LAST };

static const char *mode_names[] = {"unknown", "offboard"};

static enum px4_modes current_mode = PX4_MODE_UNKNOWN;
static bool armed = false;
static bool in_the_air = false;
static bool landing = false;
static float vehicle_x, vehicle_y, vehicle_z;

#define BASIC_INFO_HEARTBEAT_BIT          (1 << 0)
#define BASIC_INFO_LOCAL_POSITION_NED_BIT (1 << 1)
#define BASIC_INFO_EXTENDED_SYS_STATE_BIT (1 << 2)
#define BASIC_INFO_HOME_POSITION          (1 << 3)
#define BASIC_INFO_ALL_BITS                                       \
    (BASIC_INFO_HEARTBEAT_BIT | BASIC_INFO_LOCAL_POSITION_NED_BIT \
     | BASIC_INFO_EXTENDED_SYS_STATE_BIT | BASIC_INFO_HOME_POSITION)

static uint8_t have_basic_info_mask = 0;

enum actions {
    ARM_DISARM = 0,
    SET_MODE,
    MOVE_X,
    MOVE_Y,
    MOVE_Z,
    WAIT_MOVE,
    LAND,
    WAIT_LAND,
    CHECK_STATE,
    END
};

struct tasks {
    enum actions action;
    int32_t param1;
};

static struct tasks list[] = {
    {.action = CHECK_STATE},
    {.action = SET_MODE},
    {.action = ARM_DISARM, .param1 = 1},
    // Z is inverted in NED https://dev.px4.io/en/ros/external_position_estimation.html#asserting-on-reference-frames
    {.action = MOVE_Z, .param1 = -5},
    {.action = WAIT_MOVE},
    {.action = MOVE_X, .param1 = 5},
    {.action = WAIT_MOVE},
    {.action = MOVE_Y, .param1 = 5},
    {.action = WAIT_MOVE},
    {.action = MOVE_X, .param1 = -5},
    {.action = WAIT_MOVE},
    {.action = MOVE_Y, .param1 = -5},
    {.action = WAIT_MOVE},
    {.action = LAND},
    {.action = WAIT_LAND},
    {.action = ARM_DISARM, .param1 = 0},
    {.action = END}};

static uint8_t task_list_index = 0;

static void exit_signal_handler(int signum)
{
    g_should_exit = true;
}

static int setup_signal_handlers()
{
    struct sigaction sa = {};

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;

    if (sigaction(SIGTERM, &sa, nullptr) != 0 || sigaction(SIGINT, &sa, nullptr) != 0) {
        return -1;
    }

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
        if (ack->result != MAV_RESULT_ACCEPTED) {
            printf("Set mode has failed\n");
        }
        break;
    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (ack->result != MAV_RESULT_ACCEPTED) {
            printf("Arm or disarm command has failed\n");
        }
        break;
    default:
        break;
    }
}

static void handle_heartbeat(mavlink_heartbeat_t *heartbeat)
{
    if (heartbeat->base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
        if (!armed) {
            printf("Status: Vehicle armed\n");
        }
        armed = true;
    } else {
        if (armed) {
            printf("Status: Vehicle disarmed\n");
        }
        armed = false;
    }

    if (heartbeat->base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        enum px4_modes mode = PX4_MODE_UNKNOWN;
        const uint8_t base_custom_mode = heartbeat->custom_mode >> 16;

        if (base_custom_mode == PX4_CUSTOM_MODE_OFFBOARD) {
            mode = PX4_MODE_OFFBOARD;
        }

        if (mode != current_mode) {
            printf("Status: Current mode %s\n", mode_names[mode]);
        }

        current_mode = mode;
    }
}

static void handle_extended_sys_state(mavlink_extended_sys_state_t *extended_sys_state)
{
    switch (extended_sys_state->landed_state) {
    case MAV_LANDED_STATE_IN_AIR:
        if (!in_the_air) {
            printf("Status: Vehicle in the air\n");
        }
        in_the_air = true;
        landing = false;
        break;
    case MAV_LANDED_STATE_ON_GROUND:
        if (in_the_air) {
            printf("Status: Vehicle in the ground\n");
        }
        in_the_air = false;
        landing = false;
        break;
    case MAV_LANDED_STATE_LANDING:
        if (!landing) {
            printf("Status: Landing started\n");
        }
        landing = true;
        break;
    }
}

static void handle_home_position(mavlink_home_position_t *home)
{
    static bool first = true;

    if (first) {
        printf("Status: Got home position\n");
        first = false;
    }
}

static void handle_new_message(const mavlink_message_t *msg)
{
    // we only care about message coming from vehicle
    if (msg->sysid != TARGET_SYSTEM_ID) {
        return;
    }

    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        handle_heartbeat(&heartbeat);
        have_basic_info_mask |= BASIC_INFO_HEARTBEAT_BIT;
        break;
    case MAVLINK_MSG_ID_COMMAND_ACK:
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);
        handle_command_ack(&ack);
        break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        mavlink_local_position_ned_t ned;
        mavlink_msg_local_position_ned_decode(msg, &ned);

        vehicle_x = ned.x;
        vehicle_y = ned.y;
        vehicle_z = ned.z;
        have_basic_info_mask |= BASIC_INFO_LOCAL_POSITION_NED_BIT;
        break;
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
        mavlink_extended_sys_state_t extended_sys_state;
        mavlink_msg_extended_sys_state_decode(msg, &extended_sys_state);
        handle_extended_sys_state(&extended_sys_state);
        have_basic_info_mask |= BASIC_INFO_EXTENDED_SYS_STATE_BIT;
        break;
    case MAVLINK_MSG_ID_HOME_POSITION:
        mavlink_home_position_t home;
        mavlink_msg_home_position_decode(msg, &home);
        handle_home_position(&home);
        have_basic_info_mask |= BASIC_INFO_HOME_POSITION;
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

    mavlink_msg_set_position_target_local_ned_encode(SYSTEM_ID,
                                                     MAV_COMP_ID_ALL,
                                                     &msg,
                                                     &set_position);
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
        break;
    default:
        printf("Error: Mode not handled\n");
        return;
    }

    mavlink_msg_command_long_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, &cmd);
    msg_send(&msg);
}

static void next_action_print(struct tasks *t)
{
    switch (t->action) {
    case ARM_DISARM:
        printf("Requesting %s...\n", t->param1 ? "arm" : "disarm");
        break;
    case SET_MODE:
        printf("Requesting mode %s...\n", mode_names[PX4_MODE_OFFBOARD]);
        break;
    case MOVE_X:
        printf("Moving %i meters in X....\n", t->param1);
        break;
    case MOVE_Y:
        printf("Moving %i meters in Y....\n", t->param1);
        break;
    case MOVE_Z:
        printf("Moving %i meters in Z....\n", t->param1);
        break;
    case WAIT_MOVE:
        break;
    case LAND:
        printf("Requesting land...\n");
        break;
    case WAIT_LAND:
        printf("Waiting landing...\n");
        break;
    case END:
        printf("Offboard mission completed\n");
        break;
    default:
        printf("Unknown action action id=%u\n", t->action);
    }
}

static void timeout_callback()
{
    static float x = 0, y = 0, z = 0, yaw = 0;
    struct tasks *t;

    if (task_list_index >= (sizeof(list) / sizeof(struct tasks))) {
        return;
    }
    t = &list[task_list_index];

    switch (t->action) {
    case CHECK_STATE:
        /*
             * It will only go the next task when we got all basic info.
             * note: home position is not necessary but this is useful to know
             * when PX4 got a GPS fix.
             */
        if (have_basic_info_mask == BASIC_INFO_ALL_BITS) {
            if (!armed && !in_the_air) {
                task_list_index++;
            } else {
                printf("Invalid initial state, please land and disarm\n");
            }
        } else {
            static uint8_t count = 15;

            if (count == 15) {
                printf("Waiting for:\n");
                if (!(have_basic_info_mask & BASIC_INFO_HEARTBEAT_BIT)) {
                    printf("\t- heartbeat\n");
                }
                if (!(have_basic_info_mask & BASIC_INFO_LOCAL_POSITION_NED_BIT)) {
                    printf("\t- local position NED\n");
                }
                if (!(have_basic_info_mask & BASIC_INFO_EXTENDED_SYS_STATE_BIT)) {
                    printf("\t- basic extended info\n");
                }
                if (!(have_basic_info_mask & BASIC_INFO_HOME_POSITION)) {
                    printf("\t- home position(GPS fix)\n");
                }
                count = 0;
            } else {
                count++;
            }
        }
        break;
    case SET_MODE:
        if (current_mode == PX4_MODE_OFFBOARD) {
            task_list_index++;
            break;
        }

        set_mode_send(PX4_MODE_OFFBOARD);
        break;
    case ARM_DISARM: {
        if (static_cast<int>(armed) == t->param1) {
            task_list_index++;
            break;
        }

        mavlink_message_t msg;
        mavlink_command_long_t cmd = {};

        cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
        cmd.target_system = TARGET_SYSTEM_ID;
        cmd.target_component = MAV_COMP_ID_ALL;
        cmd.param1 = t->param1;

        mavlink_msg_command_long_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, &cmd);
        msg_send(&msg);
        break;
    }
    case LAND: {
        if (landing) {
            task_list_index++;
            break;
        }

        mavlink_message_t msg;
        mavlink_command_long_t cmd = {};

        cmd.command = MAV_CMD_NAV_LAND;
        cmd.target_system = TARGET_SYSTEM_ID;
        cmd.target_component = MAV_COMP_ID_ALL;
        cmd.param1 = NAN;
        cmd.param2 = NAN;
        cmd.param3 = NAN;
        cmd.param4 = NAN;
        cmd.param5 = NAN;
        cmd.param6 = NAN;
        cmd.param7 = NAN;

        mavlink_msg_command_long_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, &cmd);
        msg_send(&msg);
        break;
    }
    case WAIT_LAND: {
        if (!in_the_air) {
            task_list_index++;
        }

        break;
    }
    case MOVE_Z: {
        z += t->param1;
        task_list_index++;
        break;
    }
    case MOVE_X: {
        x += t->param1;
        task_list_index++;
        break;
    }
    case MOVE_Y: {
        y += t->param1;
        task_list_index++;
        break;
    }
    case WAIT_MOVE: {
        if (abs(vehicle_z - z) < SETPOINT_RANGE && abs(vehicle_y - y) < SETPOINT_RANGE
            && abs(vehicle_x - x) < SETPOINT_RANGE) {
            task_list_index++;
            printf("Target reached\n");
        }
        break;
    }
    default:
    case END:
        g_should_exit = true;
        break;
    };

    position_set_send(x, y, z, yaw);

    // print next action
    if (t != &list[task_list_index]) {
        next_action_print(&list[task_list_index]);
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

static int setup_connection(const char *ip, int port)
{
    tcp_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_fd == -1) {
        fprintf(stderr, "Could not create socket (%m)\n");
        return -1;
    }

    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(ip);
    sockaddr.sin_port = htons(port);

    printf("Connecting to TCP: %s:%i\n", ip, port);

    if (connect(tcp_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
        fprintf(stderr, "Could not connect to socket (%m)\n");
        return -1;
    }

    if (fcntl(tcp_fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0) {
        fprintf(stderr, "Error setting socket tcp_fd as non-blocking");
        return -1;
    }

    printf("Connected\n");

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

    timerfd_settime(timeout_fd, 0, &ts, nullptr);

    return 0;
}

int main(int argc, char *argv[])
{
    char *ip = strdup("127.0.0.1");
    int port = 5760;

    if (argc > 1) {
        free(ip);
        ip = strdup(argv[1]);
        char *portstr = strchr(ip, ':');
        if (portstr && *portstr) {
            *portstr = '\0';
            port = atoi(portstr + 1);
        }
    }

    if (setup_connection(ip, port) < 0 || setup_timeout() < 0 || setup_signal_handlers() < 0) {
        goto fail;
    }

    free(ip);

    loop();

    close(timeout_fd);
    close(tcp_fd);

    return 0;

fail:
    free(ip);

    if (tcp_fd >= 0) {
        close(tcp_fd);
    }
    if (timeout_fd >= 0) {
        close(timeout_fd);
    }

    return EXIT_FAILURE;
}
