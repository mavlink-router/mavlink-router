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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <common/mavlink.h>

#define SYSTEM_ID 10

#define AUTH_TIMEOUT_SEC      10
#define HEARTBEAT_TIMEOUT_SEC 3

#define MAX_VALID_ALTITUDE 15.0f

// 1 minute of authorization
#define AUTHORIZATION_TIMEOUT_SEC 60

static volatile bool g_should_exit;

static int fd;
static struct sockaddr_in sockaddr;

static timespec auth_request_timeout;
static timespec heartbeat_timeout;

static uint8_t target_system;

enum state_t {
    STATE_IDLE = 0,
    STATE_WAITING_MISSING_COUNT,
    STATE_WAITING_MISSION_ITEMS,
    STATE_LAST
};

static state_t state = STATE_IDLE;
static uint16_t mission_item_count;
static bool mission_valid;
static uint16_t first_mission_item_invalid;

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

static int msg_send(mavlink_message_t *msg)
{
    uint8_t data[MAVLINK_MAX_PACKET_LEN];

    uint16_t len = mavlink_msg_to_send_buffer(data, msg);
    return sendto(fd, data, len, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}

static void command_ack_send(mavlink_command_ack_t *ack)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, ack);
    msg_send(&msg);
}

static void mission_request_int_send(uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_mission_request_int_t mission_req;

    mission_req.mission_type = MAV_MISSION_TYPE_MISSION;
    mission_req.target_system = target_system;
    mission_req.target_component = MAV_COMP_ID_ALL;
    mission_req.seq = seq;

    mavlink_msg_mission_request_int_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, &mission_req);
    msg_send(&msg);
}

static void mission_ack_send()
{
    mavlink_message_t msg;
    mavlink_mission_ack_t mission_ack;

    mission_ack.target_system = target_system;
    mission_ack.target_component = MAV_COMP_ID_ALL;
    mission_ack.type = 0;
    mission_ack.mission_type = MAV_MISSION_TYPE_MISSION;
    mission_ack.opaque_id = 0;

    mavlink_msg_mission_ack_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, &mission_ack);
    msg_send(&msg);
}

static void handle_command_long(const mavlink_message_t *msg, mavlink_command_long_t *cmd)
{
    if (cmd->command != MAV_CMD_ARM_AUTHORIZATION_REQUEST || state != STATE_IDLE
        || cmd->target_system != SYSTEM_ID) {
        return;
    }

    target_system = msg->sysid;
    state = STATE_WAITING_MISSING_COUNT;

    printf("Received request of authorization to arm target_system=%u\n", target_system);

    mavlink_command_ack_t ack = {};
    ack.target_system = target_system;
    ack.target_component = MAV_COMP_ID_ALL;
    ack.command = MAV_CMD_ARM_AUTHORIZATION_REQUEST;
    ack.result = MAV_RESULT_IN_PROGRESS;
    ack.progress = 255;
    ack.result_param2 = 0;
    command_ack_send(&ack);

    mavlink_mission_request_list_t count_req = {};
    mavlink_message_t msg_output;

    count_req.target_system = target_system;
    count_req.target_component = MAV_COMP_ID_ALL;
    count_req.mission_type = MAV_MISSION_TYPE_MISSION;
    mavlink_msg_mission_request_list_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg_output, &count_req);
    msg_send(&msg_output);

    clock_gettime(CLOCK_MONOTONIC, &auth_request_timeout);
    auth_request_timeout.tv_sec += AUTH_TIMEOUT_SEC;
}

static void handle_mission_count(mavlink_mission_count_t *mission_count)
{
    if (state != STATE_WAITING_MISSING_COUNT || mission_count->target_system != SYSTEM_ID) {
        return;
    }

    if (!mission_count->count) {
        mavlink_command_ack_t ack = {};

        ack.target_system = target_system;
        ack.target_component = MAV_COMP_ID_ALL;
        ack.command = MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        ack.result = MAV_RESULT_DENIED;
        ack.progress = MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT;
        ack.result_param2 = 0;
        printf("Arm request denied because there is no mission loaded\n");

        command_ack_send(&ack);
        state = STATE_IDLE;

        return;
    }

    mission_item_count = mission_count->count;
    printf("Mission count=%u\n", mission_item_count);
    state = STATE_WAITING_MISSION_ITEMS;
    mission_valid = true;

    mission_request_int_send(0);
}

static void handle_mission_item_int(mavlink_mission_item_int_t *mission_item)
{
    if (state != STATE_WAITING_MISSION_ITEMS || mission_item->target_system != SYSTEM_ID) {
        return;
    }

    float x = mission_item->x, y = mission_item->y;
    x /= 10000000;
    y /= 10000000;

    printf("Mission item { seq=%u mission_type=%u x=%f y=%f altitude=%f }\n",
           mission_item->seq,
           mission_item->mission_type,
           x,
           y,
           mission_item->z);

    if (mission_valid && mission_item->z > MAX_VALID_ALTITUDE) {
        mission_valid = false;
        first_mission_item_invalid = mission_item->seq;
        printf("Arm request will be denied because mission item seq=%u have a invalid altitude, "
               "altitude=%f max accepted altitude=%f\n",
               mission_item->seq,
               mission_item->z,
               MAX_VALID_ALTITUDE);
    }

    mission_item->seq++;

    if (mission_item->seq == mission_item_count) {
        mavlink_command_ack_t ack;
        ack.target_system = target_system;
        ack.target_component = MAV_COMP_ID_ALL;
        ack.command = MAV_CMD_ARM_AUTHORIZATION_REQUEST;
        if (mission_valid) {
            ack.result = MAV_RESULT_ACCEPTED;
            ack.progress = 0;
            ack.result_param2 = AUTHORIZATION_TIMEOUT_SEC;
            printf("Arm request accepted\n");
        } else {
            ack.result = MAV_RESULT_DENIED;
            ack.progress = MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT;
            ack.result_param2 = first_mission_item_invalid;
            printf("Arm request denied\n");
        }
        command_ack_send(&ack);

        mission_ack_send();

        state = STATE_IDLE;
    } else {
        mission_request_int_send(mission_item->seq);
    }
}

static void handle_new_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);
        handle_command_long(msg, &cmd);
        break;
    case MAVLINK_MSG_ID_MISSION_COUNT:
        mavlink_mission_count_t mission_count;
        mavlink_msg_mission_count_decode(msg, &mission_count);
        handle_mission_count(&mission_count);
        break;
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        mavlink_mission_item_int_t mission_item;
        mavlink_msg_mission_item_int_decode(msg, &mission_item);
        handle_mission_item_int(&mission_item);
        break;
    default:
        break;
    }
}

/* return > 0 if t1 is greater, return < 0 if t2 is greater or 0 otherwise */
static long int timespec_compare(timespec *t1, timespec *t2)
{
    long int r = t1->tv_sec - t2->tv_sec;
    if (r) {
        return r;
    }

    return t1->tv_nsec - t2->tv_nsec;
}

static void loop()
{
    heartbeat_timeout.tv_sec = 0;
    heartbeat_timeout.tv_nsec = 0;

    while (!g_should_exit) {
        mavlink_message_t msg;
        mavlink_status_t status;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        socklen_t addrlen = sizeof(sockaddr);
        ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&sockaddr, &addrlen);

        if (n == -1) {
            if (errno == EINTR) {
                continue;
            }

            fprintf(stderr, "Error reading socket (%m)\n");
            return;
        }

        for (int i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                handle_new_message(&msg);
            }
        }

        timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        /* send heartbeat at 0.333 hz */
        if (timespec_compare(&now, &heartbeat_timeout) > 0) {
            mavlink_heartbeat_t heartbeat;

            heartbeat.type = MAV_TYPE_ONBOARD_CONTROLLER;
            heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
            heartbeat.base_mode = 0;
            heartbeat.custom_mode = 0;
            heartbeat.system_status = MAV_STATE_ACTIVE;

            mavlink_msg_heartbeat_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, &heartbeat);
            msg_send(&msg);

            heartbeat_timeout.tv_sec = now.tv_sec + HEARTBEAT_TIMEOUT_SEC;
            heartbeat_timeout.tv_nsec = now.tv_nsec;
        }

        /* check authorization timeout */
        if (state != STATE_IDLE) {
            if (timespec_compare(&now, &auth_request_timeout) > 0) {
                printf("Arm authorizer timeout, last state was %u...\n", state);

                mavlink_command_ack_t ack;
                ack.target_system = target_system;
                ack.target_component = MAV_COMP_ID_ALL;
                ack.command = MAV_CMD_ARM_AUTHORIZATION_REQUEST;
                ack.result = MAV_RESULT_FAILED;
                ack.progress = MAV_ARM_AUTH_DENIED_REASON_TIMEOUT;
                ack.result_param2 = 0;

                command_ack_send(&ack);

                /* to cancel mission list request */
                mission_ack_send();

                state = STATE_IDLE;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    const char *ip = "127.0.0.1";
    int port = 5760;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == -1) {
        fprintf(stderr, "Could not create socket (%m)\n");
        return 1;
    }

    memset(&sockaddr, 0, sizeof(struct sockaddr_in));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(ip);
    sockaddr.sin_port = htons(port);

    int ret = connect(fd, (struct sockaddr *)&sockaddr, sizeof(struct sockaddr_in));
    if (ret) {
        fprintf(stderr, "Could not connect to socket (%m)\n");
        goto connect_error;
    }

    setup_signal_handlers();
    loop();

    return 0;

connect_error:
    close(fd);
    return -1;
}
