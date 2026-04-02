#pragma once

#include <cstdint>

int init_serial(int fd, const char *path);
int set_serial_speed(int fd, uint32_t baudrate);
int set_serial_flow_control(int fd, bool enabled);
