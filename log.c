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

#include <alloca.h>
#include <assert.h>
#include <limits.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <sys/uio.h>
#include <unistd.h>

#include "log.h"

#define COLOR_RED          "\033[31m"
#define COLOR_LIGHTBLUE    "\033[34;1m"
#define COLOR_YELLOW       "\033[33;1m"
#define COLOR_ORANGE       "\033[0;33m"
#define COLOR_WHITE        "\033[37;1m"
#define COLOR_RESET        "\033[0m"


static int log_max_level = LOG_INFO;
static int log_target_fd = STDERR_FILENO;
static bool log_show_colors;

static const char *level_colors[] = {
    [LOG_ERR] =     COLOR_RED,
    [LOG_WARNING] = COLOR_ORANGE,
    [LOG_NOTICE] =  COLOR_YELLOW,
    [LOG_INFO] =    COLOR_WHITE,
    [LOG_DEBUG] =   COLOR_LIGHTBLUE,
};

int log_open(void)
{
    if (isatty(log_target_fd))
        log_show_colors = true;

    return 0;
}

int log_close(void)
{
    return 0;
}

void log_set_max_level(int level)
{
    assert((level & LOG_PRIMASK) == level);

    log_max_level = level;
}

int log_get_max_level(void)
{
    return log_max_level;
}

static const char *get_color(int level)
{
    if (!log_show_colors)
        return NULL;
    return level_colors[level];
}

static int log_internalv(int level, int error,
                         const char *file, int line,
                         const char *format, va_list ap)
{
    struct iovec iovec[6] = { };
    const char *color;
    int n = 0;
    char location[64];
    char buffer[LINE_MAX];
    int save_errno;

    assert((level & LOG_PRIMASK) == level);

    /* so %m works as expected */
    save_errno = errno;

    color = get_color(level);

    if (file) {
        snprintf(location, sizeof(location), "(%s:%d) ", file, line);
        IOVEC_SET_STRING(iovec[n++], location);
    }

    if (color)
        IOVEC_SET_STRING(iovec[n++], color);

    errno = save_errno;
    vsnprintf(buffer, sizeof(buffer), format, ap);

    IOVEC_SET_STRING(iovec[n++], buffer);

    if (color)
        IOVEC_SET_STRING(iovec[n++], COLOR_RESET);

    IOVEC_SET_STRING(iovec[n++], "\n");

    (void)writev(log_target_fd, iovec, n);

    return -abs(error);
}

int log_internal(int level, int error,
                 const char *file, int line,
                 const char *format, ...)
{
    va_list ap;
    int r;

    va_start(ap, format);
    r = log_internalv(level, error, file, line, format, ap);
    va_end(ap);

    return r;
}
