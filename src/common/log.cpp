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
#include "log.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/uio.h>
#include <syslog.h>
#include <unistd.h>

#define COLOR_RED       "\033[31m"
#define COLOR_LIGHTBLUE "\033[34;1m"
#define COLOR_YELLOW    "\033[33;1m"
#define COLOR_ORANGE    "\033[0;33m"
#define COLOR_WHITE     "\033[37;1m"
#define COLOR_RESET     "\033[0m"

Log::Level Log::_max_level = Level::INFO;
int Log::_target_fd = -1;
bool Log::_show_colors;
Log::Backend Log::_backend = Log::Backend::STDERR;

const char *Log::_get_color(Level level)
{
    if (!_show_colors) {
        return nullptr;
    }

    switch (level) {
    case Level::ERROR:
        return COLOR_RED;
    case Level::WARNING:
        return COLOR_ORANGE;
    case Level::NOTICE:
        return COLOR_YELLOW;
    case Level::INFO:
        return COLOR_WHITE;
    case Level::DEBUG:
        return COLOR_LIGHTBLUE;
    case Level::TRACE:
        break;
    }

    return nullptr;
}

int Log::open(Backend backend)
{
    assert_or_return(_target_fd < 0, -1);

    _backend = backend;

    switch (backend) {
    case Backend::STDERR:
        _target_fd = STDERR_FILENO;

        if (isatty(_target_fd))
            _show_colors = true;

        break;
    case Backend::SYSLOG:
        openlog(nullptr, LOG_CONS, LOG_USER);
        break;
    }

    return 0;
}

int Log::close()
{
    switch (_backend) {
    case Backend::STDERR:
        /* see _target_fd on open() */
        fflush(stderr);
        break;
    case Backend::SYSLOG:
        closelog();
        break;
    }

    return 0;
}

void Log::set_max_level(Level level)
{
    _max_level = level;
}

void Log::logv_to_fd(int fd, Log::Level level, const char *format, va_list ap)
{
    struct iovec iovec[6] = {};
    const char *color;
    int n = 0;
    char buffer[LINE_MAX];
    int save_errno;

    /* so %m works as expected */
    save_errno = errno;

    color = _get_color(level);

    if (color) {
        IOVEC_SET_STRING(iovec[n++], color);
    }

    errno = save_errno;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    vsnprintf(buffer, sizeof(buffer), format, ap);
#pragma GCC diagnostic pop

    IOVEC_SET_STRING(iovec[n++], buffer);

    if (color) {
        IOVEC_SET_STRING(iovec[n++], COLOR_RESET);
    }

    IOVEC_SET_STRING(iovec[n++], "\n");

    (void)writev(fd, iovec, n);
}

void Log::logv(Level level, const char *format, va_list ap)
{
    if (_max_level < level) {
        return;
    }

    switch (_backend) {
    case Backend::STDERR:
        logv_to_fd(_target_fd, level, format, ap);
        break;
    case Backend::SYSLOG:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
        vsyslog(syslog_log_level(level), format, ap);
#pragma GCC diagnostic pop
        break;
    }
}

void Log::log(Level level, const char *format, ...)
{
    va_list ap;

    va_start(ap, format);
    logv(level, format, ap);
    va_end(ap);
}

int Log::syslog_log_level(Log::Level level)
{
    switch (level) {
    case Level::ERROR:
        return LOG_ERR;
    case Level::WARNING:
        return LOG_WARNING;
    case Level::NOTICE:
        return LOG_NOTICE;
    case Level::INFO:
        return LOG_INFO;
    case Level::DEBUG:
        return LOG_DEBUG;
    case Level::TRACE:
        return LOG_DEBUG;
    }
    return LOG_DEBUG;
}
