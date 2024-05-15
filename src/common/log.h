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
#pragma once

#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>

#include "macro.h"

class Log {
public:
    enum class Level {
        ERROR = 0,
        WARNING,
        NOTICE,
        INFO,
        DEBUG,
        TRACE,
    };

    enum class Backend {
        STDERR,
        SYSLOG,
    };

    static int open(Backend backend);
    static int close();

    static Level get_max_level() _pure_ { return _max_level; }
    static void set_max_level(Level level);

    static void logv(Level level, const char *format, va_list ap);
    static void log(Level level, const char *format, ...) _printf_format_(2, 3);

protected:
    static void logv_to_fd(int fd, Level level, const char *format, va_list ap);
    static const char *_get_color(Level level);

    static int syslog_log_level(Level level);

    static int _target_fd;
    static Level _max_level;
    static bool _show_colors;
    static Backend _backend;
};

#define log_trace(...)   Log::log(Log::Level::TRACE, __VA_ARGS__)
#define log_debug(...)   Log::log(Log::Level::DEBUG, __VA_ARGS__)
#define log_info(...)    Log::log(Log::Level::INFO, __VA_ARGS__)
#define log_notice(...)  Log::log(Log::Level::NOTICE, __VA_ARGS__)
#define log_warning(...) Log::log(Log::Level::WARNING, __VA_ARGS__)
#define log_error(...)   Log::log(Log::Level::ERROR, __VA_ARGS__)

#define assert_or_return(exp, ...)                          \
    do {                                                    \
        if (__builtin_expect(!(exp), 0)) {                  \
            log_warning("Expresssion `" #exp "` is false"); \
            return __VA_ARGS__;                             \
        }                                                   \
    } while (0)
