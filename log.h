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
#pragma once

#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <syslog.h>

#include "macro.h"

#ifdef __cplusplus
extern "C" {
#endif

int log_open(void);
int log_close(void);

int log_get_max_level(void) _pure_;
void log_set_max_level(int level);
int log_internal(int level, int error,
                 const char *file, int line,
                 const char *format, ...) _printf_format_(5, 6);

#define log_full_errno(level, error, ...)                               \
    ({                                                                  \
         int _level = (level), _e = (error);                            \
         (log_get_max_level() >= LOG_PRI(_level))                       \
            ? log_internal(_level, _e, __FILE__, __LINE__, __VA_ARGS__) \
            : -abs(_e);                                                 \
        })

#define log_full(level, ...) log_full_errno(level, 0, __VA_ARGS__)

/* Normal logging */
#define log_debug(...)     log_full(LOG_DEBUG,   __VA_ARGS__)
#define log_info(...)      log_full(LOG_INFO,    __VA_ARGS__)
#define log_notice(...)    log_full(LOG_NOTICE,  __VA_ARGS__)
#define log_warning(...)   log_full(LOG_WARNING, __VA_ARGS__)
#define log_error(...)     log_full(LOG_ERR,     __VA_ARGS__)

/* Logging triggered by an errno-like error */
#define log_debug_errno(error, ...)     log_full_errno(LOG_DEBUG,   error, __VA_ARGS__)
#define log_info_errno(error, ...)      log_full_errno(LOG_INFO,    error, __VA_ARGS__)
#define log_notice_errno(error, ...)    log_full_errno(LOG_NOTICE,  error, __VA_ARGS__)
#define log_warning_errno(error, ...)   log_full_errno(LOG_WARNING, error, __VA_ARGS__)
#define log_error_errno(error, ...)     log_full_errno(LOG_ERR,     error, __VA_ARGS__)

#define null_check(ptr, ...)                 \
    do {                                     \
        if (__builtin_expect(!(ptr), 0)) {   \
            log_warning("%s == NULL", #ptr); \
            return __VA_ARGS__;              \
        }                                    \
    } while (0)

#ifdef __cplusplus
}
#endif
