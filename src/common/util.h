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

#include <algorithm>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <time.h>
#include <vector>

#include "macro.h"

using usec_t = uint64_t;
using nsec_t = uint64_t;
#define USEC_INFINITY ((usec_t)-1)

#define MSEC_PER_SEC  1000ULL
#define USEC_PER_SEC  ((usec_t)1000000ULL)
#define USEC_PER_MSEC ((usec_t)1000ULL)
#define NSEC_PER_SEC  ((nsec_t)1000000000ULL)
#define NSEC_PER_MSEC ((nsec_t)1000000ULL)
#define NSEC_PER_USEC ((nsec_t)1000ULL)

#define ARRAY_SIZE(arr)               (sizeof(arr) / sizeof((arr)[0]))
#define streq(a, b)                   (strcmp((a), (b)) == 0)
#define strcaseeq(a, b)               (strcasecmp((a), (b)) == 0)
#define strncaseeq(a, b, len)         (strncasecmp((a), (b), (len)) == 0)
#define memcaseeq(a, len_a, b, len_b) ((len_a) == (len_b) && strncaseeq(a, b, len_a))

int safe_atoull(const char *s, unsigned long long *ret);
int safe_atoul(const char *s, unsigned long *ret);
int safe_atoi(const char *s, int *ret);
usec_t now_usec();
usec_t ts_usec(const struct timespec *ts);

int mkdir_p(const char *path, int len, mode_t mode);

template <typename type> bool vector_contains(std::vector<type> vect, type elem)
{
    return std::find(vect.begin(), vect.end(), elem) != vect.end();
}

#ifndef strndupa
#    define strndupa(s, n)                           \
        (__extension__({                             \
            const char *__in = (s);                  \
            size_t __len = strnlen(__in, (n));       \
            char *__out = (char *)alloca(__len + 1); \
            __out[__len] = '\0';                     \
            (char *)memcpy(__out, __in, __len);      \
        }))
#endif
