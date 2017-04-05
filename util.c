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
#include "util.h"

#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

usec_t ts_usec(const struct timespec *ts)
{
    if (ts->tv_sec == (time_t) -1 &&
        ts->tv_nsec == (long) -1)
        return USEC_INFINITY;

    if ((usec_t) ts->tv_sec > (UINT64_MAX - (ts->tv_nsec / NSEC_PER_USEC)) / USEC_PER_SEC)
        return USEC_INFINITY;

    return
        (usec_t) ts->tv_sec * USEC_PER_SEC +
        (usec_t) ts->tv_nsec / NSEC_PER_USEC;
}

usec_t now_usec(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return ts_usec(&ts);
}

int safe_atoul(const char *s, unsigned long *ret)
{
    char *x = NULL;
    unsigned long l;

    assert(s);
    assert(ret);

    errno = 0;
    l = strtoul(s, &x, 0);

    if (!x || x == s || *x || errno)
        return errno ? -errno : -EINVAL;

    *ret = l;

    return 0;
}

int safe_atoull(const char *s, unsigned long long *ret)
{
    char *x = NULL;
    unsigned long long l;

    assert(s);
    assert(ret);

    errno = 0;
    l = strtoull(s, &x, 0);

    if (!x || x == s || *x || errno)
        return errno ? -errno : -EINVAL;

    *ret = l;

    return 0;
}

int safe_atoi(const char *s, int *ret)
{
    char *x = NULL;
    long l;

    assert(s);
    assert(ret);

    errno = 0;
    l = strtol(s, &x, 0);

    if (!x || x == s || *x || errno)
        return errno > 0 ? -errno : -EINVAL;

    if ((long) (int) l != l)
        return -ERANGE;

    *ret = (int) l;
    return 0;
}

static inline int is_dir(const char *path)
{
    struct stat st;

    if (stat(path, &st) >= 0)
        return S_ISDIR(st.st_mode);

    return -errno;
}

int mkdir_p(const char *path, int len, mode_t mode)
{
    char *start, *end;

    start = strndupa(path, len);
    end = start + len;

    /*
     * scan backwards, replacing '/' with '\0' while the component doesn't
     * exist
     */
    for (;;) {
        int r = is_dir(start);
        if (r > 0) {
            end += strlen(end);

            if (end == start + len)
                return 0;

            /* end != start, since it would be caught on the first
             * iteration */
            *end = '/';
            break;
        } else if (r == 0)
            return -ENOTDIR;

        if (end == start)
            break;

        *end = '\0';

        /* Find the next component, backwards, discarding extra '/'*/
        while (end > start && *end != '/')
            end--;

        while (end > start && *(end - 1) == '/')
            end--;
    }

    for (; end < start + len;) {
        if (mkdir(start, mode) < 0 && errno != EEXIST)
            return -errno;

        end += strlen(end);
        *end = '/';
    }

    return 0;
}
