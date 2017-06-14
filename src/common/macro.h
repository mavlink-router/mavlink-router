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

#define _must_check_ __attribute__((warn_unused_result))
#define _printf_format_(a,b) __attribute__((format (printf, a, b)))
#define _unused_ __attribute__((unused))
#define _always_inline_ __inline__ __attribute__((always_inline))
#define _cleanup_(x) __attribute__((cleanup(x)))
#define _pure_ __attribute__((pure))
#define _packed_ __attribute__((packed))

#define IOVEC_SET_STRING(i, s)          \
    do {                                \
        struct iovec *_i = &(i);        \
        char *_s = (char *)(s);         \
        _i->iov_base = _s;              \
        _i->iov_len = strlen(_s);       \
    } while(0)
