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
 * Optional compile-time debug messages, usually used to debug verbose
 * messages on single compilation units.
 *
 * dbg() expands to nothing if LOG_DEBUG is not defined, or to the level
 * LOG_DEBUG is defined to (default to Log::Level::DEBUG
 *
 * This header is not guarded for multiple inclusion since it changes behavior
 * according to the value of LOG_DEBUG. This allows to debug single
 * compilation units without affecting others. Don't include it in a header.
 */
#include "log.h"

#ifndef LOG_DEBUG
#  define dbg(...) do { } while (0)
#else
#  ifdef dbg
#    undef dbg
#  endif

#  ifndef LOG_DEBUG_LEVEL
#    define LOG_DEBUG_LEVEL Log::Level::DEBUG
#  endif

#  define dbg(...) Log::log(LOG_DEBUG_LEVEL, __VA_ARGS__)
#endif
