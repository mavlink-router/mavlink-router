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
#include "timeout.h"

#include <assert.h>
#include <stdint.h>
#include <unistd.h>

Timeout::Timeout(std::function<bool(void*)> cb, const void *data)
{
    assert(cb);
    _cb = cb;
    _data = data;
}

int Timeout::handle_read()
{
    uint64_t val = 0;
    int ret = read(fd, &val, sizeof(val));

    if (ret < 1 || val == 0 || remove_me)
        return 0;

    if (!_cb((void *)_data))
        remove_me = true;

    return 0;
}

bool Timeout::handle_canwrite()
{
    return false;
}
