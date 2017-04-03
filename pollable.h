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

class Pollable {
public:
    int fd = -1;
    virtual ~Pollable();
    virtual int handle_read() = 0;
    /**
     * Flush any pending messages.
     * Return true if there still more messages to be flushed.
     */
    virtual bool handle_canwrite() = 0;

    /**
     * If a pollabe isn't valid anymore, it should be removed
     * from poll.
     */
    virtual bool is_valid() { return true; };
};
