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

#include <functional>

#include "pollable.h"

class Timeout : public Pollable {
public:
    Timeout(std::function<bool(void*)> cb, const void *data);
    bool remove_me = false;
    Timeout *next = nullptr;

    int handle_read() override;
    bool handle_canwrite() override;

private:
    std::function<bool(void*)> _cb;
    const void *_data;
};
