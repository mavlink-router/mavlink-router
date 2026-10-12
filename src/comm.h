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

#include <arpa/inet.h>
#include <asm/termbits.h>
#include <errno.h>
#include <inttypes.h>

#include <common/macro.h>

struct buffer {
    unsigned int len;
    uint8_t *data;

    /*
     * Data relevant for the last parsed msg available on this buffer,
     * copied from @data when we have a complete header
     */
    struct {
        uint32_t msg_id;
        int target_sysid;
        int target_compid;
        uint8_t src_sysid;
        uint8_t src_compid;
        uint8_t payload_len;
        uint8_t *payload;
        /*
         * Whether this message had a recognized msgid and passed CRC
         * validation. Unrecognized/corrupted frames are still forwarded
         * (for forward-compat with newer message types), but must not be
         * used to learn routing state: a corrupted frame can carry
         * arbitrary sysid/compid bytes that happen to collide with a real
         * endpoint's identity, permanently poisoning that endpoint's
         * loop-prevention table for the colliding identity.
         */
        bool validated;
    } curr;
};
