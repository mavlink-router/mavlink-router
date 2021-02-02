/*
 * This file is part of the MAVLink Router project
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

#include <memory>

class DedupImpl;
class Dedup {
public:

    enum class PacketStatus {
        NEW_PACKET_OR_TIMED_OUT,
        ALREADY_EXISTS_IN_BUFFER
    };

    Dedup(uint32_t dedup_period_ms = 0);
    ~Dedup();

    void set_dedup_period(uint32_t dedup_period_ms);

    PacketStatus add_check_packet(const uint8_t* buffer, uint32_t size);
private:

    uint32_t _dedup_period_ms;
    std::unique_ptr<DedupImpl> _impl;
};
