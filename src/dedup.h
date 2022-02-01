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

/*
 * De-duplication of raw buffers.
 *
 * When presented with a buffer, this class calculates a hash value of the
 * data and determines whether that hash was already seen in a configureable
 * time period (dedup_period). A known hash value will reset the time for that
 * value. Old hash values will be cleaned up each time check_packet is called.
 *
 * A dedup_period of 0 will disable all de-duplication checks.
 */
class Dedup {
public:
    enum class PacketStatus { NEW_PACKET_OR_TIMED_OUT, ALREADY_EXISTS_IN_BUFFER };

    Dedup(uint32_t dedup_period_ms = 0);
    ~Dedup();

    void set_dedup_period(uint32_t dedup_period_ms);

    PacketStatus check_packet(const uint8_t *buffer, uint32_t size);

private:
    uint32_t _dedup_period_ms; ///< how long
    std::unique_ptr<DedupImpl> _impl;
};
