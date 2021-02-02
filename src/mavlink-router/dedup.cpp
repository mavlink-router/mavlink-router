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


#include "dedup.h"

#include <chrono>
#include <string>
#include <queue>
#include <unordered_set>

class DedupImpl {
    using hash_t = uint64_t;
    using time_t = uint64_t;

public:

    bool add_check_packet(const uint8_t* buffer, uint32_t size, uint32_t dedup_period_ms)
    {
        bool new_packet_hash = true;
        using namespace std::chrono;
        time_t timestamp = duration_cast<milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // pop data from front queue, delete corresponding data from multiset
        while (_time_hash_queue.size() > 0 && _time_hash_queue.front().first > timestamp + dedup_period_ms) {
            hash_t hash_to_delete = _time_hash_queue.front().second;
            _multiset.erase(_multiset.find(hash_to_delete)); // NOTE: don't call erase on key, it will delete all
            _time_hash_queue.pop();
        }

        // hash buffer
        // TODO: with C++17 use a string_view instead, or use a custom hash function
        _hash_buffer.assign((const char*)buffer, (uint64_t)size);
        hash_t hash = std::hash<std::string>{}(_hash_buffer);

        if (_multiset.find(hash) != _multiset.end()) {
            new_packet_hash = false;
        }

        // add hash and timestamp to back of queue, and add another copy of hash to multiset
        _multiset.insert(hash);
        _time_hash_queue.emplace(timestamp, hash);

        return new_packet_hash;
    }

private:
    std::queue<std::pair<time_t, hash_t>> _time_hash_queue;
    std::unordered_multiset<hash_t> _multiset;
    std::string _hash_buffer;
};

Dedup::Dedup(uint32_t dedup_period_ms) : _dedup_period_ms(dedup_period_ms), _impl(new DedupImpl())
{
}

Dedup::~Dedup()
{
}

void Dedup::set_dedup_period(uint32_t dedup_period_ms)
{
    _dedup_period_ms = dedup_period_ms;
}

Dedup::PacketStatus Dedup::add_check_packet(const uint8_t* buffer, uint32_t size)
{
    // short circuit if disabled
    if (_dedup_period_ms == 0) {
        return PacketStatus::NEW_PACKET_OR_TIMED_OUT;
    }

    if (_impl->add_check_packet(buffer, size, _dedup_period_ms)) {
        return PacketStatus::NEW_PACKET_OR_TIMED_OUT;
    }
    return PacketStatus::ALREADY_EXISTS_IN_BUFFER;
}
