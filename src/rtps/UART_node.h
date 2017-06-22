// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstring>
#include <string>

class UART_node {
public:
    UART_node();
    virtual ~UART_node();

    int init_uart(const char *uart_name, uint32_t baudrate);
    uint8_t close_uart();
    int writeRTPStoUART(const char topic_ID, char buffer[], uint16_t length);
    int writeMavlinkToUART(char buffer[], uint16_t length);
    int read();
    int parseMavlinkFromUART(char buffer[], size_t buflen);
    int parseRTPSfromUART(char *topic_ID, uint8_t *seq, char buffer[], size_t buflen);

protected:
    uint16_t crc16_byte(uint16_t crc, const uint8_t data);
    uint16_t crc16(uint8_t const *buffer, size_t len);

protected:
    int m_uart_filestream = -1;
    uint8_t _buf[1024] = {};
    size_t _buf_size = 0;

    static const size_t BUFFER_THRESHOLD = sizeof(_buf) * 0.8;

private:
    struct __attribute__((packed)) Header {
        char marker[3];
        uint8_t topic_ID;
        uint8_t seq;
        uint8_t payload_len_h;
        uint8_t payload_len_l;
        uint8_t crc_h;
        uint8_t crc_l;
    };
};
