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
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>

//#include <iostream>

#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <iostream>
#include "UART_node.h"
#include "log.h"

#define DEFAULT_UART "/dev/ttyACM0"

/** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
uint16_t const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

UART_node::UART_node(): m_uart_filestream(0)
{

}

UART_node::~UART_node()
{
    close_uart();
}

int UART_node::init_uart(const char *uart_name, uint32_t baudrate)
{
    struct termios2 tc;
    int fd;

    fd = ::open(uart_name, O_RDWR | O_NONBLOCK | O_CLOEXEC | O_NOCTTY);
    if (fd < 0) {
        log_error("Could not open %s (%m)", uart_name);
        return -1;
    }

    bzero(&tc, sizeof(tc));

    if (ioctl(fd, TCGETS2, &tc) == -1) {
        log_error("Could not get termios2 (%m)");
        goto fail;
    }

    tc.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    tc.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    tc.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
    tc.c_cflag &= ~(CSIZE | PARENB | CBAUD | CRTSCTS);
    tc.c_cflag |= CS8 | BOTHER;

    tc.c_cc[VMIN] = 0;
    tc.c_cc[VTIME] = 0;
    tc.c_ispeed = baudrate;
    tc.c_ospeed = baudrate;

    if (ioctl(fd, TCSETS2, &tc) == -1) {
        log_error("Could not set terminal attributes (%m)");
        goto fail;
    }

    if (ioctl(fd, TCFLSH, TCIOFLUSH) == -1) {
        log_error("Could not flush terminal (%m)");
        goto fail;
    }

    log_info("Open UART [%d] %s:%u *\n", fd, uart_name, baudrate);

    m_uart_filestream = fd;
    return m_uart_filestream;

fail:
    ::close(fd);
    fd = -1;
    return -1;
}

uint8_t UART_node::close_uart()
{
    log_debug("Close UART\n");
    close(m_uart_filestream);
    return 0;
}

int UART_node::read()
{
    /* Discard whole buffer if it's filled beyond a threshold,
     * This should prevent buffer being filled by garbage that
     * no reader (MAVLink or RTPS) can understand.
     *
     * TODO: a better approach would be checking if both reader
     * start understanding messages beyond a certain buffer size,
     * meaning that everything before is garbage.
     */
    if (_buf_size > BUFFER_THRESHOLD) {
        _buf_size = 0;
    }

    int r = ::read(m_uart_filestream, _buf + _buf_size, sizeof(_buf) - _buf_size);
    if (r < 0) {
        if (errno != EAGAIN) {
            log_error("Error reading UART %d\n", errno);
        }
        return r;
    }

    _buf_size += r;

    return r;
}

int UART_node::parseMavlinkFromUART(char buffer[], size_t buflen)
{
    unsigned i;

    if (m_uart_filestream == -1)
        return 2;

    if (_buf_size < 3)
        return 0;

    // Search for a mavlink packet on buffer to send it
    i = 0;
    while (i < (_buf_size - 3) && _buf[i] != 253 && _buf[i] != 254)
        i++;

    // We need at least the first three bytes to get packet len
    if (i == _buf_size - 3) {
        return 0;
    }

    uint16_t packet_len;
    if (_buf[i] == 253) {
        uint8_t payload_len = _buf[i + 1];
        uint8_t incompat_flags = _buf[i + 2];
        packet_len = payload_len + 12;

        if (incompat_flags & 0x1) { // signing
            packet_len += 13;
        }
    } else {
        packet_len = _buf[i + 1] + 8;
    }

    // cerr << "mavlink parse. message len " << (int)packet_len << " on: " << i << endl;

    // packet is bigger than what we've read, better luck next time
    if (i + packet_len > _buf_size) {
        return 0;
    }

    // buffer should be big enough to hold a mavlink packet
    if (packet_len > buflen) {
        return -EMSGSIZE;
    }

    // Found a whole message, send it and remove from local _buf
    memmove(buffer, _buf + i, packet_len);
    memmove(_buf + i, _buf + i + packet_len, sizeof(_buf) - i - packet_len);
    _buf_size -= packet_len;

    return packet_len;
}

int UART_node::parseRTPSfromUART(uint8_t *topic_ID, uint8_t *seq, char buffer[], size_t buflen)
{
    size_t start = 0;
    uint16_t len;
    if (m_uart_filestream == -1)
        return 2;

    if (_buf_size < sizeof(struct Header)) // starting ">>>" + topic + seq + len + crchi + crclow
        return 0;

    // look for starting ">>>"
    while (start <= (_buf_size - sizeof(struct Header)) && memcmp(_buf + start, ">>>", 3) != 0) {
        start++;
    }
    if (start >= (_buf_size - sizeof(struct Header))) {
        return 0;
    }

    struct Header *header = (struct Header*)&_buf[start];
    len = ((uint16_t)header->payload_len_h << 8) | header->payload_len_l;
    if (start + len > _buf_size)
        return 0; // we don't have a complete msg yet

    // cerr << "rtps parse. message len " << (int)len << " on: " << start << endl;

    // buffer should be big enough to hold a rtps packet
    if (len > buflen) {
        return -EMSGSIZE;
    }

    // Found a whole message, send it and remove from local _buf
    *topic_ID = header->topic_ID;
    *seq = header->seq;

    uint16_t crc = crc16(_buf + start + sizeof(struct Header), len);
    uint16_t read_crc = ((uint16_t)header->crc_h << 8) | header->crc_l;
    if (crc != read_crc)
        log_debug("CRC mismatch %u - %u (%u) %d", crc, read_crc, len, header->topic_ID);
    else
        memmove(buffer, _buf + start + sizeof(struct Header), len);

    memmove(_buf + start, _buf + start + sizeof(struct Header) + len,
            sizeof(_buf) - start - sizeof(struct Header) - len);
    _buf_size -= len + sizeof(struct Header);

    return len;
}

int UART_node::writeRTPStoUART(const uint8_t topic_ID, char buffer[], uint16_t length)
{
    static struct Header header {
        .marker = { '>', '>', '>' }
    };
    int ret = 0;
    static uint8_t seq = 0;

    if (m_uart_filestream == -1)
        return 2;

    // [>,>,>,topic_ID,seq,payload_length,CRCHigh,CRCLow,payload_start, ... ,payload_end]

    uint16_t crc = crc16((uint8_t *)buffer, length);

    header.topic_ID = topic_ID;
    header.seq = seq++;
    header.payload_len_h = (length >> 8) & 0xff;
    header.payload_len_l = length & 0xff;
    header.crc_h = (crc >> 8) & 0xff;
    header.crc_l = crc & 0xff;
    ret = write(m_uart_filestream, &header, sizeof(header));
    if (ret < 0 || ret != sizeof(header))
        goto err;
    std::cout << "Topic ID:" << topic_ID << std::endl;

    ret = write(m_uart_filestream, buffer, length);
    if (ret < 0 || (unsigned)ret != length)
        goto err;

    return length + sizeof(header);

err:
    int errsv = errno;
    if (ret == -1 && errno != EAGAIN)
        log_debug("Writing error '%d'\n", errno);

    return -errsv;
}

int UART_node::writeMavlinkToUART(char buffer[], uint16_t length)
{
    if (m_uart_filestream == -1)
        return -EINVAL;

    return write(m_uart_filestream, buffer, length);
}

uint16_t UART_node::crc16_byte(uint16_t crc, const uint8_t data)
{
    return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

uint16_t UART_node::crc16(uint8_t const *buffer, size_t len)
{
    uint16_t crc = 0;
    while (len--)
        crc = crc16_byte(crc, *buffer++);
    return crc;
}
