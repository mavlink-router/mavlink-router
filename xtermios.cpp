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
#include "xtermios.h"

#include <string.h>
#include <termios.h>
#include <unistd.h>

int reset_uart(int fd)
{
    struct termios tc = {};
    /* See termios(3) */
    const cc_t default_cc[] = { 03, 034, 0177, 025, 04, 0, 0, 0, 021, 023, 032, 0,
                                022, 017, 027, 026, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0 };

    static_assert(sizeof(default_cc) == sizeof(tc.c_cc), "Unknown termios struct with different size");

    if (tcgetattr(fd, &tc) < 0) {
        return -1;
    }

    /* Put UART in known state: it's the equivalent of "sane" in stty */
    tc.c_cflag = CREAD;

    tc.c_iflag |= BRKINT | ICRNL | IMAXBEL;
    tc.c_iflag &= ~(INLCR | IGNCR | IUTF8 | IXOFF| IUCLC | IXANY);

    tc.c_oflag |= OPOST | ONLCR;
    tc.c_oflag &= ~(OLCUC | OCRNL | ONLRET | OFILL | OFDEL | NL0 | CR0 | TAB0 | BS0 | VT0 | FF0);

    tc.c_lflag |= ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE;
    tc.c_lflag &= ~(ECHONL | NOFLSH | XCASE | TOSTOP | ECHOPRT);

    /* special characters to their default values */
    memcpy(tc.c_cc, default_cc, sizeof(default_cc));

    if (tcsetattr(fd, TCSANOW, &tc) < 0) {
        return -1;
    }

    cfsetspeed(&tc, B1200);

    if (tcsetattr(fd, TCSANOW, &tc) < 0) {
        return -1;
    }

    return 0;
}
