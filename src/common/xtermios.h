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

/*
 * workaround not being able to include termios.h and asm/termbits.h at the
 * same time
 */
int reset_uart(int fd);

/*
 * FreeBSD compat shim. Linux's <asm/termbits.h> defines
 * struct termios2, TCGETS2, TCSETS2, BOTHER, CBAUD. FreeBSD has
 * plain POSIX struct termios with c_ispeed / c_ospeed. Mapping
 * termios2 to termios + TCGETS2/TCSETS2 to TIOCGETA/TIOCSETA lets
 * endpoint.cpp's set_speed, set_flow_control and open paths
 * compile unchanged. BOTHER and CBAUD become no-ops because
 * FreeBSD does not pack baud rate into c_cflag.
 *
 * termios2 is a #define rather than a typedef so that
 * "struct termios2 tc" declarations expand to "struct termios tc"
 * (C++ does not allow the "struct" keyword with a typedef alias).
 */
#ifndef __linux__
#include <termios.h>
#include <sys/ioctl.h>
#define termios2 termios
#ifndef TCGETS2
#define TCGETS2 TIOCGETA
#endif
#ifndef TCSETS2
#define TCSETS2 TIOCSETA
#endif
#ifndef BOTHER
#define BOTHER 0
#endif
#ifndef CBAUD
#define CBAUD 0
#endif
#endif /* __linux__ */

/* Linux-only termios output flags used by endpoint.cpp; map to
 * 0 so bitwise masks become no-ops on POSIX. */
#ifndef __linux__
#ifndef OLCUC
#define OLCUC 0
#endif
#ifndef OFILL
#define OFILL 0
#endif
#ifndef OFDEL
#define OFDEL 0
#endif
#ifndef ONLRET
#define ONLRET 0
#endif
#ifndef IUCLC
#define IUCLC 0
#endif
#ifndef IUTF8
#define IUTF8 0
#endif
#ifndef XCASE
#define XCASE 0
#endif
#ifndef ECHOPRT
#define ECHOPRT 0
#endif
#ifndef ECHOCTL
#define ECHOCTL 0
#endif
#ifndef ECHOKE
#define ECHOKE 0
#endif
/* Linux defines IPV6_ADD_MEMBERSHIP in <netinet/in.h>; POSIX /
 * BSD only define IPV6_JOIN_GROUP. They identify the same option. */
#ifndef IPV6_ADD_MEMBERSHIP
#define IPV6_ADD_MEMBERSHIP IPV6_JOIN_GROUP
#endif
#endif /* __linux__ */
