#!/usr/bin/python

# This file is part of the MAVLink Router project
#
# Copyright (C) 2017  Intel Corporation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function
from threading import Thread
from time import sleep

import pymavlink.mavutil as mavutil
import sys
import time

if len(sys.argv) != 4:
    print("Usage: %s <ip:udp_port> <system-id> <target-system-id>" %
          (sys.argv[0]))
    print(
        "Send mavlink pings, using given <system-id> and <target-system-id>, "
        "to specified interface")
    quit()

mav = mavutil.mavlink_connection(
    'udpout:' + sys.argv[1], source_system=int(sys.argv[2]))


def pingloop():
    i = 0
    while (True):
        mav.mav.ping_send(int(time.time() * 1000), i, int(sys.argv[3]), 1)
        i = i + 1
        sleep(1)


pingthread = Thread(target=pingloop)
pingthread.daemon = True
pingthread.start()

while (True):
    msg = mav.recv_match(blocking=True)
    print("Message from %d: %s" % (msg.get_srcSystem(), msg))
