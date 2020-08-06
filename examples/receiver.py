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

import pymavlink.mavutil as mavutil
import sys
import time

if len(sys.argv) != 3:
    print("Usage: %s <ip:udp_port> <system-id>" % (sys.argv[0]))
    print("Receive mavlink heartbeats on specified interface. "
          "Respond with a ping message")
    quit()

srcSystem = int(sys.argv[2])
mav = mavutil.mavlink_connection(
    'udpin:' + sys.argv[1], source_system=srcSystem)

while (True):
    msg = mav.recv_match(blocking=True)
    print("Message from %d: %s" % (msg.get_srcSystem(), msg))
    if msg.target_system == 0:
        print("\tMessage sent to all")
    elif msg.target_system == srcSystem:
        print("\tMessage sent to me")
    else:
        print("\tMessage sent to other")
    mav.mav.ping_send(
        int(time.time() * 1000), msg.seq,
        msg.get_srcSystem(), msg.get_srcComponent())
