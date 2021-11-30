#!/usr/bin/python

# This file is part of the MAVLink Router project
#
# Copyright (C) 2021  Lucas De Marchi <lucas.de.marchi@gmail.com>
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

mav = mavutil.mavlink_connection('udpin:' + sys.argv[1],
                                 source_system=1,
                                 source_component=10)
mav.wait_heartbeat()

while True:
    mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    print(".", end="")
    sys.stdout.flush()
    time.sleep(1)
