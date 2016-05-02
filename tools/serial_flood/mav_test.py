#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import serial
import os
import sys
import time
from binascii import hexlify

import pymavlink.dialects.v10.lapwing as mavlink
from pymavlink import mavutil


parser = argparse.ArgumentParser(description='Description will be here.')

parser.add_argument('-d', '--device', type=str, default='/dev/ttyUSB0',
                   help='Serial communication device')
parser.add_argument('-b', '--baudrate', type=int, default=115200,
                    help='Baudrate')
args = parser.parse_args()

master = mavutil.mavlink_connection(args.device, args.baudrate)
mav = mavlink.MAVLink(master)
mav.srcSystem = 255 # прикинемся контрольным центром

decimator = 100
i = 0
try:
    while True:
        mav.sys_status_send(0,0,0,0,0,0,0,0, 0, 0, 192, 0, mavlink.MAV_STATE_ACTIVE)

        i += 1
        print(i)

except KeyboardInterrupt:
    exit()


