#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import serial
import os
import sys
import time
import struct
import random
from binascii import hexlify


parser = argparse.ArgumentParser(description='Description will be here.')

parser.add_argument('-d', '--device', type=str, default='/dev/ttyUSB0',
                   help='Serial communication device')
parser.add_argument('-b', '--baudrate', type=int, default=57600,
                    help='Baudrate')
args = parser.parse_args()

ser = serial.Serial(args.device, args.baudrate, rtscts=True)
i = 0
try:
    while True:
        # ser.write(struct.pack('d', random.random()))
        ser.write(struct.pack('128s', os.urandom(128)))
        i += 1
        print(i)

except KeyboardInterrupt:
    exit()


