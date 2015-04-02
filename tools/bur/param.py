#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import argparse
import struct
import binascii
import multiprocessing

import writerpool

import pymavlink.dialects.v10.lapwing as mavlink
import pymavlink.mavutil as mavutil

mavutil.set_dialect("lapwing")

ser = mavutil.mavserial("/dev/ttyS0", 115200)
ser.target_system = 20
ser.param_fetch_all()

while True:
    m = ser.recv_msg()
    if (None != m) and ("PARAM_VALUE" == m.get_type()):
        print (m)

print (ser.params)
