#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import argparse
import struct
import binascii
import multiprocessing

import csvpool

import pymavlink.dialects.v10.lapwing as mavlink
import pymavlink.mavutil as mavutil

parser = argparse.ArgumentParser()
parser.add_argument('-f', '--infile', type=str, required=True,
                   help='Input file')
args = parser.parse_args()

f = mavutil.mavlogfile(args.infile, robust_parsing=True, notimestamps=True, use_native=False)
total_bytes = 0

# while True:
#     msg = f.recv_msg()
#     if None == msg:
#         break;
#     else:
#         total_bytes += len(msg._msgbuf)
#         print (msg._type)
# print (total_bytes)
# exit(0)

csvpool = csvpool.CsvPool()

msgcnt = 0
bytescnt = 0
maverrorcnt = 0
indexerrorcnt = 0

while (True):
    try:
        m2 = f.recv_msg()
        if (None != m2):
            msgcnt += 1
            bytescnt += len(m2._msgbuf)
            csvpool.post(m2)
            if (m2.get_type() == 'BAD_DATA'):
                maverrorcnt += 1
                print (bytescnt)
        else:
            break
    except KeyboardInterrupt:
        pass

print("-----------------------------------------")
print("messages:", msgcnt)
print("total bytes:", bytescnt)
print("index errors:", indexerrorcnt)
print("mavlink errors:", maverrorcnt)

