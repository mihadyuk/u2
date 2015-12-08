#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket
import sys

import pymavlink.dialects.v10.lapwing as mavlink

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

# we will use a fifo as an encode/decode buffer
f = fifo()
mav = mavlink.MAVLink(f)
mav.srcSystem = 255 # прикинемся контрольным центром
msg = mav.heartbeat_encode(0, 0, 0, 0, 0, 0)
b = msg.pack(mav)
print (msg, b)

HOST = "localhost"
PORT = 40007
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

while True:
    data = s.recv(2048)
    test_array = bytearray(55)
    test_array[1] = 9
    print (len(test_array))
    msg = mavlink.MAVLink_rtcm_data_message(test_array, len(test_array))
    b = msg.pack(mav)
    print (b)
    print ('Received', len(data))

