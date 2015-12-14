#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket
import time
import serial
import threading

import pymavlink.dialects.v10.lapwing as mavlink

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

class RtcmProxy(threading.Thread):#{{{
    """ Inifinitely receive RTCM and send it to serial port """
    def __init__(self, dev):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = dev
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(0.5)
        self.sock.connect(("localhost", 40007))
        print("  connected to RTCM server")
        self.mav = mavlink.MAVLink(fifo())
        self.mav.srcSystem = 255 # прикинемся контрольным центром

    def stop(self):
        self.__stop.set()

    def mav_pack(self, len_, data):
        msg = mavlink.MAVLink_gps_inject_data_message(0, 0, len_, data)
        return msg.pack(self.mav)

    def mav_split(self, buf):
        BUF_SIZE = 110
        ret = []
        while len(buf) > BUF_SIZE:
            tmp = bytearray(buf[0:BUF_SIZE])
            buf = buf[BUF_SIZE:]
            ret.append(self.mav_pack(BUF_SIZE, tmp))
        if len(buf) > 0:
            tmp = bytearray(BUF_SIZE) # zero filled buffer
            i = 0
            for b in buf:
                tmp[i] = b
                i += 1
            ret.append(self.mav_pack(len(buf), tmp))
        return ret

    def run(self):
        cin = ''
        while True:
            if self.__stop.is_set():
                print("RtcmReceiver: exiting")
                self.sock.close()
                return
            try:
                cin = self.sock.recv(2048)
            except socket.timeout:
                pass
            if len(cin) > 0:
                rtcm = self.mav_split(cin)
                cin = ''
                for m in rtcm:
                    self.ser.write(m)
                    time.sleep(0.1)
    #}}}

