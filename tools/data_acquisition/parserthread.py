#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import csv
import os
import sys
import math
import time
import signal

from multiprocessing import Process, Queue, Event
try:
    from queue import Empty, Full # for exception catching
except ImportError:
    from Queue import Empty, Full # for exception catching

# import pymavlink.dialects.v10.griffon as mavlink
from pymavlink import mavutil

tprev = 0.0
def dbg_print(msg):
    #pass
    global tprev
    if msg._type == "RAW_IMU":
        dt = msg._timestamp - tprev
        print (dt)
        tprev = msg._timestamp

class ParserThread(Process):
    """ Class writes simulated data to navigation system and
    receives results """

    def __init__(self, device, baudrate, q):
        Process.__init__(self)
        self.device = device
        self.baudrate = baudrate
        self.q = q
        self.__stop = Event()

    def stop(self):
        self.__stop.set()

    def mav_dispatch(self, m):
        try:
            self.q.put_nowait(m)
        except Full:
            pass

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print("*** parser process started. Pid:", os.getpid())
        master = mavutil.mavlink_connection(self.device, self.baudrate, dialect="lapwing")

        while not self.__stop.is_set():
            m = master.recv_msg()
            if (m is not None):
                dbg_print(m)
                self.mav_dispatch(m)

        master.close()
        print("*** parser process stopped")

