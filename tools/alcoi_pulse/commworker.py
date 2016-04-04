#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import argparse
import struct
import binascii
import multiprocessing
import time

from multiprocessing import Process, Queue, Event, freeze_support
from threading import Thread
from queue import Empty, Full # for exception catching

import pymavlink.dialects.v10.lapwing as mavlink
import pymavlink.mavutil as mavutil

mavutil.set_dialect("lapwing")

TOTAL_PIDS = 16

class AcquiredParam(object):
    def __init__(self, name, var):
        self.name = name
        self.var = var


class CommWorker(Thread):

    def __init__(self, to_pnc, from_pnc, gui, udpin, udpout):
        Thread.__init__(self)
        self.to_pnc = to_pnc
        self.from_pnc = from_pnc
        self.gui = gui
        self.udpin  = udpin
        self.udpout = udpout
        self.__stop = Event()
        self.param_list = []
        for i in range(0, TOTAL_PIDS):
            s = ('PID_%02d_' % i)
            self.param_list.append(s + 'P')
            self.param_list.append(s + 'I')
            self.param_list.append(s + 'D')
            self.param_list.append(s + 'Min')
            self.param_list.append(s + 'Max')
            self.param_list.append(s + 'proc')

    def stop(self):
        self.__stop.set()


    def run(self):
        print("Communication worker started")

        mavin  = mavutil.mavlink_connection(self.udpin)
        mavout = mavutil.mavlink_connection(self.udpout)
        recv = None

        # wait heartbeat
        m = mavin.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if m is not None:
            self.from_pnc.put_nowait(m)
            self.gui.event_generate('<<NewParam>>', when='tail')
            mavout.target_system = m.get_srcSystem()
        else:
            self.gui.update("Connection time is out")
            return

        # acquire PIDs settings
        self._recv_param_all(mavin, mavout)

        # main working loop
        while not (self.__stop.is_set()):
            try:
                recv = self.to_pnc.get_nowait()
            except Empty:
                pass
            time.sleep(0.02)

        # that's all
        print("Communication worker stopped")


    def _recv_param_value(self, name, timeout, retry, mavin, mavout):
        while retry > 0:
            mavout.param_fetch_one(bytearray(name, "ascii"))
            print("Trying to get:", name)
            m = mavin.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
            if m is None:
                retry -= 1
                print ("Time is out. Retrying:", retry)
                continue
            st = m.param_id.decode("utf-8")
            st = st[0:len(name)]
            if st == name:
                if m.param_type == 9 or m.param_type == 10:
                    return float(m.param_value)
                else:
                    b = struct.pack('<f', m.param_value)
                    i = struct.unpack('<I', b)
                    return i[0]
            else:
                retry -= 1
                print ("Name is incorrect. Expected: %s Got: %s", name, st)
        return None


    def _recv_param_all(self, mavin, mavout):
        for name in self.param_list:
            if self.__stop.is_set():
                return
            pv = self._recv_param_value(name, 2, 5, mavin, mavout)
            if (pv is not None):
                self.from_pnc.put_nowait(AcquiredParam(name, pv))
                try:
                    self.gui.event_generate('<<NewParam>>', when='tail')
                except:
                    self.__stop.set()
                    return


    def _acqure(self):
        return

    def _save(self, param):
        return

    def _write_rom(self, param):
        return

