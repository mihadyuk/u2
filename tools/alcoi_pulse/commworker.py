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
from queue import Empty, Full # for exception catching

import pymavlink.dialects.v10.lapwing as mavlink
import pymavlink.mavutil as mavutil

mavutil.set_dialect("lapwing")

param_list = [
    "PID_ail_h_P",
    "PID_ail_h_I",
    "PID_ail_h_D",
    "PID_ail_h_B",
    "PID_ail_m_P",
    "PID_ail_m_I",
    "PID_ail_m_D",
    "PID_ail_m_B",
    "PID_ail_l_P",
    "PID_ail_l_I",
    "PID_ail_l_D",
    "PID_ail_l_B",
    "PID_ele_h_P",
    "PID_ele_h_I",
    "PID_ele_h_D",
    "PID_ele_h_B",
    "PID_ele_m_P",
    "PID_ele_m_I",
    "PID_ele_m_D",
    "PID_ele_m_B",
    "PID_ele_l_P",
    "PID_ele_l_I",
    "PID_ele_l_D",
    "PID_ele_l_B",
    "PID_rud_h_P",
    "PID_rud_h_I",
    "PID_rud_h_D",
    "PID_rud_h_B",
    "PID_rud_m_P",
    "PID_rud_m_I",
    "PID_rud_m_D",
    "PID_rud_m_B",
    "PID_rud_l_P",
    "PID_rud_l_I",
    "PID_rud_l_D",
    "PID_rud_l_B",
    "PID_thr_h_P",
    "PID_thr_h_I",
    "PID_thr_h_D",
    "PID_thr_h_B",
    "PID_thr_m_P",
    "PID_thr_m_I",
    "PID_thr_m_D",
    "PID_thr_m_B",
    "PID_thr_l_P",
    "PID_thr_l_I",
    "PID_thr_l_D",
    "PID_thr_l_B"
]


class AcquiredParam(object):
    def __init__(self, name, val):
        self.name = name
        self.val = val


class CommWorker(Process):

    def __init__(self, paramq, commandq, guihandler, device):
        Process.__init__(self)
        self.paramq = paramq
        self.commandq = commandq
        self.guihandler = guihandler
        self.device = device
        self.__stop = Event()


    def stop(self):
        self.__stop.set()


    def run(self):
        print("Communication worker started")

        mav = mavutil.mavserial(self.device, 115200)
        recv = None

        # wait heartbeat
        m = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if m is not None:
            self.guihandler(m)
            mav.target_system = m.get_srcSystem()
        else:
            self.guihandler("Connection time is out")
            return

        # acquire PIDs settings
        self._recv_param_all(mav)

        # main working loop
        while not (self.__stop.is_set()):
            try:
                recv = self.commandq.get_nowait()
            except Empty:
                pass
            try:
                recv = self.paramq.get_nowait()
            except Empty:
                pass
            time.sleep(0.02)

        # that's all
        print("Communication worker stopped")


    def _recv_param_value(self, name, timeout, retry, mav):
        while retry > 0:
            mav.param_fetch_one(bytearray(name, "ascii"))
            m = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
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
                    return int(m.param_value)
            else:
                retry -= 1
                print ("Time is out. Retrying:", retry)
        return None


    def _recv_param_all(self, mav):
        for name in param_list:
            if self.__stop.is_set():
                return
            pv = self._recv_param_value(name, 2, 5, mav)
            if (pv is not None):
                self.guihandler(AcquiredParam(name, pv))


    def _acqure(self):
        return

    def _save(self, param):
        return

    def _write_rom(self, param):
        return

