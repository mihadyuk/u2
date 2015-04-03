#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import argparse
import struct
import binascii
import multiprocessing
import time

import pymavlink.dialects.v10.lapwing as mavlink
import pymavlink.mavutil as mavutil

mavutil.set_dialect("lapwing")

ser = mavutil.mavserial("/dev/ttyS0", 115200)

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


m = ser.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
if m is not None:
    print (m)
    ser.target_system = m.get_srcSystem()
else:
    print ("Time is out")
    exit()


def recv_param_value(name, timeout, retry):
    while retry > 0:
        ser.param_fetch_one(bytearray(name, "ascii"))
        m = ser.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
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


for name in param_list:
    print (name)
    pv = recv_param_value(name, 2, 5)
    print (pv)


class Paramproxy(object):
    def __init__(self, callback, inqueue):
        self.callback = callback
        self.inqueue = inqueue

    def acqure(self):
        return

    def save(self, param):
        return

    def write_rom(self, param)
