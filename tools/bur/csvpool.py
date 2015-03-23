#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import os
import csv
import sys

def open_csv(dirname, msgname, msg_dict):

    filename = dirname + '/' + msgname + '.csv'

    # open file for saving
    if sys.version_info >= (3,0,0):
        f = open(filename, 'w', newline='')
    else:
        f = open(filename, 'wb')

    fieldnames = []
    for name in msg_dict:
        fieldnames.append(name)

    writer = csv.DictWriter(f, fieldnames, delimiter=';')
    writer.writeheader()
    return writer


class CsvWorker(object):
    """ Description here """

    def __init__(self, dirname, msgname):
        self.msgname = msgname
        self.dirname = dirname
        self.writer = None

    def write(self, mav_msg, time_boot_ms, time_usec):
        d = mav_msg.to_dict()

        del d["mavpackettype"]

        if time_boot_ms != None:
            d["time_boot_ms"] = time_boot_ms
        if time_usec != None:
            d["time_usec"] = time_usec

        if self.writer is None :
            self.writer = open_csv(self.dirname, self.msgname, d)
            print(self.msgname)

        self.writer.writerow(d)


class CsvPool(object):
    """ Description here """

    def __init__(self, dirname=None):
        self.time_boot_ms = 0
        self.time_usec = 0

        if dirname is None:
            self.dirname = time.strftime('%Y-%m-%d~%H.%M.%S')
        else:
            self.dirname = dirname

        if not os.path.exists(self.dirname):
            os.makedirs(self.dirname)
        self.wdict = {}

    def post(self, mavmsg):
        posted = False
        boot = None
        utc = None

        if (mavmsg.get_type() == 'GRIFFON_MEASUREMENT'):
            self.time_boot_ms = mavmsg.time_boot_ms
            self.time_usec = mavmsg.time_usec

        if not hasattr(mavmsg, 'time_boot_ms'):
            boot = self.time_boot_ms
        else:
            boot = None

        if not hasattr(mavmsg, 'time_usec'):
            utc = self.time_usec
        else:
            utc = None

        name = mavmsg.get_type()
        try:
            self.wdict[name].write(mavmsg, boot, utc)
        except KeyError:
            self.wdict[name] = CsvWorker(self.dirname, name)
            self.wdict[name].write(mavmsg, boot, utc)


