#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import csvworker
import sys


def mav2dict(mav_msg, time_boot_ms, time_usec):
    d = mav_msg.to_dict()

    del d["mavpackettype"]

    if time_boot_ms != None:
        d["time_boot_ms"] = time_boot_ms
    if time_usec != None:
        d["time_usec"] = time_usec

    return d


class WriterPool(object):
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
        else:
            raise NameError('Target directory already exists')

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
        mav_dict = mav2dict(mavmsg, boot, utc)
        try:
            self.wdict[name].write(mav_dict)
        except KeyError:
            self.wdict[name] = csvworker.CsvWorker(self.dirname, name)
            self.wdict[name].write(mav_dict)


