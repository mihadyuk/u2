#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import csv
import sys


def open_csv(dirname, msgname, msg_dict):

    dirname += "/csv"
    if not os.path.exists(dirname):
        os.makedirs(dirname)

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

    def write(self, mavdict):

        if self.writer is None :
            self.writer = open_csv(self.dirname, self.msgname, mavdict)
            print(self.msgname)

        self.writer.writerow(mavdict)


