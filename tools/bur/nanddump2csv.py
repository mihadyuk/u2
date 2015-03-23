#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import argparse
import struct
import binascii
import multiprocessing

import csvpool

import pymavlink.dialects.v10.griffon as mavlink

# create a mavlink instance, which will do IO on file object 'f'
mav = mavlink.MAVLink(None)

parser = argparse.ArgumentParser()
parser.add_argument('-f', '--infile', type=str, required=True,
                   help='Input file')
args = parser.parse_args()


class BinDump(object):
    def __init__(self, filename, pagesize):
        self.f = open(filename)
        self.pagesize = pagesize

    def __iter__(self):
        return self

    def __next__(self):
        page = self.f.read(self.pagesize)
        if len(page) != 0:
            # print struct.unpack("<q", page[0:8])
            # drop page id and unneeded timestamp
            return page[16:]
        else:
            raise StopIteration("End of file")


class MessageDump(object):
    def __init__(self, page):
        self.page = page
        self.tip  = 0

    def __iter__(self):
        return self

    def __next__(self):
        HEADERLEN = 2
        MAVLINK_MIN_LEN = 8
        if len(self.page) >= HEADERLEN:
            mid = struct.unpack("<H", self.page[0:HEADERLEN])
            msglen = mid[0]
            self.page = self.page[HEADERLEN:]
            if msglen > 263 or len(self.page) < HEADERLEN + MAVLINK_MIN_LEN:
                raise StopIteration("End of page")
            else:
                msg = self.page[:msglen]
                self.page = self.page[msglen:]
                return msg
        else:
            raise StopIteration("End of page")


PAGE_SIZE = 2048
pagecnt = 0
msgcnt = 0
bytescnt = 0
maverrorcnt = 0
indexerrorcnt = 0
bindump = BinDump(args.infile, PAGE_SIZE)
csvpool = csvpool.CsvPool()

try:
    for page in bindump:
        pagecnt += 1
        msgdump = MessageDump(page)
        for msg in msgdump:
            msgcnt += 1
            bytescnt += len(msg)
            try:
                m2 = mav.decode(msg)
                csvpool.post(m2)
            except mavlink.MAVError:
                maverrorcnt += 1
            except IndexError:
                indexerrorcnt += 1
except KeyboardInterrupt:
    pass

print("-----------------------------------------")
print("pages:", pagecnt)
print("messages:", msgcnt)
print("total bytes:", PAGE_SIZE * pagecnt)
print("payload bytes:", bytescnt)
print("index errors:", indexerrorcnt)
print("mavlink errors:", maverrorcnt)


