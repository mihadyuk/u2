#!/usr/bin/python
# -*- coding: utf-8 -*-

from binascii import hexlify

SLIP_START = '\xC0'

# dictionary for unstuffing
dic = {'\xDC' : '\xC0',
       '\xDD' : '\xDB'}

# inverse dictionary for stuffing
dic_inv = {'\xC0' : '\xDC',
           '\xDB' : '\xDD'}

SLIP_MARK = '\xDB'

def unstuff(data):
    i = iter(data)
    while True:
        d = next(i)  # throws StopIteration on the end
        if d == SLIP_MARK:
            d2 = next(i)
            if d2 in dic:
                yield dic[d2]
            else:
                yield SLIP_MARK
                yield d2
        else:
            yield d

def stuff(data):
    i = iter(data)
    while True:
        d = next(i) # throws StopIteration on the end
        if d in dic_inv:
            yield SLIP_MARK
            yield dic_inv[d]
        else:
            yield d

def unframe(data):
    ret = ''
    for b in data:
        if b != SLIP_START:
            ret += b
    return ret

def frame(data):
    s = SLIP_START
    s += data
    s += SLIP_START
    return s
