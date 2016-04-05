#!/usr/bin/python
# -*- coding: utf-8 -*-

import struct
import binascii

import pymavlink.dialects.v10.lapwing as mavlink
import pymavlink.mavutil as mavutil
mavutil.set_dialect("lapwing")

# from tkinter import ttk
# from tkinter import *
# from tkinter.ttk import *

from tkinter import *

# create in and out communication channels
mavin  = mavutil.mavlink_connection("udpin:localhost:14551")
mavout = mavutil.mavlink_connection("udpout:localhost:14556")

RECV_TIMEOUT = 2.0
RECV_TRIES = 4
CHANNEL_NUMBER = 1



def mavlink_wait_param_timeout(param_name):
    ret = None
    print("Trying to get:", param_name)
    m = mavin.recv_match(type="PARAM_VALUE", blocking=True, timeout=RECV_TIMEOUT)
    if m is None:
        print ("Time is out. Retrying:")
    else:
        st = m.param_id.decode("utf-8")
        st = st[0:len(param_name)]
        if st == param_name:
            if m.param_type == 9 or m.param_type == 10:
                ret = float(m.param_value)
            else:
                b = struct.pack('<f', m.param_value)
                i = struct.unpack('<I', b)
                ret = i[0]
    return ret


def mavlink_acqure_with_retry(param_name):
    retry = RECV_TRIES
    ret = None
    while retry > 0:
        retry -= 1
        ascii_name = bytearray(param_name, "ascii")
        mavout.param_fetch_one(ascii_name)
        ret = mavlink_wait_param_timeout(param_name)
        if ret is not None:
            return ret


def mavlink_set_with_retry(param_name, param_value):
    retry = RECV_TRIES
    check = None
    if "proc" == param_name[-4:]:
        param_type = 5
    else:
        param_type = 9

    while retry > 0:
        retry -= 1
        ascii_name = bytearray(param_name, "ascii")
        mavout.param_set_send(ascii_name, param_value, param_type)
        print("Trying to send:", param_name)
        check = mavlink_wait_param_timeout(param_name)
        if check is not None:
            return check

    print("ERROR: unable to send", self.paramname)
    return None



class PostProcMenu(Frame):
    def __init__(self, parent, guiname, param_name):
        Frame.__init__(self, parent)
        self.param_name = param_name
        self.var = StringVar()
        self.option = OptionMenu(parent, self.var, "none", "wrap_pi", "wrap_2pi")
        self.option.pack(side = RIGHT)
        self.label = Label(self, text=guiname, width=3)
        self.label.pack(side = LEFT)
        self.state = "UNINIT"

    def _set_val(self, intval):
        if intval == 0:
            self.var.set("none")
        elif intval == 1:
            self.var.set("wrap_pi")
        elif intval == 2:
            self.var.set("wrap_2pi")
        else:
            raise("ERROR: unexpected postprocessing function number")

    def _decode_val(self):
        if self.var.get() == "none":
            return 0
        elif self.var.get() == "wrap_pi":
            return 1
        elif self.var.get() == "wrap_2pi":
            return 2
        else:
            raise("ERROR: unexpected menu value")

    def acqure(self):
        val = mavlink_acqure_with_retry(self.param_name)
        if val is not None:
            self._set_val(val)
            self.state = "GOT"
            self.var.trace("w", self._changed)

    def _changed(self, a1, a2, a3):
        self.state = "CHANGED"

    def send(self):
        if self.state == "CHANGED":
            ret = mavlink_set_with_retry(self.param_name, self._decode_val())
            if None == ret:
                print("ERROR: unable to send", self.param_name)
            else:
                print("OK")
                self.state = "GOT"


class PIDSpinbox(Frame):

    def __init__(self, parent, guiname, param_name):
        Frame.__init__(self, parent)
        self.var = StringVar()
        self.var.set("??????")
        self.param_name = param_name
        self.spinbox = Entry(self, width=10, textvariable=self.var)
        self.spinbox.pack(side = RIGHT)
        self.label = Label(self, text=guiname, width=3)
        self.label.pack(side = LEFT)
        self.spinbox.bind('<FocusIn>', self._changed)
        self.state = "UNINIT" # CHANGED, GOT

    def acqure(self):
        val = mavlink_acqure_with_retry(self.param_name)
        if val is not None:
            self.var.set(round(val, 5))
            self.state = "GOT"
            self.spinbox.configure(background="white")
        else:
            self.var.set("timeout")

    def _changed(self, event):
        print(self.var.get())
        self.state = "CHANGED"
        self.spinbox.configure(background="green")

    def send(self):
        if "CHANGED" == self.state:
            ret = mavlink_set_with_retry(self.param_name, float(self.var.get()))
            if None == ret:
                print("ERROR: unable to send", self.param_name)
            elif float(self.var.get()) != ret:
                self.var.set(round(ret, 5))
                self.state = "GOT"
                self.spinbox.configure(background="yellow")
                print("WARNING: sent: %s read back", self.var.get(), ret)
            else:
                print("OK")
                self.state = "GOT"
                self.spinbox.configure(background="white")


class PIDFrame(LabelFrame):

    def __init__(self, parent, guiname, internalname):
        LabelFrame.__init__(self, parent, text=guiname)
        self.controls = {}

        self.name_list = ["P", "I", "D", "Min", "Max", "proc"]

        for n in self.name_list:
            if "proc" == n:
                self.controls[n] = PostProcMenu(self, n, internalname + n)
            else:
                self.controls[n] = PIDSpinbox(self, n, internalname + n)
            self.controls[n].pack()

    def acquire(self):
        for c in self.controls:
            self.controls[c].acqure()

    def send(self):
        for c in self.controls:
            self.controls[c].send()


class ChannelFrame(LabelFrame):

    def __init__(self, parent, guiname, ch_number):
        LabelFrame.__init__(self, parent, text=guiname)
        self.pid_list = [] # list for PIDs

        for i in range(0, 4):
            pid_name = ('PID_%02d_' % (ch_number*4+i))
            pid = PIDFrame(self, "High", pid_name)
            pid.pack(side = LEFT)
            self.pid_list.append(pid)

    def acqure(self):
        for p in self.pid_list:
            p.acquire()

    def send(self):
        for p in self.pid_list:
            p.send()


class Gui(object):

    def __init__(self, parent):
        self.ch_list = [] # channel list
        self.connected = False

        for i in range(0, CHANNEL_NUMBER):
            ch_name = "CH_" + str(i)
            ch = ChannelFrame(parent, ch_name, i)
            ch.pack()
            self.ch_list.append(ch)

        self.getbutton = Button(parent, text="Get", command=self.acqure)
        self.sendbutton = Button(parent, text="Set", command=self.send)
        self.writeconnect = Button(parent, text="Write", command=self.write)

        self.getbutton.pack(side = LEFT)

    def acqure(self):
        if self.connected:
            for ch in self.ch_list:
                ch.acqure()
        else:
            print ("Waiting heartbeat...")
            m = mavin.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            if m is not None:
                print(m)
                mavout.target_system = m.get_srcSystem()
                self.connected = True
                self.acqure() # recursive call to avoid copypasta
                self.sendbutton.pack(side = LEFT)
                self.writeconnect.pack(side = LEFT)
                print("Success!")
            else:
                print("Connection time is out")

    def send(self):
        if self.connected:
            for ch in self.ch_list:
                ch.send()
        else:
            print("ERROR: you need to connect first")

    def write(self):
        if self.connected:
            print("Write stub!")
        else:
            print("ERROR: you need to connect first")


root = Tk()
# try to use native look from ttk
try:
    Style().theme_use('native')
except:
    pass

if __name__ == '__main__':
    gui = Gui(root)
    root.mainloop()


