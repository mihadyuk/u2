#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
ZetCode Tkinter tutorial

This script shows a simple window
on the screen.

author: Jan Bodnar
last modified: January 2011
website: www.zetcode.com
"""

# from tkinter import ttk
# from tkinter import *
# from tkinter.ttk import *

from tkinter import *


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

param_recvd = {}
param_changed = {}

class PIDSpinbox(Frame):

    def __init__(self, parent, var, name):
        Frame.__init__(self, parent)
        self.var = var
        self.name = name
        self.spinbox = Spinbox(self, textvariable=var, width=8, from_=0.0, to=10.0, command=self._callback)
        self.spinbox.pack(side = RIGHT)
        self.label = Label(self, text=name, width=2)
        self.label.pack(side = LEFT)

    def _callback(self):
        print (self.name, self.var.get())

class PIDFrame(LabelFrame):

    def __init__(self, parent, name, pid_dict):
        LabelFrame.__init__(self, parent, text=name)

        self.pspin = PIDSpinbox(self, pid_dict["p"], "P")
        self.pspin.pack()

        self.ispin = PIDSpinbox(self, pid_dict["i"], "I")
        self.ispin.pack()

        self.dspin = PIDSpinbox(self, pid_dict["d"], "D")
        self.dspin.pack()

        self.c = Checkbutton(self, text="Bypass", variable=pid_dict["b"])
        self.c.pack()


class PIDDict():

    def __init__(self):
        self.d = {}
        self.d["p"] = DoubleVar()
        self.d["i"] = DoubleVar()
        self.d["d"] = DoubleVar()
        self.d["b"] = IntVar()


class PIDChannelDict():

    def __init__(self):
        self.d = {}
        self.d["h"] = PIDDict()
        self.d["m"] = PIDDict()
        self.d["l"] = PIDDict()


class ChannelFrame(LabelFrame):

    def __init__(self, parent, name, channel_dict):
        LabelFrame.__init__(self, parent, text=name)

        h_pid = PIDFrame(self, "High", channel_dict.d["h"].d)
        m_pid = PIDFrame(self, "Mid",  channel_dict.d["m"].d)
        l_pid = PIDFrame(self, "Low",  channel_dict.d["l"].d)

        h_pid.pack(side = LEFT)
        m_pid.pack(side = LEFT)
        l_pid.pack(side = LEFT)



def main():

    root = Tk()

    # Style().theme_use('default')

    ch_ail_dict = PIDChannelDict()
    ch_ele_dict = PIDChannelDict()
    ch_rud_dict = PIDChannelDict()
    ch_thr_dict = PIDChannelDict()

    ch_ail = ChannelFrame(root, "Aileron",  ch_ail_dict)
    ch_ele = ChannelFrame(root, "Elevator", ch_ele_dict)
    ch_rud = ChannelFrame(root, "Rudder",   ch_rud_dict)
    ch_thr = ChannelFrame(root, "Thrust",   ch_thr_dict)

    ch_ail.pack()
    ch_ele.pack()
    ch_rud.pack()
    ch_thr.pack()

    root.mainloop()


if __name__ == '__main__':
    main()
