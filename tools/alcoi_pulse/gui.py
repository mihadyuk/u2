#!/usr/bin/python
# -*- coding: utf-8 -*-

import commworker

from multiprocessing import Process, Queue, Event, freeze_support
from queue import Empty, Full # for exception catching

# from tkinter import ttk
# from tkinter import *
# from tkinter.ttk import *

from tkinter import *

param_recvd = {}
param_changed = {}


class PIDSpinbox(Frame):

    def __init__(self, parent, guiname, internalname, callback):
        Frame.__init__(self, parent)


        # valid percent substitutions (from the Tk entry man page)
        # %d = Type of action (1=insert, 0=delete, -1 for others)
        # %i = index of char string to be inserted/deleted, or -1
        # %P = value of the entry if the edit is allowed
        # %s = value of entry prior to editing
        # %S = the text string being inserted or deleted, if any
        # %v = the type of validation that is currently set
        # %V = the type of validation that triggered the callback
        #      (key, focusin, focusout, forced)
        # %W = the tk name of the widget
        vcmd = (parent.register(self.OnValidate),
                '%s', '%P')

        self.callback = callback
        self.from_ = 0.0
        self.to = 5.0
        self.var = DoubleVar()
        self.internalname = internalname
        self.spinbox = Entry(self,
                width=8,
                textvariable=self.var,
                validate="all", validatecommand=vcmd)
        # self.spinbox = Spinbox(self,
        #         increment=1.0/10000,
        #         width=8,
        #         from_=self.from_,
        #         to=self.to,
        #         command=self._cb,
        #         validate="all", validatecommand=vcmd)
        self.spinbox.pack(side = RIGHT)
        self.label = Label(self, text=guiname, width=2)
        self.label.pack(side = LEFT)

    def OnValidate(self, old, new):
        d = 0.0
        i = 0
        try:
            i = int(new)
            if (i < self.from_) or (i > self.to):
                return False
        except:
            print ("---- validation error:", new)
            return False

        try:
            d = float(new)
            if (d < self.from_) or (d > self.to):
                return False
        except:
            print ("---- validation error:", new)
            return False


        return True

    def _cb(self):
        self.callback(self.internalname, self.spinbox.get())


class Bypassbutton(Checkbutton):

    def __init__(self, parent, guiname, internalname, callback):
        self.var = IntVar()
        Checkbutton.__init__(self, parent, variable=self.var, text=guiname, command=self._cb)
        self.callback = callback
        self.internalname = internalname

    def update(self, cmd):
        return

    def _cb(self):
        self.callback(self.internalname, self.var.get())
        return


class PIDFrame(LabelFrame):

    def __init__(self, parent, guiname, internalname, callback):
        LabelFrame.__init__(self, parent, text=guiname)
        self.controls = {}

        self.controls["P"] = PIDSpinbox(self, "P", internalname + "P", callback)
        self.controls["P"].pack()

        self.controls["I"] = PIDSpinbox(self, "I", internalname + "I", callback)
        self.controls["I"].pack()

        self.controls["D"] = PIDSpinbox(self, "D", internalname + "D", callback)
        self.controls["D"].pack()

        self.controls["B"] = Bypassbutton(self, "Bypass", internalname + "B", callback)
        self.controls["B"].pack()

    def update(self, cmd):
        return


class ChannelFrame(LabelFrame):

    def __init__(self, parent, guiname, internalname, callback):
        LabelFrame.__init__(self, parent, text=guiname)
        self.pid = {} # dictionary for PIDs

        self.pid["h"] = PIDFrame(self, "High", internalname + "h_", callback)
        self.pid["m"] = PIDFrame(self, "Mid",  internalname + "m_", callback)
        self.pid["l"] = PIDFrame(self, "Low",  internalname + "l_", callback)

        self.pid["h"].pack(side = LEFT)
        self.pid["m"].pack(side = LEFT)
        self.pid["l"].pack(side = LEFT)

    def update(self, cmd):
        return


class Gui(object):

    def __init__(self, parent, paramq, commandq):
        self.paramq = paramq
        self.commandq = commandq
        self.ch = {} # channel dictionary

        self.ch["ail"] = ChannelFrame(parent, "Aileron",  "PID_ail_", self._cb)
        self.ch["ele"] = ChannelFrame(parent, "Elevator", "PID_ele_", self._cb)
        self.ch["rud"] = ChannelFrame(parent, "Rudder",   "PID_rud_", self._cb)
        self.ch["thr"] = ChannelFrame(parent, "Thrust",   "PID_thr_", self._cb)

        self.ch["ail"].pack()
        self.ch["ele"].pack()
        self.ch["rud"].pack()
        self.ch["thr"].pack()
        return

    def update(self, cmd):
        if type(cmd) is commworker.AcquiredParam:
            print (">>> GUI handler:", cmd.name)

    def _cb(self, name, value):
        print (">>> GUI main callback", name, value)
        param = commworker.AcquiredParam(name, value)
        self.paramq.put_nowait(param)
        return



root = Tk()
# try to use native look from ttk
try:
    Style().theme_use('native')
except:
    pass



if __name__ == '__main__':
    paramq = Queue(4)
    commandq = Queue(4)
    gui = Gui(root, paramq, commandq)

    comm_worker = commworker.CommWorker(paramq, commandq, gui.update, "/dev/ttyS0")
    comm_worker.start()

    root.mainloop()

    comm_worker.stop()
    comm_worker.join()


