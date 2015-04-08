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

    def __init__(self, parent, guiname, internalname):
        Frame.__init__(self, parent)

        self.FROM = 0.0
        self.TO = 5.0
        self.var = StringVar()
        self.internalname = internalname
        self.spinbox = Entry(self, width=8, textvariable=self.var)
        # self.spinbox = Spinbox(self, width=8, textvariable=self.var, increment=1.0/10000)
        self.spinbox.pack(side = RIGHT)
        self.label = Label(self, text=guiname, width=2)
        self.label.pack(side = LEFT)
        self.spinbox.bind('<FocusIn>', self._changed)
        self.changed = False

    def validate(self):
        d = 0.0
        try:
            d = float(self.var.get())
            if (d < self.FROM) or (d > self.TO):
                self.spinbox.configure(background="yellow")
                return False
        except:
            self.spinbox.configure(background="red")
            return False

        self.spinbox.configure(background="white")
        return True

    def set(self, cmd):
        if (cmd.name == self.internalname):
            self.var.set(round(cmd.var, 5))
            self.changed = False

    def _changed(self, event):
        print ("changed")
        self.changed = True


class Bypassbutton(Checkbutton):

    def __init__(self, parent, guiname, internalname):
        self.var = IntVar()
        Checkbutton.__init__(self, parent, variable=self.var, text=guiname, command=self._changed)
        self.internalname = internalname
        self.changed = False

    def validate(self):
        return True

    def _changed(self):
        self.changed = True

    def set(self, cmd):
        if (cmd.name == self.internalname):
            self.var.set(cmd.var)
            self.changed = False


class PIDFrame(LabelFrame):

    def __init__(self, parent, guiname, internalname):
        LabelFrame.__init__(self, parent, text=guiname)
        self.controls = {}

        self.name_list = ["P", "I", "D"]

        for n in self.name_list:
            self.controls[n] = PIDSpinbox(self, n, internalname + n)
            self.controls[n].pack()

        self.controls["B"] = Bypassbutton(self, "Bypass", internalname + "B")
        self.controls["B"].pack()

    def validate(self):
        ret = True
        for n in self.controls:
            if False == self.controls[n].validate():
                ret = False
        return ret

    def set(self, cmd):
        for c in self.controls:
            self.controls[c].set(cmd)


class ChannelFrame(LabelFrame):

    def __init__(self, parent, guiname, internalname):
        LabelFrame.__init__(self, parent, text=guiname)
        self.pid = {} # dictionary for PIDs
        self.name_list = ["h", "m", "l"]

        for n in self.name_list:
            self.pid[n] = PIDFrame(self, "High", internalname + n + "_")
            self.pid[n].pack(side = LEFT)

    def validate(self):
        ret = True
        for n in self.pid:
            if False == self.pid[n].validate():
                ret = False
        return ret

    def set(self, cmd):
        for n in self.name_list:
            self.pid[n].set(cmd)


class Gui(object):

    def __init__(self, parent, to_pnc, from_pnc):
        self.to_pnc = to_pnc
        self.from_pnc = from_pnc
        self.ch = {} # channel dictionary
        self.name_list = ["ail", "ele", "rud", "thr"]

        for n in self.name_list:
            self.ch[n] = ChannelFrame(parent, "Aileron",  "PID_" + n + "_")
            self.ch[n].pack()

        self.sendbutton = Button(parent, text="Send", command=self.send_all)
        self.sendbutton.pack(side = LEFT)

        self.connect = Button(parent, text="Write")
        self.connect.pack(side = LEFT)

    def validate(self):
        ret = True
        for n in self.name_list:
            if False == self.ch[n].validate():
                ret = False
        return ret

    def send_all(self):
        if self.validate():
            pass

    def set(self, cmd):
        for n in self.name_list:
            self.ch[n].set(cmd)

    def update(self, Event):
        try:
            cmd = self.from_pnc.get_nowait()
        except:
            cmd = None
        if type(cmd) is commworker.AcquiredParam:
            print (">>> GUI updater:", cmd.name, cmd.var)
            self.set(cmd)


root = Tk()
# try to use native look from ttk
try:
    Style().theme_use('native')
except:
    pass

if __name__ == '__main__':
    to_pnc = Queue(256)
    from_pnc = Queue(256)
    gui = Gui(root, to_pnc, from_pnc)
    root.bind('<<NewParam>>', gui.update)

    comm_worker = commworker.CommWorker(to_pnc, from_pnc, root, "/dev/ttyS0")
    comm_worker.start()

    root.mainloop()

    comm_worker.stop()
    comm_worker.join()


