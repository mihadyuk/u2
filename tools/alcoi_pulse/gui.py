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

        self.callback = callback
        self.FROM = 0.0
        self.TO = 5.0
        self.var = DoubleVar()
        self.internalname = internalname
        self.spinbox = Entry(self, width=8, textvariable=self.var)
        self.spinbox.pack(side = RIGHT)
        self.label = Label(self, text=guiname, width=2)
        self.label.pack(side = LEFT)

    def validate(self):
        d = 0.0
        try:
            d = float(self.var.get())
            if (d < self.FROM) or (d > self.TO):
                self.spinbox.configure(background="yellow")
                print ("1:", d)
                return False
        except:
            self.spinbox.configure(background="red")
            print ("2:", d)
            return False

        self.spinbox.configure(background="white")
        return True


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

        self.name_list = ["P", "I", "D"]

        for n in self.name_list:
            self.controls[n] = PIDSpinbox(self, n, internalname + n, callback)
            self.controls[n].pack()

        self.controls["B"] = Bypassbutton(self, "Bypass", internalname + "B", callback)
        self.controls["B"].pack()

    def validate(self):
        ret = True
        for n in self.name_list:
            if False == self.controls[n].validate():
                ret = False
        return ret

    def update(self, cmd):
        return


class ChannelFrame(LabelFrame):

    def __init__(self, parent, guiname, internalname, callback):
        LabelFrame.__init__(self, parent, text=guiname)
        self.pid = {} # dictionary for PIDs
        self.name_list = ["h", "m", "l"]

        for n in self.name_list:
            self.pid[n] = PIDFrame(self, "High", internalname + n + "_", callback)
            self.pid[n].pack(side = LEFT)

    def validate(self):
        ret = True
        for n in self.name_list:
            if False == self.pid[n].validate():
                ret = False
        return ret

    def update(self, cmd):
        return


class Gui(object):

    def __init__(self, parent, paramq, commandq):
        self.paramq = paramq
        self.commandq = commandq
        self.ch = {} # channel dictionary
        self.name_list = ["ail", "ele", "rud", "thr"]

        for n in self.name_list:
            self.ch[n] = ChannelFrame(parent, "Aileron",  "PID_" + n + "_", self._cb)
            self.ch[n].pack()

        self.b = Button(parent, text="Send", command=self.validate)
        self.b.pack()

    def validate(self):
        ret = True
        for n in self.name_list:
            if False == self.ch[n].validate():
                ret = False
        return ret

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


