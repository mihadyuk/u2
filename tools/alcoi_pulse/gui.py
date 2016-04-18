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
from tkinter import messagebox

# create in and out communication channels
mavin  = mavutil.mavlink_connection("udpin:localhost:14551")
mavout = mavutil.mavlink_connection("udpout:localhost:14556")

RECV_TIMEOUT = 2.0
RECV_TRIES = 4
CHANNEL_NUMBER = 4
ENTRY_WIDTH = 12
LABEL_WIDTH = 3

def mavlink_wait_param_timeout(param_name): #{{{
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
#}}}
def mavlink_acqure_with_retry(param_name):#{{{
    retry = RECV_TRIES
    ret = None
    while retry > 0:
        retry -= 1
        ascii_name = bytearray(param_name, "ascii")
        mavout.param_fetch_one(ascii_name)
        ret = mavlink_wait_param_timeout(param_name)
        if ret is not None:
            return ret
#}}}
def mavlink_set_with_retry(param_name, param_value):#{{{
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
#}}}
def mavlink_alcoi_with_confirm(pid_num, width, strength):#{{{
    #define PARAM_PULSE_CHANNEL       param5
    #define PARAM_PULSE_WIDTH         param6
    #define PARAM_PULSE_STRENGTH      param7
    print("Pulse: channel", pid_num, width, strength)
    mavout.mav.command_long_send(mavout.target_system,
            0,
            mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            0, #param1
            0,
            0,
            0,
            pid_num,
            width,
            strength)

    # m = mavin.recv_match(type="COMMAND_ACK", blocking=True, timeout=RECV_TIMEOUT*2)
    # if m is None:
    #     messagebox.showerror(
    #         "Error",
    #         "Time is out."
    #     )
    # else:
    #     if m.result == mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
    #         messagebox.showerror(
    #             "Error",
    #             "TEMPORARILY_REJECTED"
    #         )
    #     elif m.result == mavlink.MAV_RESULT_DENIED:
    #         messagebox.showerror(
    #             "Error",
    #             "DENIED"
    #         )
    #     elif m.result == mavlink.MAV_RESULT_UNSUPPORTED:
    #         messagebox.showerror(
    #             "Error",
    #             "UNSUPPORTED"
    #         )
    #     elif m.result == mavlink.MAV_RESULT_FAILED:
    #         messagebox.showerror(
    #             "Error",
    #             "FAILED"
    #         )
#}}}

class AlcoiPulse(LabelFrame):#{{{
    def __init__(self, parent, channel_num):
        LabelFrame.__init__(self, parent, text="Alcoi")
        self.channel = channel_num
        self.var_w = DoubleVar()
        self.var_w.trace("w", self._changed_w)
        self.frame_w = Frame(self)
        self.frame_w.pack()
        self.entry_w = Entry(self.frame_w, width=ENTRY_WIDTH-5, textvariable=self.var_w)
        self.entry_w.pack(side = RIGHT)
        self.label_w = Label(self.frame_w, text="Width", width=LABEL_WIDTH+5)
        self.label_w.pack(side = LEFT)

        self.var_s = DoubleVar()
        self.var_s.trace("w", self._changed_s)
        self.frame_s = Frame(self)
        self.frame_s.pack()
        self.entry_s = Entry(self.frame_s, width=ENTRY_WIDTH-5, textvariable=self.var_s)
        self.entry_s.pack(side = RIGHT)
        self.label_s = Label(self.frame_s, text="Strength", width=LABEL_WIDTH+5)
        self.label_s.pack(side = LEFT)

        self.pulse_button = Button(self, text="Пыщь!", command=self._pulse)
        self.pulse_button.pack()

    def disable(self):
        self.entry_w.configure(state='disabled')
        self.entry_s.configure(state='disabled')
        self.pulse_button.configure(state='disabled')

    def enable(self):
        self.entry_w.configure(state='normal')
        self.entry_s.configure(state='normal')
        self.pulse_button.configure(state='normal')

    def _changed_w(self, a1, a2, a3):
        try:
            float(self.var_w.get())
        except ValueError:
            self.entry_w.configure(background="red")
            return
        self.entry_w.configure(background="white")

    def _changed_s(self, a1, a2, a3):
        try:
            float(self.var_s.get())
        except ValueError:
            self.entry_s.configure(background="red")
            return
        self.entry_s.configure(background="white")

    def _pulse(self):
        mavlink_alcoi_with_confirm(self.channel, self.var_w.get(), self.var_s.get())
#}}}
class PostProcMenu(Frame):#{{{
    def __init__(self, parent, guiname, param_name):
        Frame.__init__(self, parent)
        self.default_color = "light gray"
        self.param_name = param_name
        self.var = StringVar()
        self.option = OptionMenu(self, self.var, "none", "wrap_pi", "wrap_2pi")
        self.option.configure(background=self.default_color)
        self.option.pack(side = RIGHT)
        self.label = Label(self, text=guiname, width=LABEL_WIDTH)
        self.label.pack(side = LEFT)
        self.state = "UNINIT"

    def disable(self):
        self.option.configure(state='disabled')

    def enable(self):
        self.option.configure(state='normal')

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
        if self.state == "UNINIT":
            val = mavlink_acqure_with_retry(self.param_name)
            if val is not None:
                self._set_val(val)
                self.state = "GOT"
                self.var.trace("w", self._changed)

    def _changed(self, a1, a2, a3):
        self.state = "CHANGED"
        self.option.configure(background="green")

    def send(self):
        if self.state == "CHANGED":
            ret = mavlink_set_with_retry(self.param_name, self._decode_val())
            if None == ret:
                print("ERROR: unable to send", self.param_name)
            else:
                print("OK")
                self.state = "GOT"
                self.option.configure(background=self.default_color)
#}}}
class PIDSpinbox(Frame):#{{{

    def __init__(self, parent, guiname, param_name):
        Frame.__init__(self, parent)
        self.var = DoubleVar()
        self.var.set("???")
        self.param_name = param_name
        self.entry = Entry(self, width=ENTRY_WIDTH, textvariable=self.var)
        self.entry.pack(side = RIGHT)
        self.label = Label(self, text=guiname, width=LABEL_WIDTH)
        self.label.pack(side = LEFT)
        self.state = "UNINIT" # CHANGED, GOT

    def disable(self):
        self.entry.configure(state='disabled')

    def enable(self):
        self.entry.configure(state='normal')

    def acqure(self):
        if self.state == "UNINIT":
            val = mavlink_acqure_with_retry(self.param_name)
            if val is not None:
                self.var.set(round(val, 5))
                self.state = "GOT"
                self.entry.configure(background="white")
                self.var.trace("w", self._changed)

    def _changed(self, a1, a2, a3):
        try:
            float(self.var.get())
        except ValueError:
            self.entry.configure(background="red")
            return
        self.state = "CHANGED"
        self.entry.configure(background="green")

    def send(self):
        if "CHANGED" == self.state:
            ret = mavlink_set_with_retry(self.param_name, self.var.get())
            if None == ret:
                print("ERROR: unable to send", self.param_name)
            elif abs(self.var.get() - ret) > 0.0001:
                print("WARNING: sent: {} read back {}".format(self.var.get(), ret))
                self.var.set(round(ret, 5))
                self.state = "GOT"
                self.entry.configure(background="yellow")
            else:
                print("OK")
                self.state = "GOT"
                self.entry.configure(background="white")
#}}}
class PIDFrame(LabelFrame):#{{{

    def __init__(self, parent, pid_number):
        self.pid_number = pid_number
        paramname = ('PID_%02d_' % pid_number)
        guiname = ('PID #%02d' % pid_number)
        LabelFrame.__init__(self, parent, text=guiname)
        self.controls = {}

        self.name_list = ["P", "I", "D", "Min", "Max", "proc"]
        self.pid_control_frame = Frame(self)
        self.alcoi_pulse_frame = Frame(self)
        self.pid_control_frame.pack(side = LEFT)
        self.alcoi_pulse_frame.pack(side = RIGHT)

        for n in self.name_list:
            if "proc" == n:
                self.controls[n] = PostProcMenu(self.pid_control_frame, n, paramname + n)
            else:
                self.controls[n] = PIDSpinbox(self.pid_control_frame, n, paramname + n)
            self.controls[n].pack()

        self.alcoi = AlcoiPulse(self.alcoi_pulse_frame, pid_number)
        self.alcoi.pack()

    def disable(self):
        self.alcoi.disable()
        for key in self.controls:
            self.controls[key].disable()

    def enable(self, pid_number):
        if pid_number == self.pid_number:
            self.alcoi.enable()
            for key in self.controls:
                self.controls[key].enable()

    def acquire(self):
        for c in self.controls:
            self.controls[c].acqure()

    def send(self):
        for c in self.controls:
            self.controls[c].send()
#}}}
class ChannelFrame(LabelFrame):#{{{

    def __init__(self, parent, guiname, ch_number):
        LabelFrame.__init__(self, parent, text=guiname, background="grey")
        self.pid_list = [] # list for PIDs

        for i in range(0, 4):
            pid_number = ch_number*4 + i
            pid = PIDFrame(self, pid_number)
            pid.pack(side = LEFT)
            self.pid_list.append(pid)

    def disable(self):
        for p in self.pid_list:
            p.disable()

    def enable(self, pid_num):
        for p in self.pid_list:
            p.enable(pid_num)

    def acqure(self):
        for p in self.pid_list:
            p.acquire()

    def send(self):
        for p in self.pid_list:
            p.send()
#}}}
class PidEnableMenu(Frame):#{{{
    def __init__(self, parent, var):
        Frame.__init__(self, parent)
        self.var = var
        self.entry = Entry(self, width=ENTRY_WIDTH // 2, textvariable=self.var)
        self.entry.pack(side = RIGHT)
        self.label = Label(self, text="Select PID")
        self.label.pack(side = LEFT)
#}}}
class Gui(object):#{{{

    def __init__(self, parent):
        self.ch_list = [] # channel list
        self.connected = False
        self.pid_enabled_var = IntVar()
        self.pid_enabled_var.set(-1)

        for i in range(0, CHANNEL_NUMBER):
            ch_name = "CH_" + str(i)
            ch = ChannelFrame(parent, ch_name, i)
            ch.disable()
            ch.pack()
            self.ch_list.append(ch)

        self.getbutton = Button(parent, text="Get", command=self.acqure)
        self.sendbutton = Button(parent, text="Set", command=self.send)
        self.pid_enable_menu = PidEnableMenu(parent, self.pid_enabled_var)

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
                self.pid_enable_menu.pack(side = LEFT)
                self.pid_enabled_var.trace("w", self._changed_pid_menu)
                print("Success!")
            else:
                print("Connection time is out")
                messagebox.showerror(
                    "Error",
                    "Time is out."
                )

    def _changed_pid_menu(self, a1, a2, a3):
        pid_num = None
        for ch in self.ch_list:
            ch.disable()
        try:
            pid_num = self.pid_enabled_var.get()
        except ValueError:
            return
        for ch in self.ch_list:
            ch.enable(pid_num)

    def send(self):
        if self.connected:
            for ch in self.ch_list:
                ch.send()
        else:
            print("ERROR: you need to connect first")
#}}}

root = Tk()
# try to use native look from ttk
try:
    Style().theme_use('native')
except:
    pass

if __name__ == '__main__':
    gui = Gui(root)
    root.resizable(width=FALSE, height=FALSE)
    root.mainloop()


