#!/usr/bin/python
# -*- coding: utf-8 -*-

import configparser
import socket
import time
import serial
from binascii import hexlify
import threading

import pymavlink.dialects.v10.lapwing as mavlink

# load settings from config file
config = configparser.SafeConfigParser()
config.read('config.cfg')

# serial port
bufsize  = config.getint('Serial', 'bufsize')
baudrate = config.getint('Serial', 'baudrate')
serport  = config.get('Serial', 'port')

print("trying to open", serport, "at", baudrate, "speed...")
ser = serial.Serial(serport, baudrate, timeout = 0.5)
print("  success!")

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

class SerialReader(threading.Thread):#{{{
    """ Infinitely read data from serial port and write it to all registered ports """
    def __init__(self, dev):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = dev
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.portlist = []
        for port in config.items("SocketOut"):
            print("  adding", port[1], "port to destination list")
            self.portlist.append(config.getint("SocketOut", port[0]))

    def stop(self):
        self.__stop.set()

    def run(self):
        while True:
            if self.__stop.is_set():
                print("SerialProxy: reader exiting")
                return
            c = self.ser.read(bufsize)
            if len(c) != 0:
                for port in self.portlist:
                    # self.sock.sendto(c, ("localhost", port))
                    self.sock.sendto(c, ("10.37.61.147", port))
    #}}}
class RtcmReceiver(threading.Thread):#{{{
    """ Inifinitely receive RTCM and send it to serial port """
    def __init__(self, dev):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = dev
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(0.5)
        self.sock.connect(("localhost", 40007))
        print("  connected to RTCM server")
        self.mav = mavlink.MAVLink(fifo())
        self.mav.srcSystem = 255 # прикинемся контрольным центром

    def stop(self):
        self.__stop.set()

    def mav_pack(self, len_, data):
        msg = mavlink.MAVLink_gps_inject_data_message(0, 0, len_, data)
        return msg.pack(self.mav)

    def mav_split(self, buf):
        BUF_SIZE = 110
        ret = []
        while len(buf) > BUF_SIZE:
            tmp = bytearray(buf[0:BUF_SIZE])
            buf = buf[BUF_SIZE:]
            ret.append(self.mav_pack(BUF_SIZE, tmp))
        if len(buf) > 0:
            tmp = bytearray(BUF_SIZE) # zero filled buffer
            i = 0
            for b in buf:
                tmp[i] = b
                i += 1
            ret.append(self.mav_pack(len(buf), tmp))
        print (len(ret))
        return ret

    def run(self):
        cin = ''
        while True:
            if self.__stop.is_set():
                print("RtcmReceiver: exiting")
                self.sock.close()
                return
            try:
                cin = self.sock.recv(2048)
            except socket.timeout:
                pass
            if len(cin) > 0:
                rtcm = self.mav_split(cin)
                cin = ''
                for m in rtcm:
                    self.ser.write(m)
                    time.sleep(0.1)
    #}}}
class SerialWriter(threading.Thread):#{{{
    def __init__(self, dev):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = dev
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        p = config.getint("SocketIn", "PORT_UDP_SERPROXY")
        self.sock.bind(("localhost", p))
        print("  binding writer to", p, "port")

    def stop(self):
        self.__stop.set()

    def run(self):
        cin = ''
        while True:
            if self.__stop.is_set():
                print("SerialProxy: writer exiting")
                self.sock.close()
                return
            try:
                cin = self.sock.recv(1024)
            except socket.timeout:
                pass
            if len(cin) > 0:
                self.ser.write(cin)
                cin = ''
    #}}}
def main():#{{{
    print("starting serial reader thread")
    reader = SerialReader(ser)
    reader.start()
    print("starting serial writer thread")
    writer = SerialWriter(ser)
    writer.start()
    print("starting RTCM receiver thread")
    rtcm = RtcmReceiver(ser)
    rtcm.start()
    while True:
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nKeyboard Interrupt caught")
            reader.stop()
            writer.stop()
            rtcm.stop()
            reader.join()
            writer.join()
            rtcm.join()
            print("closing serial port")
            ser.close()
            print("SerialProxy: stopped")
            return
#}}}

main()

