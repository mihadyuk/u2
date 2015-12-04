#!/usr/bin/python
# -*- coding: utf-8 -*-

import configparser
import socket
import time
import serial
from binascii import hexlify
import threading
import rtcmproxy

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
class SerialWriter(threading.Thread):#{{{
    def __init__(self, dev):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = dev
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        p = config.getint("SocketIn", "PORT_UDP_SERPROXY")
        self.sock.bind(("", p))
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
    rtcm = rtcmproxy.RtcmProxy(ser)
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

