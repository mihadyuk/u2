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
hwcontrol= config.getboolean('Serial', 'hwcontrol')

print("trying to open", serport, "at", baudrate, "speed...")
ser = serial.Serial(serport, baudrate, timeout=0.5, rtscts=hwcontrol)
print("  success!")

class SerialReader(threading.Thread):#{{{
    """ Infinitely read data from serial port and write it to all registered ports """
    def __init__(self, dev):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = dev
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.portoutlist = []
        for port in config.items("SocketOut"):
            print("  adding", port[1], "port to destination list")
            self.portoutlist.append(config.getint("SocketOut", port[0]))

    def stop(self):
        self.__stop.set()

    def run(self):
        while True:
            if self.__stop.is_set():
                print("SerialProxy: reader exiting")
                return
            c = self.ser.read(bufsize)
            if len(c) != 0:
                for port in self.portoutlist:
                    self.sock.sendto(c, ('', port))
    #}}}
class SerialWriter(threading.Thread):#{{{
    def __init__(self, dev):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = dev
        self.socklist = []
        for port in config.items("SocketIn"):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.05)
            s.bind(('', int(port[1])))
            self.socklist.append(s)
            print("  binding writer to", port[1], "port")

    def stop(self):
        self.__stop.set()

    def run(self):
        cin = ''
        while True:
            if self.__stop.is_set():
                print("SerialProxy: writer exiting")
                for s in self.socklist:
                    s.close()
                return
            for s in self.socklist:
                try:
                    cin = s.recv(1024)
                except socket.timeout:
                    pass
                if len(cin) > 0:
                    self.ser.write(cin)
                    print(len(cin), "bytes accepeted from", s)
                    cin = ''
    #}}}
def main():#{{{
    print("starting serial reader thread")
    reader = SerialReader(ser)
    reader.start()
    print("starting serial writer thread")
    writer = SerialWriter(ser)
    writer.start()
    while True:
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nKeyboard Interrupt caught")
            reader.stop()
            writer.stop()
            reader.join()
            writer.join()
            print("closing serial port")
            ser.close()
            print("SerialProxy: stopped")
            return
#}}}

main()

