#!/usr/bin/python
# -*- coding: utf-8 -*-

import configparser
import socket
import time
import serial
import threading
import rtcmproxy

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

class SerialToQgc(threading.Thread):#{{{
    def __init__(self, ser, tcpconn):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = ser
        self.conn = tcpconn

    def stop(self):
        self.__stop.set()

    def run(self):
        while True:
            if self.__stop.is_set():
                print("SerialToQgc: terminating")
                return

            data = self.ser.read(bufsize)
            try:
                if data:
                    sent = self.conn.sendall(data)
                else:
                    sent = self.conn.sendall(bytearray(1)) # keep alive
            except BrokenPipeError:
                print ("SerialToQgc: connection closed")
                return;
    #}}}
class QgcToSerial(threading.Thread):#{{{
    def __init__(self, ser, tcpconn):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = ser
        self.conn = tcpconn

    def stop(self):
        self.__stop.set()

    def run(self):
        while True:
            if self.__stop.is_set():
                print("QgcToSerial: terminating")
                return

            data = self.conn.recv(1024)
            if not data:
                print("QgcToSerial: connection closed")
                return
            else:
                self.ser.write(data)
    #}}}
class ConnListener(threading.Thread):#{{{
    def __init__(self, ser):
        threading.Thread.__init__(self)
        self.__stop = threading.Event()
        self.ser = ser

    def stop(self):
        self.__stop.set()

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('', 5760))
        sock.settimeout(0.5)
        sock.listen(5)
        conn = None
        addr = None
        print("Starting ConnListener")

        while True:
            if self.__stop.is_set():
                print ("ConnListener: terminating")
                return

            try:
                (conn, addr) = sock.accept()
            except socket.timeout:
                pass

            if conn:
                print ("Connected by", addr)
                q2s = QgcToSerial(ser, conn)
                s2q = SerialToQgc(ser, conn)
                q2s.start()
                s2q.start()
                q2s.join()
                s2q.join()

                print("--- socket closed")
                conn.close()
                conn = None
                addr = None
    #}}}

listener = ConnListener(ser)
listener.start()
rtcm = rtcmproxy.RtcmProxy(ser)
rtcm.start()

while True:
    try:
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt caught")
        listener.stop()
        rtcm.stop()
        listener.join()
        rtcm.join()
        ser.close()
        exit()

