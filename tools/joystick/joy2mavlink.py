#!/usr/bin/python
# -*- coding: utf-8 -*-

import socket
import pygame

import pymavlink.dialects.v10.lapwing as mavlink

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

pygame.init()

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
JOYSTICK_UDP_PORT = 14556

mav = mavlink.MAVLink(fifo())
mav.srcSystem = 255 # прикинемся контрольным центром

msgbuf = None

# -------- Main Program Loop -----------
while done == False:
    btns = 0
    thrust = 0.0
    rudder = 0.0

    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something

        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        print("Joystick name: {}".format(name) )

        # Get thrust and break first
        # mix 2 shifts in single channels
        thr =  (joystick.get_axis(5) + 1) / 2
        brk = -(joystick.get_axis(2) + 1) / 2
        thrust = thr + brk
        print("Thrust value: {:>6.3f}".format(thrust))

        rudder = joystick.get_axis(0)
        print("Thrust value: {:>6.3f}".format(rudder))

        # now collect all buttons
        btns = 0
        for i in range(joystick.get_numbuttons()):
            btns |= joystick.get_button(i) << i

        # pack acquired data and throw it to socket
        msg = mavlink.MAVLink_manual_control_message(
                target = 20,
                x = 32767,
                y = 32767,
                z = round(thrust*1000),
                r = round(rudder*1000),
                buttons = btns)
        msgbuf = msg.pack(mav)

    # Limit to 20 frames per second
    clock.tick(25)
    if msgbuf:
        sock.sendto(msgbuf, ('', JOYSTICK_UDP_PORT))

# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()

