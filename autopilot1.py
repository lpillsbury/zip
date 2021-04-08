#!/usr/bin/env python3
# autopilot1.py
# Autopilot API to go with zip_sim.py
#
# Copyright 2021 Leah Pillsbury leahp@bu.edu
#
# This program communicates with zip_sim, receiving telemetry packets and
# sending command packets.
# Command decisions are made within this program and could be made within
# other subprograms if that was simpler for code reuse.
# Functions for calculations for command decisions are stored in packetmath.py


import zip_sim as zs
import struct
import sys
from packetmath import *
import numpy as np

'''
Controlled in this autopilot program:
    1) vehicle speed
    2) vehicle heading
    3) when a package is dropped
    4) communication with zip_sim.py using sys.stdin and sys.stdout
       pipes that are defined in zip_sim.py
'''

# Constant Packet Sizes
# size of the telemetry struct packet to receive, should be 44 bytes
TP_SIZE = zs.TELEMETRY_STRUCT.size
# size of the command struct packet to send, should be 8 bytes
CP_SIZE = zs.COMMAND_STRUCT.size

# Other global variables

# stdin and stdout go straight to the zip_sim.py program
recd_telem = sys.stdin
send_cmd = sys.stdout
# record the timestamp of when the last package was dropped


def receivepkt ():
    # receive a packet that is the size of the telemetry packet in bytes
    telempkt = recd_telem.buffer.raw.read(TP_SIZE)
    return telempkt

def sendpkt (timestamp, lateral_airspeed, last_dropped, drop_pkg, number_dropped):
    # send a command packet that is the command packet size and contains
    # lateral_airspeed, drop_package_commanded, and 3 bytes of padding
    command = zs.COMMAND_STRUCT.pack(float(lateral_airspeed), drop_pkg, bytes(3))
    # I don't know how Python does it, but send_cmd.write() doesn't work
    send_cmd.buffer.raw.write(command)

    # if a package was dropped, increment the number dropped counter
    # reset the drop_pkg variable back to 0
    # record the current timestamp when drop_pkg was commanded so that
    # don't do that again immediately. (is there a way to record where the
    # vehicle is on drop?)
    if drop_pkg == 1:
        number_dropped +=1
        last_dropped = timestamp
        drop_pkg = 0

# when not debug put parse_telem here



def main():
    # timestamp of the last dropped package. Initialize to -1 when game starts
    last_dropped = -1000
    # number of packages already dropped. increment to 10 and
    # head to recovery site if all payloads are dropped
    number_dropped = 0
    # command of whether or not to drop the next package. 0 means don't drop, 1 drop
    drop_pkg = 0

    # create a loop to receive packet, decide what to do with the info and
    # send a command back every 1/60th of a second (game time not real time)
    while True: # what is the actual while condition here?
        telem_b = receivepkt()
        timestamp, recovery_x, wind_x, wind_y, recovery_y, lidar_samples = parse_telem(telem_b)
        # if the recovery distance is close, go there, else go to the next drop, avoiding trees
        desired_y = go_where(timestamp, recovery_x, wind_x, wind_y, recovery_y, lidar_samples, last_dropped, drop_pkg)
        sendpkt(timestamp, desired_y, last_dropped, drop_pkg, number_dropped)


if __name__=="__main__":
    main()
