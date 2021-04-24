#!/usr/bin/env python3
# packetmath.py
#
# Extra math functions to work with variables in autopilot1.py program for zip_sim game
#
# Copyright 2021 Leah Pillsbury leahp@bu.edu
import math
import zip_sim_modif as zs

def distance(x,y):
    return math.sqrt(x**2+y**2)

def angleheading(index):
    # this function receives the index of a lidar sample and returns an angle
    # in degrees from positive x axis (note this looks like degrees N)
    # (0 to 90 degrees is to the "right" of the forward facing vehicle,
    # 0 to -90 is to the left
    angleX = 15 - index
    return angleX

def convert_to_cartesian(magnitude, angle):
    # convert a magnitude and angleX and vector to cartesian coordinates
    # Note: fwd is the posititive x axis, to the left is positive y
    # to the left is negative angle, and to the right is positive angle
    # (the way it would be in a degrees N reference frame)

    angleRad = math.radians(angle)
    x = magnitude * math.cos(angleRad)
    y = -magnitude * math.sin(angleRad)
    return x,y

def convert_to_polar(x,y):
    # Note: fwd is the posititive x axis, to the left is positive y
    # to the left is negative angle, and to the right is positive angle
    # (the way it would be in a degrees N reference frame)
    # angle sign is reversed from what you'd expect neg angle quadrant IV and
    # positive angle quadrant I
    magnitude = distance(x,y)
    angle = -math.atan(y/x)
    angle = math.degrees(angle)
    return [magnitude, angle]

def angle_diam(diameter, distance):
    # calculate the angular diameter of a circle (i.e. tree or reflector) with
    # given distance from the viewer
    # return angle in degrees
    # theta = 2 arctan(diameter/(2*distance))
    theta = 2 * math.atan(diameter/(2 * distance))
    return math.degrees(theta)

def current_velocity(wind_x, wind_y, my_velocity):
    # sets the vehicle's current velocity and wind adjusted velocity
    velocity_adjusted=[0.0,0.0]
    velocity_adjusted[0] = my_velocity[0] + wind_x
    velocity_adjusted[1] = my_velocity[1] + wind_y
    return velocity_adjusted

def parse_telem(telem_bin):
    telem_form = ">Hhffb31B"
    # telem_vals = zs.TELEMETRY_STRUCT.unpack(telem_bin)
    # print(telem_vals)
    timestamp, recovery_x, wind_x, wind_y, recovery_y, *lidar_samples = zs.TELEMETRY_STRUCT.unpack(telem_bin)
    return timestamp, recovery_x, wind_x, wind_y, recovery_y, lidar_samples
