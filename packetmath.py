#!/usr/bin/env python3
# packetmath.py
#
# Extra math functions to work with autopilot1.py program for zip_sim game
#
# Copyright 2021 Leah Pillsbury leahp@bu.edu

import numpy as np
# TODO: change to import zip_sim
import zip_sim_modif as zs
import math


# fwd velocity is constant
velocity = [30,0]

def checklidar(lidar_samples, prior_trees):
    # This function looks at each sample and labels non-zero lidar samples as
    # being maybe drop points or trees.
    # if a drop point can't be reached because of a tree obstruction, it will
    # not be included in the list of drop_points
    # function returns 2 lists of tuples, one for drop point distance/angles,
    # and another for trees.
    # when the zip is very close to the mirror, the lidar reading is also zero,
    # but at this point package should have already been dropped

    mayb_drop_point = []
    tree = []
    tree_diam = 6 # trees are defined to be 6 meters wide
    drop_diam = 1 # reflective mirror at drop point is 1 m wide
    for i in range (0,31):
        if (lidar_samples[i] != 0):
            max_num_drop_samples = math.ceil(angle_diam(drop_diam, lidar_samples[i]))
            max_num_tree_samples = math.ceil(angle_diam(tree_diam, lidar_samples[i]))

            k = i + 1
            count = 1
            # compare difference between samples above
            while(k > 0 and k <= 30 and abs(lidar_samples[i]-lidar_samples[k]) < 3):
                k +=1
                count += 1
            # compare difference between samples below
            k = i - 1
            while(k >= 0 and k < 30 and abs(lidar_samples[i]-lidar_samples[k]) < 3):
                k -=1
                count += 1

            # if a drop point is 20 m or more away, it is substantially different from lidar neighbors
            # count = 1 because neither loop was entered, it's a drop point
            # if it's the first or last element of the lidar samples it could also
            # be a tree edge, but that will be sorted out as move towards it
            angle = angleheading(i)
            if (lidar_samples[i] > 35):
                if (count == 1):
                    mayb_drop_point.append(((lidar_samples[i],angle)))
                else:
                    tree.append((lidar_samples[i],angle))
            elif (count <= max_num_drop_samples):
                # when zip gets close to a tree on the edge of the lidar range, it
                # can be confused with a drop point. Assume that if a sample is on
                # the edge and close, it's a tree (this is the best assumption
                # because if it was a drop point I already would have navigated
                # directly towards it. Missing a drop point is better than dropping
                # at a tree.)
                # if(lidar_samples[i] < 30 and i < 3 or i > 27):
                    # continue
                # as get in closer range, sometimes things that were known trees
                # get mislabeled as drop_points. try to avoid this
                # check to see if the same angle was listed as a tree angle before

                istree = False
                for pt in prior_trees:
                    if(pt[1] == angle):
                        istree = True
                        break
                if istree == True:
                    tree.append((lidar_samples[i],angle))
                else:
                    # only consider a close lidar a drop point if approaching it head on
                    # this could leave some out, but it's better than making a drop
                    # near a tree
                    if(abs(angle) <=5):
                        mayb_drop_point.append(((lidar_samples[i],angle)))
            else:
                tree.append((lidar_samples[i],angle))
    # if drop point and tree list are not empty then remove drop points that
    # cannot be accessed because trees are in the way.
    if(mayb_drop_point and tree):
        # if drop points are near trees then avoid them (take them out of the list)
            drop_points = remove_collision(mayb_drop_point,tree)
            return(drop_points, tree)
    # if tree list or mayb_drop_point is empty, it's fine, an empty list will be returned
    return(mayb_drop_point, tree)



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

def remove_collision(drops, trees):
    # if it's hard to get to a drop point without crashing, then better just avoid it
    # make a copy of drop so I can remove conflicts while iterating
    drop_copy = drops
    for d_index, d in enumerate(drops):
        for t_index, t in enumerate(trees):
            # create variables for parts of each tuples to keep track of them better
            d_dist = d[0]
            d_angle = d[1]
            t_dist = t[0]
            t_angle = t[1]
            # check how close angles are
            if d_angle > (t_angle + 1) and d_angle < (t_angle -1):
                # this is only a problem when tree comes first
                if d_dist > tree_dist:
                    drop_copy.remove(d)
    # drop_copy is list of tuples of angles, distances
    return drop_copy
    # now we should only have drop points that we theoretically can get to

def avoid_tree(wind_vector_x, wind_vector_y,trees):
    # avoid the closest tree by vearing to the opposite direction
    # this could be made more sophisticated
    # remember I don't know what is happening from outside of the field of view

    global velocity
    # find vector to closest tree
    nearest = min(trees)
    # the find_closest2 function is now half redundant
    x_near, y_near = find_closest2(trees)

    #[x,y] components of resultant velocity from forward movement and wind
    actual_velocity = current_velocity(wind_vector_x,wind_vector_y)

    # [magnitude, angle] components of resultant velocity from wind and fwd movement
    act_vel_polar = convert_to_polar(actual_velocity[0],actual_velocity[1])
    numtree_samples = len(trees)

    '''
    # if the nearest tree is pretty close, avoid it
    # this is wrong
    neardist = distance(x_near, y_near)
    seconddist = distance(actual_velocity[0],actual_velocity[1]) # distance can travel in 1 second
    if ( neardist < (2 * seconddist) ):
        if nearest[1] >= 0:
            y = 30 # totally arbitrary yes there is a less jerky way to do this
        else:
            y = -30
        return y

    # else look for clusters (could be several trees or multiple lidar from
    # same tree) and avoid them
    else:
    '''
    range_to_avoid = []
    for i in range(numtree_samples):
        for j in range(numtree_samples):
            if (i == j):
                continue
            else:
                # calculate the difference in angles between the trees
                diff_angle = abs(trees[i][1] - trees[j][1])
                if(diff_angle < 3):
                    for angle in range(trees[i][1], trees[j][1]):
                        range_to_avoid.append(angle)

    if (range_to_avoid):
        # how do I avoid the things I want to avoid?
        range_to_avoid.sort()
        desired_angle = act_vel_polar[1]
        for a in range_to_avoid:
            while(abs(a-desired_angle) < 2):
                # if the current direction is similar to the range to avoid, need to change course
                if desired_angle >= 0:
                    desired_angle += 1
                if desired_angle < 0:
                    desired_angle -= 1
        # now we have a desired angle way forward, convert that to desired_y
        desired_x, desired_y = convert_to_cartesian(act_vel_polar[0], desired_angle)
        return desired_y
    else:
        return 0

def find_closest2(objs):
    # this function receives magnitude/angle pairs of drop point or tree objects
    # and returns the x, y coordinates of the closest one in the set

    min_of_set = min(objs)
    x,y = convert_to_cartesian(min_of_set[0], min_of_set[1])
    return x,y

def current_velocity(wind_x, wind_y):
    global velocity
    actual_x = velocity[0] + wind_x
    actual_y = velocity[1] + wind_y
    return [actual_x,actual_y]


def target_velocity(wind_x,wind_y, target_x, target_y):
    # based on fixed forward velocity vector, changing wind vector, and x, y
    # vector components to target, need to compute desired target_velocity
    # note that x velocity is fixed, only flexible is y
    actual_v = current_velocity(wind_x,wind_y)
    desired_x = target_x - actual_v[0]
    desired_y = target_y - actual_v[1]
    return desired_y

def parse_telem(telem_bin):
    telem_form = ">Hhffb31B"
    # telem_vals = zs.TELEMETRY_STRUCT.unpack(telem_bin)
    # print(telem_vals)
    timestamp, recovery_x, wind_x, wind_y, recovery_y, *lidar_samples = zs.TELEMETRY_STRUCT.unpack(telem_bin)
    return timestamp, recovery_x, wind_x, wind_y, recovery_y, lidar_samples

def go_where(timestamp, recovery_x, wind_x, wind_y, recovery_y, lidar_samples, last_dropped, drop_pkg, prior_trees):
    # if recovery_distance is close, GO THERE EVEN IF ALL PACKAGES NOT DROPPED
    # if already dropped 10 packages, head to recovery center
    # avoid trees
    # find where the trees are and where delivery sites are that
    # drops and trees are lists of tuples with distance and angle

    drops, trees = checklidar(lidar_samples, prior_trees)
    # if the recovery distance is close, then head there, avoiding trees
    recovery_dist = distance(recovery_x, recovery_y)
    if (recovery_dist < 250):
        if(trees):
            desired_y = avoid_tree(wind_x, wind_y, trees)
        else:
            desired_y = target_velocity(wind_x, wind_y, recovery_x, recovery_y)
    # find the closest drop point if there are drop points:
    elif drops:
        drop_x, drop_y = find_closest2(drops)
        # the returned drop point will be drop point without a tree in the way
        # if the coast is clear to go to this point:
        desired_y = target_velocity(wind_x, wind_y, drop_x, drop_y)
        # decide whether the drop point is close enough to command a drop
        drop_pkg = dropnow(wind_x, wind_y, drop_x, drop_y, timestamp, last_dropped, drop_pkg)

    # if there is no course for a drop point without a tree in the way, check
    # to see if a tree should be avoided
    else:
        if(trees):
            desired_y = avoid_tree(wind_x, wind_y,trees)
        else:
            # if there are no trees, then just stay the straight course ie have
            # desired y be the opposite of the wind vector_y component
            desired_y = -wind_y

    # desired_y can only be in the parameters of what the game allows
    if (desired_y > 30):
        desired_y = 30
    elif(desired_y < -30):
        desired_y = -30
    return desired_y, drop_pkg, trees

def dropnow(wind_x, wind_y, drop_x, drop_y, timestamp, last_dropped, drop_pkg):
    vehicle_velocity = current_velocity(wind_x, wind_y)
    # if vehicle velocity vector and vector to drop point are roughly collinear
    # then see how close the drop point is and drop accordingly
    # don't need to be "exaclty" collinear because of floating point math and
    # wind affect on the package as it falls
    # check for collinearity by comparing ratio of x and y components (should be exactly
    # equal if vectors are exactly collinear)
    # print("vehicle velocity: ", vehicle_velocity)
    # print("drop x, y: ", drop_x, drop_y)
    #if (abs(vehicle_velocity[0]/float(drop_x) - vehicle_velocity[1]/float(drop_y)) < 0.3):
    vehicle_velocity = convert_to_polar(vehicle_velocity[0], vehicle_velocity[1])
    speed = vehicle_velocity[0] #* math.cos(math.radians(vehicle_velocity[1]))
    # using horizontal launch and free fall equations
    # ignoring whether the wind will affect the trajectory on the way down
    # Range = speed * time of flight and time of flight is defined as 0.5
    fallrange = speed * 0.5
    if(abs(fallrange-distance(drop_x, drop_y)) < 1.5):
        # edge case is that when there is a tree on the very edge, package is
        # being dropped
        # either this shouldn't be put in "maybe_drop point" or need to test
        # for that situation here

        # we don't want to drop 2 packages in same drop area, I figure that
        # if the vehicle is moving forward 30m/s and the drop zone is 10m
        # diameter, then 1 second later the vehicle wouldn't be in the same
        # drop zone. Obviously massive headwind and lateral motion back and
        # forth could make this assumption incorrect
        if(timestamp - last_dropped > 1000):
            drop_pkg = 1
    return drop_pkg
