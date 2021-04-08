#!/usr/bin/env python3
# packetmath.py
#
# Extra math functions to work with autopilot1.py program for zip_sim game
#
# Copyright 2021 Leah Pillsbury leahp@bu.edu

import numpy as np
import zip_sim as zs
import math

# fwd velocity is constant
velocity = [30,0]

def checklidar(lidar_samples):
    # I've noticed that delivery points usually don't have trees next to them.
    # Is that always true?
    # It seems like the next delivery point usually only has 1-3 lidar values in
    # that range and nearby values are 0 or more than 10 different
    # Trees often have 8-12 similar values nearby. (if a tree got pruned this might not work)
    #
    # Based on the assumptions above, this function returns the distance and
    # degrees from the zip for the next package drop.
    #
    # what does it do if it finds a tree?

    mayb_drop_point = []
    tree = []
    # make 2 lists: one of tuples of trees with distance/angles and one of drop_points
    # if a point is more than 10 different from its neighbors, it's probably a drop point
    # 10 is a somewhat arbitrary choice, but it seems to work
    # if a point is non zero and near neighbors, it's probably a tree... unless it is
    # quite close, and then it can be a drop point
    for i in range (0,31):
        if (lidar_samples[i] != 0):
            # deal with first and last sample separately because they have one neighbor
            # not two
            if (i == 0 or i == 30):
                # in this case where the first or last sample is a lot bigger than the next,
                # we might have the edge of a tree, or a drop point. assume it's a
                # drop point... if not that will be obvious as we approach
                if (i == 0):
                    if(abs(lidar_samples[i]-lidar_samples[i+1]) >= 10):
                        d_val = (lidar_samples[i], angleheading(i))
                        mayb_drop_point.append(d_val)
                    else:
                        continue
                if (i == 30):
                    if(abs(lidar_samples[i]-lidar_samples[i-1]) >= 10):
                        d_val = (lidar_samples[i], angleheading(i))
                        mayb_drop_point.append(d_val)
                    else:
                        continue

            # check for drop points
            elif (abs(lidar_samples[i]-lidar_samples[i-1]) >= 10 and abs(lidar_samples[i]-lidar_samples[i+1]) >= 10):
                d_val = (lidar_samples[i], angleheading(i))
                mayb_drop_point.append(d_val)

            # for faraway points, a drop point lidar sample would be different from
            # its neighbors. When approaching a drop point, as many as 3 lidar
            # samples can reflect off the same drop point. This seems to happen
            # when the zip is about 20m away.
            elif(lidar_samples[i] <= 20):
                mayb_drop_point.append((lidar_samples[i],angleheading(i)))

            # If 2 points are close together, probably part of the same tree
            # I'm not sure what the case should be if the difference in samples is
            # between 3 and 10... these are arbitrary values I chose based on observation
            elif(abs(lidar_samples[i]-lidar_samples[i-1]) <= 3):
                # if the ith element is a tree, i - 1 is also
                # if the i +1 element is also a tree, this will create duplicates
                tree.append((lidar_samples[i-1],angleheading(i-1)))
                tree.append((lidar_samples[i],angleheading(i)))
                if(i == 29):
                    if(abs(lidar_samples[i]-lidar_samples[i+1]) <= 3):
                        tree.append((lidar_samples[i+1],angleheading(i)))

    # get rid of duplicates in tree list (there should be a better way to not add duplicates in teh first place)
    tree = list(set(tree))
    # print("I think I am a tree: ", tree)
    lidar_samples = np.array(lidar_samples)
    # mayb_drop_point is a list. only proceed if list is not empty
    if(mayb_drop_point):
        # if drop points are near trees then avoid them (take them out of the list)
        if(tree):
            drop_points = remove_collision(mayb_drop_point,tree)
            # print("I think I am a tree: ", tree)
            # print("Deconflicted drop points: ", drop_points)
            return(drop_points, tree)
    # if tree list or mayb_drop_point is empty, it's fine, an empty list will be returned
    # is there any reason why this should be an empty list of tuples?
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

    # if the nearest tree is pretty close, avoid it
    neardist = distance(x_near, y_near)
    seconddist = distance(velocity[0],velocity[1]) # distance can travel in 1 second
    if ( neardist < (2 * seconddist) ):
        if nearest[0] >= 0:
            y = 20 # totally arbitrary yes there is a less jerky way to do this
        else:
            y = -20
        return y

    # else look for clusters (could be several trees or multiple lidar from
    # same tree) and avoid them
    else:
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

def go_where(timestamp, recovery_x, wind_x, wind_y, recovery_y, lidar_samples, last_dropped, drop_pkg):
    # if recovery_distance is close, GO THERE EVEN IF ALL PACKAGES NOT DROPPED
    # if already dropped 10 packages, head to recovery center
    # avoid trees
    # find where the trees are and where delivery sites are that
    # drops and trees are lists of tuples with distance and angle

    drops, trees = checklidar(lidar_samples)
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
        dropnow(wind_x, wind_y, drop_x, drop_y, timestamp, last_dropped, drop_pkg)

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
    return desired_y

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
        # we don't want to drop 2 packages in same drop area, I figure that
        # if the vehicle is moving forward 30m/s and the drop zone is 10m
        # diameter, then 1 second later the vehicle wouldn't be in the same
        # drop zone. Obviously massive headwind and lateral motion back and
        # forth could make this assumption incorrect
        if(timestamp - last_dropped > 1000):
            drop_pkg = 1
