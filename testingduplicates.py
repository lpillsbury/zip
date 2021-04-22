import numpy as np
import math
import packetmath as pm

def current_velocity(wind_x, wind_y, last_desired_y, my_velocity, velocity_adjusted):
    # sets the vehicle's current velocity and wind adjusted velocity
    my_velocity[1] = last_desired_y
    velocity_adjusted[0] = my_velocity[0] + wind_x
    velocity_adjusted[1] = my_velocity[1] + wind_y
    return my_velocity, velocity_adjusted

# TODO fix this
def target_velocity(wind_x,wind_y, desired_angle, my_velocity):
    # given the current wind speed, the desired angle of travel,
    # and the current velocity (global)
    # compute the y component of the new desired velocity vector
    # note: the x component never changes
    # vel_y = proportionality constant * (magnitude to travel/size timestep)
    #         * sin(desired_angle) - (wind vector dot velocity)/(magnitude velocity)
    # set magnitude to travel to 1m since with no wind and no lateral airspeed,
    # would be traveling 0.5 m/timestep

    timestep = 1/60 # size of timestep in seconds
    magnitude_v = pm.distance(my_velocity[0],my_velocity[1]) * timestep
    p = 20 # experimentally determined proportionality constant

    vel_y = ((1 * p / timestep) * math.sin(math.radians(desired_angle))
        - ((wind_x * my_velocity[0] + wind_y * my_velocity[1])/magnitude_v))


    return vel_y

def go_where(timestamp, recovery_x, wind_x, wind_y, recovery_y, lidar_samples, last_dropped, drop_pkg, prior_trees, my_velocity, velocity_adjusted):
    # if recovery_distance is close, GO THERE EVEN IF ALL PACKAGES NOT DROPPED
    # if already dropped 10 packages, head to recovery center
    # avoid trees
    # find where the trees are and where delivery sites are that
    # drops and trees are lists of tuples with distance and angle
    # these are only deconflicted drops that don't have a tree in the way
    drops, trees = checklidar(lidar_samples, prior_trees)
    # if the recovery distance is close, then head there, avoiding trees
    recovery_dist = pm.distance(recovery_x, recovery_y)
    if (recovery_dist < 250):
        if(trees):
            desired_y = avoid_tree(wind_x, wind_y, trees)
        else:
            mag, theta = pm.convert_to_polar(recovery_x, recovery_y)
            desired_y = pm.target_velocity(wind_x, wind_y, theta, my_velocity)
    # find the closest drop point if there are drop points:
    elif drops:
        drop_pt = min(drops)
        # the returned drop point will be drop point without a tree in the way
        # if the coast is clear to go to this point:
        desired_y = target_velocity(wind_x, wind_y, drop_pt[1], my_velocity)
        # decide whether the drop point is close enough to command a drop
        drop_pkg = dropnow(wind_x, wind_y, drop_pt, timestamp, last_dropped, drop_pkg, velocity_adjusted)

    # if there is no course for a drop point without a tree in the way, check
    # to see if a tree should be avoided
    else:
        if(trees):
            desired_y = avoid_tree(wind_x, wind_y,trees, my_velocity)
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

# TODO fix velocity based on global
def dropnow(wind_x, wind_y, drop_pt, timestamp, last_dropped, drop_pkg, velocity_adjusted):

    # if vehicle velocity vector and vector to drop point are roughly collinear
    # then see how close the drop point is and drop accordingly
    # don't need to be "exaclty" collinear because of floating point math and
    # wind affect on the package as it falls
    # check for collinearity by comparing ratio of x and y components (should be exactly
    # equal if vectors are exactly collinear)
    # print("vehicle velocity: ", vehicle_velocity)
    # print("drop x, y: ", drop_x, drop_y)
    #if (abs(vehicle_velocity[0]/float(drop_x) - vehicle_velocity[1]/float(drop_y)) < 0.3):
    drop_x, drop_y = pm.convert_to_cartesian(drop_pt[0], drop_pt[1])
    vehicle_velocity = pm.convert_to_polar(velocity_adjusted[0], velocity_adjusted[1])
    speed = vehicle_velocity[0] #* math.cos(math.radians(vehicle_velocity[1]))
    # using horizontal launch and free fall equations
    # ignoring whether the wind will affect the trajectory on the way down
    # Range = speed * time of flight and time of flight is defined as 0.5
    fallrange = speed * 0.5
    if(abs(fallrange-pm.distance(drop_x, drop_y)) < 1.5):
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
            max_num_drop_samples = math.ceil(pm.angle_diam(drop_diam, lidar_samples[i]))
            max_num_tree_samples = math.ceil(pm.angle_diam(tree_diam, lidar_samples[i]))

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
            angle = pm.angleheading(i)
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

def avoid_tree(wind_vector_x, wind_vector_y,trees, my_velocity):
    # avoid trees by choosing the widest path between them.
    # this function only gets called when there are trees
    # when there is no path, then head away from the 30 degree field of view
    # [magnitude, angle] components of resultant velocity from wind and fwd movement
    numtree_samples = len(trees)

    # make a list of angles where trees are:
    # only include trees 150 m or closer
    tree_angles = [x[1] for x in trees if x[0]<150]

    #take differences of angles between tree elements. a gap is only useful
    # for travel if it is 3 degrees or wider
    # travel to the widest gap
    angle_gaps = abs(np.diff(tree_angles))
    if max(angle_gaps > 2):
        # index of the largest gap
        angle_gap_ind = np.argmax(angle_gaps)
        # angle endpoints of the largest gap
        gap_max_angle = tree_angles[angle_gap_ind]
        gap_min_angle = tree_angles[angle_gap_ind + 1]
        desired_angle = math.ceil((gap_max_angle - gap_min_angle)/2) + gap_min_angle
    # if there is no gap big enough to travel through between -15 and 15 degrees,
    # default to desired_angle = -40 degrees (if this requires a bigger lateral
    # velocity than allowed, it will be cut off later)
    else:
        desired_angle = -40
    desired_y = target_velocity(wind_vector_x, wind_vector_y, desired_angle, my_velocity)
    return desired_y
