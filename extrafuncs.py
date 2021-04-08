def angular_extent(dist, angle, s):
    # calculate the angular extent of a tree (or drop point)
    # s = r * theta, approximate s as diameter of the tree/drop point
    # s = 10 for drop point, s = 6 for tree
    theta = s/dist

    # the angular extent that we care about is the angle that the object "takes up"
    extent = [angle - (1/2)*theta, angle + (1/2)* theta]
    return extent

def find_closest(lidar_samples,indices):
    # given the lidar_samples and indices of potential object, return
    # the vector pointing to that object in cartesian form

    # make numpy array of distances to the object
    # numpy.take Takes elements from an array along an axis (none here)
    # this seems to implicitly convert a list to an array for this operation
    lidar_distances = np.take(lidar_samples, indices)
    min_dist = min(lidar_distances)

    # now I have a min _dist I don't know which angle it came from
    # find which angles... how to know which one to use? probably best to take
    # the average if there are several nearby
    # if there are 2 equal distances in opposite directions?
    indx = np.where(lidar_samples == min_dist)
    # for now just take the first index
    # what if there is also another lidar sample with the same min distance that isnt part of the object
    # where returns a tuple... I just want first array element
    # of the first part of the tuple
    print("lidar distances: ", lidar_distances)
    print("min_dist: ", min_dist)
    print("indx: ", indx)
    indx = indx[0][0]
    print("indx: ", indx)
    angle = angleheading(indx)
    print("angle: ", angle)
    x,y = convert_to_cartesian(angle, min_dist)
    print("x: ", x)
    print("y: ", y)
    return x,y

def make_fake_telem():
    # This function is just to test structure packing and won't be used when
    # there are real telemetry packets to send
    bform = ">Hhffb31B"
    mystruct = struct.Struct(bform)
    telem = mystruct.pack(100, 32, 3.0, 5.0, 4,   0,0,0,0,0,0,3,0,0,0, 44,0,0,0,0,0,0,0,0,0, 0,0,0,4,5,6,7,8,9,10,11)
    return telem

from checklidar

    # convert lidar_samples to numpy array for ease of use
    ls = np.array(lidar_samples)

    # compute difference between samples. big difference means drop location
    # several small or zero differences is a tree
    ld = np.diff(ls)
    # numpy.where(array) returns a tuple of indices that match the where()
    lidar_change = np.where(abs(ld) >= 20)
    '''
    # potential tree
    # this seems to work
    tree = np.where(ld <=3) and (ls != 0)
    tree = np.where(tree == True)
    tree = tree[0]
    '''
def remove_collision(drop_indices, drop_dists, tree_indices, tree_dists):
    # if it's hard to get to a drop point without crashing, then better just avoid it
    # make lists of drop angles and tree_ angles to correspond with drop_dists, tree_dists
    # inputs to this function: drop_dists and tree_dists are numpy arrays
    # drop_indices and tree_indices are lists
    drop_angles = angle_list(drop_indices)
    tree_angles = angle_list(tree_indices)

    # make a list of tuples with angle/distance for ttree and for drops
    drops = list(zip(drop_angles, drop_dists))
    trees = list(zip(tree_angles, tree_dists))
    drop_copy = drops

    for d in drops:
        for t in trees:
            # check how close angles are
            if d[0] > t[0] + 1 and d[0] < t[0] -1:
                # this is only a problem when tree comes first
                if d[1] > t[1]:
                    drop_copy.remove(d)
    # drop_copy is list of tuples of angles, distances
    return drop_copy, trees
    # now we should only have drop points that we theoretically can get to



def find_closest2(drop_points):
    # this function receives angle and magnitude pairs of potential drop
    # points and returns the x, y coordinates of the closest drop point

    # angles, distances = zip*(drop_points)
    #min_dist = min(distances)

    # this is a tuple with the angle and the distance of the closest drop point
    mindrop = min(drop_points, key = lambda t: t[1])
    x,y = convert_to_cartesian(mindrop[0], mindrop[1])
    return x,y
def angle_list(indices):
    # takes a list of indices of lidar samples and converts them to angles
    # angles are defined as in angleheading: straight ahead is 0 and to the
    # right is positive and left is negative
    angle_list = []
    for index in indices:
        angle = angleheading(index)
        angle_list.append(angle)

def make_mag_angle_tuple(indices, distances):
    # given a list of indices from the lidar_samples, and the distances that go
    # with them, make a tuple of magnitude and direction in the form of
    # angle from X, distance
    # inputs to this function: distances are numpy arrays
    # ndices are lists
    angles = angle_list(indices)
    vector_tuple = zip(distances, angles)
    return vector_tuple

def checksentpkt(lateral_airspeed, drop_pkg):
    command = zs.COMMAND_STRUCT.pack(float(lateral_airspeed), drop_pkg, bytes(3))
    #print("command to send back: ", command)
    #print("")
    #print("unpacked command: ", COMMAND_STRUCT.unpack(command))
