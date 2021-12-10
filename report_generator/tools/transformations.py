from math import *
import numpy as np
import math

direction_dict = {
    'X': 0,
    'Y': 1,
    'Z': 2
}

rot_matrix = np.eye(4, 4)


def rot_x(angle):
    """
    Creates a 4x4 rotation matrix around the x axis
    :param angle: Angle around x axis
    :return: Rotation matrix around x axis
    """
    x_rot_matrix = np.array([
        [1, 0, 0, 0],
        [0, cos(angle), sin(angle), 0],
        [0, sin(angle), cos(angle), 0],
        [0, 0, 0, 1]
    ])
    return x_rot_matrix


def rot_y(angle):
    """
    Creates a 4x4 rotation matrix around the y axis
    :param angle: Angle around y axis
    :return: Rotation matrix around y axis
    """
    y_rot_matrix = np.array([
        [cos(angle), 0, sin(angle), 0],
        [0, 1, 0, 0],
        [-sin(angle), 0, cos(angle), 0],
        [0, 0, 0, 1]
    ])
    return y_rot_matrix


def rot_z(angle):
    """
    Creates a 4x4 rotation matrix around the xz axis
    :param angle: Angle around z axis
    :return: Rotation matrix around z axis
    """
    z_rot_matrix = np.array([
        [cos(angle), sin(angle), 0, 0],
        [sin(angle), cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return z_rot_matrix


def rot_xyz(x_angle, y_angle, z_angle):
    """
    Creates a rotation matrix for given angles around all axis
    :param x_angle: Angle around x axis
    :param y_angle: Angle around y axis
    :param z_angle: Angle around z axis
    :return: 4x4 Rotation matrix
    """
    xyz_rot_matrix = np.matmul(np.matmul(rot_x(x_angle), rot_y(y_angle)), rot_z(z_angle))
    return xyz_rot_matrix


def get_furthest_point(trajectory, direction):
    """
    Finds the furthest point of a trajectory in a given direction.
    Needed to calculate the angle to rotate the map and trajectories by.
    :param trajectory: Matrix of trajectory points
    :param direction: Axis direction
    :return: A vector of the furthest point [x, y, z]
    """
    furthest_distance = 0
    for i in range(len(trajectory)):
        point = trajectory[i]
        if fabs(point[direction]) > furthest_distance:
            furthest_point = point
            furthest_distance = fabs(furthest_point[direction])

    return furthest_point


def get_angles(trajectory):
    '''
    Gets the angle offsets between the furthest points on the map and the origin in the X and Z direction\n
    :returns: [roll, pitch, yaw] -> angles by which to rotate
    '''
    furthest_z_point = get_furthest_point(trajectory, direction_dict['Z'])
    furthest_x_point = get_furthest_point(trajectory, direction_dict['X'])
    print('furthest x point:')
    print(furthest_x_point)
    print('furthest z point:')
    print(furthest_z_point)

    '''
    Can only rotate around one axis at a time because of the way we get the angle of rotation
    Returns only the wider angle
    '''
    if fabs(furthest_z_point[direction_dict['Y']]) >= fabs(furthest_x_point[direction_dict['Y']]):
        pitch = atan2(furthest_z_point[direction_dict['Y']], furthest_z_point[direction_dict['Z']])
        '''
        If the point is in the 2nd quadrant of the YZ plane we need to add 180 degrees or PI to get the correct angle
        from the atan2 function
        If it is in the 4th quadrant we need to add 360 degrees or 2PI
        '''
        if furthest_z_point[direction_dict['Z']] < 0 < furthest_z_point[direction_dict['Y']]:
            pitch += math.pi
        elif furthest_z_point[direction_dict['Y']] < 0 < furthest_z_point[direction_dict['Z']]:
            pitch += 2 * math.pi
        roll = 0.0
    else:
        roll = atan2(furthest_x_point[direction_dict['Y']], furthest_x_point[direction_dict['X']]) + math.pi
        '''
        If the point is in the 2nd quadrant of the YZ plane we need to add 180 degrees or PI to get the correct angle
        from the atan2 function
        If it is in the 4th quadrant we need to add 360 degrees or 2PI
        '''
        if furthest_x_point[direction_dict['X']] < 0 < furthest_x_point[direction_dict['Y']]:
            roll += math.pi
        elif furthest_x_point[direction_dict['Y']] < 0 < furthest_x_point[direction_dict['X']]:
            roll += 2 * math.pi
        pitch = 0.0
    yaw = 0.0

    return roll, pitch, yaw


def rotate_map_and_trajectory(mapping_trajectory, map_points):
    """
    Since the camera doesn't know it's initial orientation it's possible that the trajectory roll or pitch will be
    offset, so we need to rotate the map and trajectory so that the trajectory is in the XZ plane.\n
    Rotates both the trajectory and the map around the origin (0, 0, 0).\n
    Only rotates the map around a single axis which has the biggest vertical offset.\n
    Assumes that the mapping was done on a flat plane.\n
    :param mapping_trajectory: Mapping trajectory
    :param map_points: Map point cloud
    :returns: Rotated trajectory and map
    """
    global rot_matrix
    roll, pitch, yaw = get_angles(mapping_trajectory)
    rot_matrix = rot_xyz(-pitch, yaw, roll)

    # rotates the trajectory
    for i in range(len(mapping_trajectory)):
        mapping_trajectory[i, :] = np.matmul(mapping_trajectory[i, :], rot_matrix)

    #  rotates the map
    for i in range(len(map_points)):
        map_points[i, :] = np.matmul(map_points[i, :], rot_matrix)

    return mapping_trajectory, map_points


def rotate_trajectory(trajectory):
    """
    Rotates each trajectory point by the global rotation matrix that is calculated in rotate_point_cloud()
    :param trajectory: Numpy array of trajectory points
    :return: Rotated trajectory
    """
    global rot_matrix

    for i in range(len(trajectory)):
        trajectory[i, :] = np.matmul(trajectory[i, :], rot_matrix)

    return trajectory
