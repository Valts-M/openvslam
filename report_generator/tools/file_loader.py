import numpy as np
import msgpack

def toCameraCoord(pose_mat):
    """
        Convert the pose of lidar coordinate to camera coordinate
    """
    R_C2L = np.array([[0, 0, 1, 0],
                      [-1, 0, 0, 0],
                      [0, -1, 0, 0],
                      [0, 0, 0, 1]])
    inv_R_C2L = np.linalg.inv(R_C2L)
    R = np.dot(inv_R_C2L, pose_mat)
    rot = np.dot(R, R_C2L)
    return rot


def cull_map(map_points, ceiling, floor):
    """
    Removes points above the ceiling and below the floor so that when it\'s plotted in the horizontal plane points on
    the ceiling and floor get cut out for a cleaner map
    :param floor: Threshold below which map points will be removed
    :param ceiling: Threshold above which map points will be removed
    :param map_points: Point cloud
    :return: culled map
    """
    culled_map = []
    for i in range(len(map_points)):
        point = map_points[i]
        if ceiling > point[1] > floor:
            culled_map.append(point)

    return culled_map


def load_map(map_path):
    """
    Loads the map for use in the report generation
    :param: map_path path to the map.msg file
    :returns: map
    """
    # Read file as binary and unpack data using MessagePack library
    with open(map_path, "rb") as f:
        data = msgpack.unpackb(f.read(), use_list=False, raw=False)

    # The point data is tagged "landmarks"
    landmarks = data["landmarks"]

    map_points_list = []
    for _, point in landmarks.items():
        map_points_list.append(np.block([np.asarray(point["pos_w"]), 1.0]))

    #  y is in the down direction so we flip it
    map_points = np.asfarray(map_points_list) * np.array([1, -1, 1, 1])
    return map_points


def load_poses(file_name, toCameraCoord = False):
    """
    Each line in the file should follow the following structure
    pose(3x4 matrix in terms of 12 numbers)
    :param file_name: Path to file
    :param toCameraCoord: Whether to convert to camera coordinates
    :return: Numpy array of trajectory points n x [x, y, z]
    """
    f = open(file_name, 'r')
    s = f.readlines()
    f.close()
    poses = []

    for cnt, line in enumerate(s):
        P = np.eye(4)
        line_split = [float(i) for i in line.split()]
        for row in range(3):
            for col in range(4):
                P[row, col] = line_split[row * 4 + col]
        if toCameraCoord:
            poses.append(toCameraCoord(P))
        else:
            poses.append(P)

    trajectory = np.asarray(poses)
    #  y is in the down direction so we flip it
    trajectory = trajectory[:, :, 3] * np.array([1, -1, 1, 1])
    return trajectory

