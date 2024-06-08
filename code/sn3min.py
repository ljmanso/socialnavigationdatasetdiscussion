import time

from collections import namedtuple, OrderedDict

import numpy as np



MAP_ROWS_OFFSET = 940
MAP_ROWS_END = 1160
MAP_ROWS_LEN = MAP_ROWS_END-MAP_ROWS_OFFSET
MAP_ROWS_HLEN = MAP_ROWS_LEN//2

MAP_COLS_OFFSET = 915
MAP_COLS_END = 1135
MAP_COLS_LEN = MAP_COLS_END-MAP_COLS_OFFSET
MAP_COLS_HLEN = MAP_COLS_LEN//2

ORG_MAP_SCALE = 4.

FINAL_MAP_WIDTH = int(MAP_ROWS_LEN*ORG_MAP_SCALE)



# ros2noros data types
Header = namedtuple("Header", ['stamp', 'frame_id'])
MapMetaData = namedtuple("MapMetaData", ['map_load_time', 'resolution', 'width', 'height', 'origin'])
OccupancyGrid = namedtuple("OccupancyGrid", ['header', 'info', 'data'])
Odometry = namedtuple("Odometry", ['header', 'frame_id', 'pose', 'twist'])
Point = namedtuple("Point", ['x', 'y', 'z'])
Pose = namedtuple("Pose", ['position', 'orientation'])
PoseWithCovariance = namedtuple("PoseWithCovariance", ['pose', 'covariance'])
Quaternion = namedtuple("Quaternion", ['x', 'y', 'z', 'w'])
Time = namedtuple("Time", ['sec', 'nanosec'])
Twist = namedtuple("Twist", ['linear', 'angular'])
TwistWithCovariance = namedtuple("TwistWithCovariance", ['pose', 'covariance'])
LaserScan = namedtuple("LaserScan", ['header', 'angle_min', 'angle_max', 'angle_increment',
                                     'time_increment', 'scan_time', 'range_min',
                                     'range_max', 'ranges', 'intensities'])


def print_msg(msg):
    print("{", end="")
    print(f"type: {msg['type']}", end=", ")
    print(f"msg: {type(msg['data'])}", end=", ")
    print(f"time: {msg['time']}", end="")
    print("}")



def timed_print(name, print_times, *args,  **kwargs):
    t = time.time()

    do_it = False
    if name in print_times.keys():
        if t - print_times[name] > 5:
            do_it = True
    else:
        do_it = True
        
    if do_it:
        print_times[name] = t
        print(f"name={name}", *args, **kwargs)

def euler_from_tuple(tup):
    return euler_from_quaternion(Quaternion(*tup))

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    if type(quaternion) == OrderedDict:
        x = quaternion["x"]
        y = quaternion["y"]
        z = quaternion["z"]
        w = quaternion["w"]
    else:
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw