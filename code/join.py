import sys

import pickle as pk

from sn3min import *

import std_msgs
import geometry_msgs
import nav_msgs
import sensor_msgs

from sn3noros import *

def pose2noros(data):
    position = Point(x=data.position.x, y=data.position.y, z=data.position.z)
    orientation = Quaternion(x=data.orientation.x, y=data.orientation.y, z=data.orientation.z, w=data.orientation.w)
    return Pose(position=position, orientation=orientation)

def time2noros(msg):
    return Time(sec=msg.sec, nanosec=msg.nanosec)

def header2noros(msg):
    stamp = Time(sec=msg.stamp.sec, nanosec=msg.stamp.nanosec)
    return Header(frame_id=str(msg.frame_id), stamp=stamp)

def mapmetadata2noros(msg):
    map_load_time = time2noros(msg.map_load_time)
    origin = pose2noros(msg.origin)
    return MapMetaData(map_load_time=map_load_time, resolution=msg.resolution, width=msg.width, height=msg.height, origin=origin)

def occupancygrid2noros(msg):
    header = header2noros(msg.header)
    info = mapmetadata2noros(msg.info)   
    data = np.array(msg.data)
    return OccupancyGrid(header=header, info=info, data=data)

def laser2noros(msg):
    header = header2noros(msg.header)
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment
    time_increment = msg.time_increment
    scan_time = msg.scan_time
    range_min = msg.range_min
    range_max = msg.range_max
    ranges = np.array(msg.ranges)
    intensities = np.array(msg.intensities)

    return LaserScan(header=header, angle_min=angle_min, angle_max=angle_max,
                     angle_increment=angle_increment, time_increment=time_increment,
                     scan_time=scan_time, range_min=range_min, range_max=range_max,
                     ranges=ranges, intensities=intensities)

def ros2noros(msg):
    if type(msg["data"]) == list:
        return msg["data"]
    elif type(msg["data"]) == np.ndarray:
        return msg["data"]
    elif type(msg["data"]) == dict:
        return msg["data"]
    elif type(msg["data"]) == std_msgs.msg._string.String:
        return msg["data"].data
    elif type(msg["data"]) == geometry_msgs.msg._pose.Pose:
        return pose2noros(msg["data"])
    elif type(msg["data"]) == nav_msgs.msg._occupancy_grid.OccupancyGrid:
        return occupancygrid2noros(msg["data"])
    elif "laser_scan" in str(type(msg["data"])):
        return laser2noros(msg["data"])
    else:
        print(type(msg["data"]))
        print("ros2noros option not handled")
        sys.exit(-1)


def ros2dict(msg):
    ret = {
        "type": msg["type"],
        "time": msg["time"],
        "data": ros2noros(msg)
        }
    return ret

if __name__ == "__main__":
    seed_filename = sys.argv[1]

    wfd = open(seed_filename.replace("_", "_trj.pickle"), "wb")


    afd = open(seed_filename.replace("_", "_pre.pickle"), "rb")
    while True:
        try:
            pk.dump(ros2dict(pk.load(afd)), wfd)
        except EOFError:
            break
    afd.close()


    bfd = open(seed_filename.replace("_", "_dat.pickle"), "rb")
    while True:
        try:
            pk.dump(ros2dict(pk.load(bfd)), wfd)
        except EOFError:
            break
    bfd.close()

    wfd.close()

