import sys
import time
import pickle
import datetime
import itertools

from collections import namedtuple, deque
from threading import Thread, RLock

import numpy as np
import  Ice
import cv2


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from image_thread import image_thread

vid = cv2.VideoCapture("/dev/video6")
if vid is None:
    print("Cannot open camera")
    sys.exit(-1)
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024)

image_thread_data = {
    'lock': RLock(),
    'tags': dict(),
    'newdata': False,
    'stop': False

}
tags = {}
cv_thread = Thread(target=image_thread, args=[vid, image_thread_data])
cv_thread.start()

import pose3DI

# from pytransform3d import rotations as pr
# from pytransform3d import transformations as pt
# from pytransform3d.transform_manager import TransformManager
# from pytransform3d.plot_utils import remove_frame


global wfd
wfd = None

import rosidl_runtime_py

def identity(x):
    return x
msg2dict = rosidl_runtime_py.convert.message_to_ordereddict 
msg2dict = identity

from sn3min import *

communicator = Ice.initialize(sys.argv)
base = communicator.stringToProxy("pose3d:default -h 192.168.88.104 -p 11301")
pose = pose3DI.Pose3DPrx.checkedCast(base)
if not pose:
    raise RuntimeError("Invalid proxy")
else:
    print('valid proxy')



def draw_transform(cmap, t, yaw):
    xc = int(t[0] * 60 + 400)
    yc = int(t[1] * 60 + 400)
    cv2.circle(cmap, (xc, yc), 10, (0, 255, 0), 2)
    xc2 = xc + int(np.cos(yaw)*20)
    yc2 = yc + int(np.sin(yaw)*20)
    cv2.line(cmap, (xc, yc), (xc2, yc2), (0, 255, 0), 2)



def draw_tags(tags, ss):
    for tag in tags.values():
        t = tag['t']
        x = t[0]
        y = t[1]
        z = t[2]
        yaw = tag['yaw']
        x = int(-(x)/ss.map_mult)+MAP_COLS_HLEN+420
        y = MAP_COLS_HLEN-int(-(y)/ss.map_mult)+120
        x2 = x + int(25*np.cos(-yaw+ss.map_yaw+np.pi))
        y2 = y + int(25*np.sin(-yaw+ss.map_yaw+np.pi))
        cv2.circle(ss.canvas, (x, y), 10, (255,0,0), 2)
        cv2.line(ss.canvas, (x, y), (x2, y2), (0,255, 0), 2)

print_times={}

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('sn3')
        self.record = False

        self.waitqueue = deque()

        self.subscription = self.create_subscription(Pose, "/robot/base_pose", self.listener_pose, 10)
        self.subscription_map = self.create_subscription(OccupancyGrid, "/robot/map", self.listener_map, 10)
        self.subscription_goal = self.create_subscription(String, "/robot/goal", self.listener_goal, 10)
        self.subscription_cmd = self.create_subscription(String, "/robot/velocity_command", self.listener_cmd, 10)
        # self.subscription_laser = self.create_subscription(LaserScan, "/robot/laser_data", self.listener_laser, 10)

        self.org_map_resolution = None
        self.org_map_width = None
        self.org_map_height = None

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.cmd = None

        self.map_data = None
        self.canvas = np.zeros((FINAL_MAP_WIDTH, FINAL_MAP_WIDTH, 3))
        self.canvas_stored = np.zeros((FINAL_MAP_WIDTH, FINAL_MAP_WIDTH, 3))

        self.pose_msg = None
        self.laser_msg = None
        self.goal_msg = None
        self.skeletons = None

        self.listener_map(pickle.load(open("map.pickle", "rb")), dont_save=True)

    def remove_old_in_queue(self):
        t = time.time()
        while len(self.waitqueue) > 0:
            if t - self.waitqueue[0]["time"] > 5:
                p = self.waitqueue.popleft()
            else:
                break

    def draw_things(self):
        if self.map_data is None:
            timed_print("draw", print_times, "pose (map is None)")
            return

        # Draw map
        self.canvas[:, :, :] = self.canvas_stored[:, :, :]

        # Draw pose
        if self.pose_msg is not None:
            position = self.pose_msg.position
            self.robot_x = position.x
            self.robot_y = position.y
            # print(self.robot_x, self.robot_y)
            self.robot_yaw = euler_from_quaternion(self.pose_msg.orientation)[2]
            x = int(-(self.robot_x)/self.map_mult)+MAP_COLS_HLEN+420
            y = MAP_COLS_HLEN-int(-(self.robot_y)/self.map_mult)+120
            x2 = x + int(25*np.cos(-self.robot_yaw+self.map_yaw+np.pi))
            y2 = y + int(25*np.sin(-self.robot_yaw+self.map_yaw+np.pi))
            cv2.circle(self.canvas, (x, y), 10, (255,0,0), 2)
            cv2.line(self.canvas, (x, y), (x2, y2), (0,255, 0), 2)
        else:
            timed_print("draw1", print_times, "no pose")

        # Draw skeletons
        if self.skeletons is not None:
            for skeleton in self.skeletons:
                if len(skeleton)<18:
                    continue
                for joint in range(18):
                    x_idx = joint*3
                    y_idx = x_idx+1
                    x = skeleton[x_idx]
                    y = skeleton[y_idx]
                    x2d = int(-(x)/self.map_mult)+MAP_COLS_HLEN+420
                    y2d = MAP_COLS_HLEN-int(-(y)/self.map_mult)+120
                    if joint in [2,4,6,8,10,12,14,16]:
                        color = (255,0,)
                    elif joint in [1,3,5,7,9,11,13,15]:
                        color = (0,0,255)
                    else:
                        color = (0,125,125)
                    cv2.circle(self.canvas, (x2d, y2d), 3, color, 2)

        # Draw goal
        if self.goal_msg is not None:

            # self.waitqueue

            position = self.goal_msg[0:3]
            quat = self.goal_msg[3:7]
            status = self.goal_msg[-1]
            gx = position[0]
            gy = position[1]
            if status < 3:
                color = tuple((0,1,0))
            else:
                color = tuple((0,0.35,0.65))
            goal_yaw = euler_from_tuple(quat)[2]
            gx = int(-(gx)/self.map_mult)+MAP_COLS_HLEN+420
            gy = MAP_COLS_HLEN-int(-(gy)/self.map_mult)+120
            x2 = gx + int(25*np.cos(-goal_yaw+self.map_yaw+np.pi))
            y2 = gy + int(25*np.sin(-goal_yaw+self.map_yaw+np.pi))
            cv2.circle(self.canvas, (gx, gy), 10, color, 2)
            cv2.line(self.canvas, (gx, gy), (x2, y2), (0,0,0), 2)

        # # Draw laser
        # if self.laser_msg is not None and self.robot_yaw is not None:
        #     angle = self.laser_msg.angle_min - self.robot_yaw
        #     subsample = 1
        #     for r in itertools.islice(self.laser_msg.ranges, None, None, subsample):
        #         if np.isinf(r) is True:
        #             r = 0
        #         #convert angle and radius to cartesian coordinates
        #         lx = r*np.cos(-angle) + self.robot_x
        #         ly = r*np.sin(- angle) + self.robot_y
        #         lx = int(-(lx)/self.map_mult)+MAP_COLS_HLEN+420
        #         ly = MAP_COLS_HLEN-int(-(ly)/self.map_mult)+120
        #         angle = angle + self.laser_msg.angle_increment
        #         cv2.circle(self.canvas, (lx, ly), 2, (0.7, 0.7, 0))
        # else:
        #     timed_print("draw_no_laser", print_times, "no laser?")

        draw_tags(tags, self)

        cv2.imshow("sn3", self.canvas)
        if cv2.waitKey(1) == 27:
            image_thread_data['lock'].acquire()
            image_thread_data['stop'] = True
            image_thread_data['lock'].release()
            sys.exit(0)

        if not self.record:
            self.remove_old_in_queue()

    def listener_pose(self, msg):
        if self.map_data is None:
            timed_print("pose", print_times, "(map is None)")
            return
        # print(msg2dict(msg))
        self.pose_msg = msg2dict(msg)
        if self.pose_msg is None:
            print("POSE IS NONE")
            sys.exit(-1)

        if self.record is True:
            global wfd
            pickle.dump({"type": "pose", "time": time.time(), "data": self.pose_msg}, wfd)
        else:
            self.waitqueue.append({"type": "pose", "time": time.time(), "data": self.pose_msg})

    def listener_cmd(self, msg):
        self.cmd = [float(x) for x in msg.data.split(",")]


        if self.map_data is None:
            timed_print("cmd", print_times, "(map is None)")
            return
        
        if self.cmd is None:
            print("CMD IS NONE")
            sys.exit(1)

        if self.record is True:
            global wfd
            pickle.dump({"type": "command", "time": time.time(), "data": self.cmd}, wfd)
        else:
            self.waitqueue.append({"type": "command", "time": time.time(), "data": self.cmd})


    def listener_laser(self, msg):
        self.laser_msg = msg

        msgw = {"type": "laser", "time": time.time(), "data": msg}
        if self.record is True:
            global wfd
            d = pickle.dump(msgw, wfd)
            print(d)
        else:
            self.waitqueue.append(msgw)

    def listener_goal(self, msg):
        self.goal_msg = [float(x) for x in msg.data.split(",")]

        global wfd
        ## IF THERE IS A GOAL
        if self.goal_msg[-1] < 3:
            ## IF THERE IS A (((NEW))) GOAL
            if self.record is False:
                # then we close the previous file (if necessary) and open a new one
                # global wfd
                if wfd is not None:
                    print("CLOSE2")
                    wfd.close()
                time_time = datetime.datetime.fromtimestamp(int(time.time())).isoformat()
                self.fname = f"{time_time}_dat.pickle"
                wfd = open(self.fname, "wb")
                self.force_first_message_map = {"type": "map", "time": time.time(), "data": msg2dict(self.map_msg)}
                print("OPEN", self.fname, wfd)
            # set record to true, so that messages are recorded
            self.record = True
        ## if there's no goal but.. THERE WAS A GOAL!
        elif self.record == True:
            self.record = False
            if wfd is not None:
                # then we need to close the file
                wfd.close()
                print("CLOSE")
            pre_fname = self.fname
            pre_fname = pre_fname.replace("_dat", "_pre")
            print("Writing queue", )
            prefd = open(pre_fname, "wb")
            print(prefd)
            while True:
                try:
                    msg = self.waitqueue.popleft()
                    if self.force_first_message_map is not None:
                        self.force_first_message_map["time"] = msg["time"]
                        pickle.dump(self.force_first_message_map, prefd)
                        self.force_first_message_map = None
                    pickle.dump(msg, prefd)
                except IndexError:
                    break
            prefd.close()

        if self.record is True:
            # global wfd
            pickle.dump({"type": "goal", "time": time.time(), "data": msg}, wfd)
        else:
            self.waitqueue.append({"type": "goal", "time": time.time(), "data": msg})

    def listener_map(self, msg, dont_save=False):
        print("MAP!")
        if dont_save is True:
            f = open("map.pickle", "wb")
            pickle.dump(msg, f)
        # dict = msg2dict(msg)
        self.org_map_resolution = msg.info.resolution
        self.org_map_width = msg.info.width
        self.org_map_height = msg.info.height
        origin = msg.info.origin
        self.map_yaw = euler_from_quaternion(origin.orientation)[2]
        self.x_orig = origin.position.x*self.org_map_resolution
        self.y_orig = origin.position.y*self.org_map_resolution

        self.map_data = np.asarray(msg.data, dtype=np.int8).reshape(self.org_map_height, self.org_map_width)
        # print(self.map_data.dtype, self.map_data.min(), self.map_data.max())
        self.map_data[self.map_data==-1] = 50
        # print('A', np.unique(self.map_data))
        self.map_data = self.map_data[MAP_ROWS_OFFSET:MAP_ROWS_END , MAP_COLS_OFFSET:MAP_COLS_END].astype(np.uint8)[:,::-1]
        self.map_data[self.map_data==50] = 127
        self.map_data[self.map_data==100] = 255
        # print('b', np.unique(self.map_data))
        self.map_data = cv2.resize(self.map_data, None, fx=ORG_MAP_SCALE, fy=ORG_MAP_SCALE, interpolation= cv2.INTER_NEAREST)
        self.canvas_stored[:,:,0] = 255-self.map_data[:,:]
        self.canvas_stored[:,:,1] = 255-self.map_data[:,:]
        self.canvas_stored[:,:,2] = 255-self.map_data[:,:]
        self.map_mult = self.org_map_resolution/ORG_MAP_SCALE

        self.map_msg = msg
        print("=================== M A P ====================")
        if dont_save is False:
            self.destroy_subscription(self.subscription_map)

def main(args=None):
    check_time = time.time()
    draw_time = time.time()

    global tags

    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
 
    while True:
        rclpy.spin_once(minimal_subscriber)
        t = time.time()
        if t-check_time > 0.05:
            check_time = t
            skeletons = []
            skeletons_tmp = pose.getskeletons()
            radians = 4.1
            for skeleton_tmp in skeletons_tmp:
                if len(skeleton_tmp)<18:
                    continue
                skeleton = []
                for joint in range(18):
                    x_idx = joint*3
                    y_idx = x_idx+1
                    z_idx = x_idx+2
                    x = -skeleton_tmp[x_idx]
                    y =  skeleton_tmp[y_idx]
                    z =  skeleton_tmp[z_idx]
                    xx = ( x * np.cos(radians) + y * np.sin(radians))+0.977
                    yy = (-x * np.sin(radians) + y * np.cos(radians))+2.21
                    zz = z
                    skeleton += [xx, yy, zz]
                skeletons.append(skeleton)
            minimal_subscriber.skeletons = skeletons
            if minimal_subscriber.record is True:
                global wfd
                pickle.dump({"type": "humans", "time": time.time(), "data": minimal_subscriber.skeletons}, wfd)
            else:
                minimal_subscriber.waitqueue.append({"type": "humans", "time": time.time(), "data": minimal_subscriber.skeletons})

        if t-draw_time > 0.05:
            draw_time = t
            minimal_subscriber.draw_things()

        image_thread_data['lock'].acquire()
        
        if image_thread_data['newdata'] is True:
            image_thread_data['newdata'] = False
            tags = image_thread_data['tags']
            frame = image_thread_data['frame']
            image_thread_data['lock'].release()
            cv2.imshow('frame', frame) 
            if minimal_subscriber.record is True:
                pickle.dump({"type": "objects", "time": time.time(), "data": tags}, wfd)
                pickle.dump({"type": "video0",  "time": time.time(), "data": frame}, wfd)
            else:
                minimal_subscriber.waitqueue.append({"type": "objects", "time": time.time(), "data": tags})
                minimal_subscriber.waitqueue.append({"type": "video0", "time": time.time(), "data": frame})
        else:
            image_thread_data['lock'].release()
            




    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# def find_bounds(self):
#     """
#     Calculates the bounding box of a map array
#     """
#     map = self.occupancy_grid
#     a = np.where(map != -1)
#     bbox = np.min(a[0]), np.max(a[0]), np.min(a[1]), np.max(a[1])
#     return Bounds(bbox[0], bbox[1], bbox[2], bbox[3])

# def get_submap(self):
#     """
#     Retrieve the filled sub map that's contained inside the map array
#     """
#     self.sub_map_bounds = self.find_bounds()
#     return self.occupancy_grid[self.sub_map_bounds.xmin:self.sub_map_bounds.xmax+1, self.sub_map_bounds.ymin:self.sub_map_bounds.ymax+1]
