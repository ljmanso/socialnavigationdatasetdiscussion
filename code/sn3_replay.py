import sys
import time
import pickle

import itertools

import numpy as np

import cv2

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String
# from geometry_msgs.msg import Pose
# from nav_msgs.msg import OccupancyGrid
# from sensor_msgs.msg import LaserScan


from sn3min import *

print_times = {}
t0file = None
t0real = time.time()

tags = None

def delay_until(msg):
    global t0file
    global t0real

    if t0file is None:
        t0file = msg["time"]
        return

    inc_file = msg["time"] - t0file
    inc_real = (time.time() - t0real)
    error = inc_file - inc_real

    if error > 0:
        time.sleep(error)

def move_objects_rf(tags):
    ret = {}

    yaw_inc = 2.6 # 2.5
    x_inc = 1.2
    y_inc = 3.
    print('TAGS', tags)
    for tag_id, tag in tags.items():
        x = tag['t'][0]
        y = -tag['t'][1]
        z = tag['t'][2]
        yaw = tag['yaw']
        # ret[tag_id] = {"id": tag_id, "t": [x, y, z], "yaw": yaw}
        new_yaw = yaw - yaw_inc - np.pi/2.
        new_x = ( x*np.cos(yaw_inc) + y*np.sin(yaw_inc)) + x_inc
        new_y = (-x*np.sin(yaw_inc) + y*np.cos(yaw_inc)) + y_inc   
        ret[tag_id] = {"id": tag_id, "t": [new_x, new_y, z], "yaw": new_yaw}

    return ret

def print_msg(msg):
    print("{", end="")
    print(f"type: {msg['type']}", end=", ")
    print(f"msg: {type(msg['data'])}", end=", ")
    print(f"time: {msg['time']}", end="")
    print("}")

def draw_tags(tags, ss):
    if tags is None:
        return

    for tag in tags.values():
        t = tag['t']
        x = t[0]
        y = t[1]
        yaw = tag['yaw']
        x = int(-(x)/ss.map_mult)+MAP_COLS_HLEN+420
        y = MAP_COLS_HLEN-int(-(y)/ss.map_mult)+120
        x2 = x + int(25*np.cos(-yaw+ss.map_yaw+np.pi))
        y2 = y + int(25*np.sin(-yaw+ss.map_yaw+np.pi))
        cv2.circle(ss.canvas, (x, y), 10, (255,0,0), 2)
        cv2.line(ss.canvas, (x, y), (x2, y2), (0,255, 0), 2)
        cv2.putText(ss.canvas, str(tag["id"]),
            org=(x, y),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.8,
            color=(0, 0, 255))

class FileSubscriber():
    def __init__(self):
        self.org_map_resolution = None
        self.org_map_width = None
        self.org_map_height = None

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.map_data = None
        self.canvas = np.zeros((FINAL_MAP_WIDTH, FINAL_MAP_WIDTH, 3))
        self.canvas_stored = np.zeros((FINAL_MAP_WIDTH, FINAL_MAP_WIDTH, 3))

        self.pose_msg = None
        self.laser_msg = None
        self.goal_msg = None
        self.skeletons = None

        self.video = {}

    def draw_things(self):

        # Draw map
        self.canvas[:, :, :] = self.canvas_stored[:, :, :]

        # Draw pose
        if self.pose_msg is not None:
            position = self.pose_msg.position
            self.robot_x = position.x
            self.robot_y = position.y
            self.robot_yaw = euler_from_quaternion(self.pose_msg.orientation)[2]
            rx = int(-(self.robot_x)/self.map_mult)+MAP_COLS_HLEN+420
            ry = MAP_COLS_HLEN-int(-(self.robot_y)/self.map_mult)+120
            x2 = rx + int(25*np.cos(-self.robot_yaw+self.map_yaw+np.pi))
            y2 = ry + int(25*np.sin(-self.robot_yaw+self.map_yaw+np.pi))
            cv2.circle(self.canvas, (rx, ry), 10, (255,0,0), 2)
            cv2.line(self.canvas, (rx, ry), (x2, y2), (0,255, 0), 2)
        else:
            return

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

        # Draw laser
        if self.laser_msg is not None:
            angle = self.laser_msg.angle_min - self.robot_yaw + -0.
            for r in self.laser_msg.ranges:
                if np.isinf(r) is True:
                    r = 0
                #convert angle and radius to cartesian coordinates
                lx = r*np.cos(-angle) + self.robot_x
                ly = r*np.sin(- angle) + self.robot_y
                lx = int(-(lx)/self.map_mult)+MAP_COLS_HLEN+420
                ly = MAP_COLS_HLEN-int(-(ly)/self.map_mult)+120
                angle = angle + self.laser_msg.angle_increment
                cv2.circle(self.canvas, (lx, ly), 2, (0.7, 0.7, 0))


        # Draw video
        for k in self.video.keys():
            if self.video[k] is not None:
                cv2.imshow(k, self.video[k])

        global tags
        draw_tags(tags, self)

        cv2.imshow("sn3", self.canvas)
        k = cv2.waitKey(1)
        if k == 27:
            sys.exit(0)
        elif k == 112:
            global pause
            pause = not pause


    def listener_pose(self, msg):
        # print(msg2dict(msg))
        self.pose_msg = msg


    def listener_laser(self, msg):
        # print(msg2dict(msg))
        self.laser_msg = msg


    def listener_goal(self, msg):
        self.goal_msg = [float(x) for x in msg.split(",")]


    def listener_map(self, msg, dont_save=False):
        # dict = msg2dict(msg)
        self.org_map_resolution = msg.info.resolution
        self.org_map_width = msg.info.width
        self.org_map_height = msg.info.height
        origin = msg.info.origin
        self.map_yaw = euler_from_quaternion(origin.orientation)[2]
        self.map_yaw = 0 # 0.53 # float(sys.argv[2])
        
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


if __name__ == "__main__":
    # global t0file
    # global t0real
    pause = False
    last_draw = time.time()

    stuff = FileSubscriber()

    wfd = open(sys.argv[1], "rb")

    while True:
        if pause is not True:
            try:
                msg = pickle.load(wfd)
                if type(msg) != type({}):
                    print("??", msg)
                    continue
                # print("--------------------------------")
                # print_msg(msg)
                delay_until(msg)
                if msg["type"] == "map":
                    stuff.listener_map(msg["data"])
                elif msg["type"] == "goal":
                    stuff.listener_goal(msg["data"])
                elif msg["type"] == "pose":
                    stuff.listener_pose(msg["data"])
                elif msg["type"] == "laser":
                    stuff.listener_laser(msg["data"])
                elif msg["type"] == "humans":
                    stuff.skeletons = msg["data"]
                elif msg["type"].startswith("video"):
                    stuff.video[msg["type"]] = msg["data"]
                elif msg["type"] == "objects":
                    tags = move_objects_rf(msg["data"])
                elif msg["type"] == "command":
                    pass
                else:
                    print("Unhandled message", msg["type"])
                    print(msg)
                    sys.exit(-1)
                t = time.time()
            except EOFError:
                print('EOF')
                break

            if t-last_draw>0.05:
                last_draw = t
                stuff.draw_things()

    wfd.close()



