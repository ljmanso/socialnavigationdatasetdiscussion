import sys
import json
import pickle

import numpy as np

import cv2

from sn3min import *


try:
    with open('calibration.pickle', 'rb') as handle:
        ret, mtx, dist, rvecs, tvecs, h, w, mtx, dist, rvecs, tvecs = pickle.load(handle)
except:
    print("Can't get calibration file")
    sys.exit(-1)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0.5, (w,h))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)

DRAW_JOINTS = False
ROBOT_WIDTH = 23
HUMAN_WIDTH = 23
FPS = 25

writer = None
concatenated = None

t0file = None

data_structure = {
    "grid": None,
    "sequence": []
}

concatenated = None

objects = json.load(open("objects.json","r"))
walls = json.load(open("walls.json","r"))


def rotate(x, y, radians):
    xx = x * np.cos(radians) + y * np.sin(radians)
    yy = -x * np.sin(radians) + y * np.cos(radians)
    return [xx, yy]

def draw_object(o, canvas, map_mult):
    if o["type"] == "table":
        draw_rectangle(o, canvas, map_mult, (63,133,205))
    if o["type"] == "shelf":
        draw_rectangle(o, canvas, map_mult, (205,133,63))
    if o["type"] == "TV":
        draw_rectangle(o, canvas, map_mult, (100,100,100))

def draw_rectangle(o, canvas, map_mult, color):
    pts = np.array([rotate(-o["width"]/2, -o["depth"]/2, o["angle"]),
                    rotate(-o["width"]/2, +o["depth"]/2, o["angle"]),
                    rotate(+o["width"]/2, +o["depth"]/2, o["angle"]),
                    rotate(+o["width"]/2, -o["depth"]/2, o["angle"]),
                    rotate(-o["width"]/2, -o["depth"]/2, o["angle"])])
    offset = np.array([o["x"], o["y"]])
    pts += offset
    pts[:,0] = (-(pts[:,0])/map_mult)+MAP_COLS_HLEN+420
    pts[:,1] = MAP_COLS_HLEN-(-(pts[:,1])/map_mult)+120
    pts = pts.reshape((1,-1,2)).astype(np.int32)
    cv2.fillPoly(canvas, pts, color)

def draw_wall(w, canvas, map_mult, color=None):
    if color is None:
        c = (0,0,190)
    else:
        c = color
    pt1x = int((-(w[0][0])/map_mult)+MAP_COLS_HLEN+420)
    pt1y = int(MAP_COLS_HLEN-(-(w[0][1])/map_mult)+120)
    pt2x = int((-(w[1][0])/map_mult)+MAP_COLS_HLEN+420)
    pt2y = int(MAP_COLS_HLEN-(-(w[1][1])/map_mult)+120)
    cv2.line(canvas, (pt1x, pt1y), (pt2x, pt2y), c, thickness=4)

def json_struct_from_map(main, map):
    print(map)
    ret = {
        "width": MAP_ROWS_HLEN,
        "height": MAP_ROWS_HLEN,
        "resolution": map.info.resolution,
        "data": map.data.tolist()
    }
    return ret

def get_human_pose(skeleton):
    shoulders = (5, 6) # left, right
    hips = (11, 12) # left, right
    angle_srcs = (shoulders, hips)
    angle_final = None
    x = 0
    y = 0
    for angle_src in angle_srcs:
        a = angle_src[0]
        b = angle_src[1]
        ax = skeleton[a*3+0]
        ay = skeleton[a*3+1]
        bx = skeleton[b*3+0]
        by = skeleton[b*3+1]
        x += ax + bx
        y += ay + by
        angle = np.arctan2(by-ay, bx-ax)
        if angle_final is None:
            angle_final = angle
        else:
            angle_final = angle_final/2 + angle/2
    angle += np.pi/2.
    hx = x / 4
    hy = y / 4
    return hx, hy, angle

class FileSubscriber():
    def __init__(self):
        self.org_map_resolution = None
        self.org_map_width = None
        self.org_map_height = None

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.R = cv2.getRotationMatrix2D((FINAL_MAP_WIDTH//2, FINAL_MAP_WIDTH//2), -30.5-90, 1.0)


        self.map_data = None
        self.canvas = np.zeros((FINAL_MAP_WIDTH, FINAL_MAP_WIDTH, 3), dtype=np.uint8)
        self.canvas_stored = np.zeros((FINAL_MAP_WIDTH, FINAL_MAP_WIDTH, 3), dtype=np.uint8)

        self.pose_msg = None
        self.laser_msg = None
        self.goal_msg = None
        self.skeletons = None

        self.rx = self.ry = self.ra = 0.
        self.linear_vel = [0.,0.]
        self.angular_vel = 0.
        self.rgx = self.rgy = self.rga = 0
        self.timestamp = 0
        self.objects = objects
        self.walls = walls
        self.humans = []
        self.interactions = []


        self.video = {}


    def add_to_json_structure(self):

        ret = {
            "timestamp": self.timestamp,
            "robot": {
                    "x": self.robot_x,
                    "y": self.robot_y,
                    "a": self.robot_yaw,
                    "linear_vel": self.linear_vel,
                    "angular_vel": self.angular_vel,
                    "goalx": self.rgx,
                    "goaly": self.rgy,
                    "goala": self.rga
                },
            "people": self.humans,
            "objects": self.objects,
            "walls": self.walls,
            "interactions": self.interactions # TO DO
        }
        data_structure["sequence"].append(ret)

    def draw_things(self):
        # Draw map
        self.canvas[:, :, :] = self.canvas_stored[:, :, :]

        # Draw objects
        for o in objects:
            draw_object(o, self.canvas, self.map_mult)

        # Draw walls
        for w in walls:
            draw_wall(w, self.canvas, self.map_mult)

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
            cv2.circle(self.canvas, (rx, ry), ROBOT_WIDTH, (255,0,0), 2)
            cv2.line(self.canvas, (rx, ry), (x2, y2), (0,255, 0), 2)
        else:
            return

        # Draw skeletons
        if self.skeletons is not None:
            for skeleton_idx in range(len(self.skeletons)):
                skeleton = self.skeletons[skeleton_idx]
                if len(skeleton)<18:
                    continue
                for joint in range(18):
                    x_idx = joint*3
                    y_idx = x_idx+1
                    x = skeleton[x_idx]
                    y = skeleton[y_idx]
                    x2d = int(-(x)/self.map_mult)+MAP_COLS_HLEN+420
                    y2d = MAP_COLS_HLEN-int(-(y)/self.map_mult)+120
                    if joint in (2,4,6,8,10,12,14,16):
                        color = (255,150,150)
                    elif joint in (1,3,5,7,9,11,13,15):
                        color = (150,150,255)
                    else:
                        color = (120,225,225)
                    if DRAW_JOINTS:
                        cv2.circle(self.canvas, (x2d, y2d), 4, color, 2)
                # Draw as a oriented circle
                hx, hy, hangle = self.humans[skeleton_idx]
                cx = int(-(hx)/self.map_mult)+MAP_COLS_HLEN+420
                cy = MAP_COLS_HLEN-int(-(hy)/self.map_mult)+120
                x2 = cx + int(25*np.cos(-hangle+self.map_yaw+np.pi))
                y2 = cy + int(25*np.sin(-hangle+self.map_yaw+np.pi))
                cv2.circle(self.canvas, (cx, cy), HUMAN_WIDTH, (0,0,0), 2)
                cv2.line(self.canvas, (cx, cy), (x2, y2), (0,0,255), 2)


        # Draw goal
        if self.goal_msg is not None:
            position = self.goal_msg[0:3]
            quat = self.goal_msg[3:7]
            status = self.goal_msg[-1]
            self.rgx = position[0]
            self.rgy = position[1]
            if status < 3:
                color = tuple((0,1,0))
            else:
                color = tuple((0,0.35,0.65))
            self.rga = euler_from_tuple(quat)[2]
            gx = int(-(self.rgx)/self.map_mult)+MAP_COLS_HLEN+420
            gy = MAP_COLS_HLEN-int(-(self.rgy)/self.map_mult)+120
            x2 = gx + int(25*np.cos(-self.rga+self.map_yaw+np.pi))
            y2 = gy + int(25*np.sin(-self.rga+self.map_yaw+np.pi))
            cv2.circle(self.canvas, (gx, gy), ROBOT_WIDTH, color, 2)
            cv2.line(self.canvas, (gx, gy), (x2, y2), (0,0,0), 2)

        rotated = np.array(self.canvas)
        rotated = cv2.warpAffine(rotated, self.R, (rotated.shape[0], rotated.shape[1]), borderValue=(127,127,127))[120:-85, 85:-85]
        concatenate = [rotated]

        # Draw video
        f = np.zeros((720, 1280), dtype=np.uint8)
        for k in self.video.keys():
            if self.video[k] is not None:
                # cv2.imshow(k, self.video[k])
                global mapx, mapy, roi
                f[:, 160:-160] = cv2.resize(self.video[k], (0,0), fx=1.5, fy=1.5)
                dst = cv2.remap(f, mapx, mapy, cv2.INTER_LINEAR)
                dst = dst[8:520, 415:1210]
                new_width = int(float(dst.shape[1]*rotated.shape[0])/float(dst.shape[0]))
                dst = cv2.resize(dst, (new_width, rotated.shape[0]), cv2.INTER_CUBIC)
                concatenate.append(cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR))
                # cv2.imshow(f"{k} undistorted", dst)

        global concatenated
        concatenated = np.concatenate(concatenate, axis=1)

        global writer
        if writer == None:
            writer = cv2.VideoWriter(
                sys.argv[1].replace(".pickle", ".avi"),
                cv2.VideoWriter_fourcc('M','J','P','G'),
                FPS,
                (concatenated.shape[1], concatenated.shape[0]))
        writer.write(concatenated)

        cv2.imshow("view", concatenated)



        if cv2.waitKey(250) == 27:
            sys.exit(0)


    def listener_pose(self, msg):
        self.pose_msg = msg

    def listener_command(self, msg):
        self.linear_vel = [x for x in msg[:2]]
        self.angular_vel = msg[-1]
        


    def listener_laser(self, msg):
        self.laser_msg = msg


    def listener_goal(self, msg):
        self.goal_msg = [float(x) for x in msg.split(",")]


    def listener_map(self, msg, dont_save=False):
        self.org_map_resolution = msg.info.resolution
        self.org_map_width = msg.info.width
        self.org_map_height = msg.info.height
        origin = msg.info.origin
        self.map_yaw = euler_from_quaternion(origin.orientation)[2]
        self.map_yaw = 0 # 0.53 # float(sys.argv[2])
        
        self.x_orig = origin.position.x*self.org_map_resolution
        self.y_orig = origin.position.y*self.org_map_resolution

        self.map_data = np.asarray(msg.data, dtype=np.int8).reshape(self.org_map_height, self.org_map_width)
        self.map_data[self.map_data==-1] = 50
        self.map_data = self.map_data[MAP_ROWS_OFFSET:MAP_ROWS_END , MAP_COLS_OFFSET:MAP_COLS_END].astype(np.uint8)[:,::-1]
        self.map_data[self.map_data==50] = 127
        self.map_data[self.map_data==100] = 255
        self.map_data = cv2.resize(self.map_data, None, fx=ORG_MAP_SCALE, fy=ORG_MAP_SCALE, interpolation= cv2.INTER_NEAREST)
        self.canvas_stored[:,:,0] = 255-self.map_data[:,:]
        self.canvas_stored[:,:,1] = 255-self.map_data[:,:]
        self.canvas_stored[:,:,2] = 255-self.map_data[:,:]
        self.map_mult = self.org_map_resolution/ORG_MAP_SCALE

        self.map_msg = msg
        data_structure["grid"] = json_struct_from_map(self, msg)

    def listener_skeletons(self, data):
        self.skeletons = data
        self.humans = [ get_human_pose(x) for x in self.skeletons ]

if __name__ == "__main__":
    last_draw = None

    stuff = FileSubscriber()

    wfd = open(sys.argv[1], "rb")

    while True:
        # Process message
        try:
            msg = pickle.load(wfd)
            if type(msg) != type({}):
                continue
            if msg["type"] == "map":
                stuff.listener_map(msg["data"])
            elif msg["type"] == "goal":
                stuff.listener_goal(msg["data"])
            elif msg["type"] == "pose":
                stuff.listener_pose(msg["data"])
            elif msg["type"] == "laser":
                stuff.listener_laser(msg["data"])
            elif msg["type"] == "humans":
                stuff.listener_skeletons(msg["data"])
            elif msg["type"].startswith("video"):
                stuff.video[msg["type"]] = msg["data"]
            elif msg["type"] == "command":
                stuff.listener_command(msg["data"])
            else:
                print("Message type not handled?", msg["type"])
                sys.exit(1)
        except EOFError:
            break

        # Process drawing and video
        if last_draw is None:
            draw = True
        elif msg["time"]-last_draw>1./float(FPS):
            draw = True
        else:
            draw = False
        if draw:
            last_draw = msg["time"]
            stuff.draw_things()
            stuff.add_to_json_structure()

    json_fd = open(sys.argv[1].replace(".pickle", ".json"), "w")
    # print(data_structure)
    json.dump(data_structure, json_fd, indent=4)
    wfd.close()

    for i in range(FPS*3):
        writer.write(concatenated)
        concatenated[concatenated>2] -= 2


