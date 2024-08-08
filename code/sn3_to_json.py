import sys
import json
import pickle
import copy

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

VID = "video" in sys.argv
VID = False

XOFFSET = 435
YOFFSET = 125

DRAW_JOINTS = False
DRAW_CIRCLE = False
DRAW_SQUARE = False
if len(sys.argv) >= 3:
    DESC = sys.argv[2]
    if DESC == "joints":
        DRAW_JOINTS = True
    elif DESC == "circles":
        DRAW_CIRCLE = True
    elif DESC == "polygons":
        DRAW_SQUARE = True
NOX = (DRAW_JOINTS or DRAW_CIRCLE or DRAW_SQUARE) is False

ROBOT_WIDTH = 20
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

json_objects = json.load(open("objects.json","r"))
# for object_idx in range(len(json_objects)):
#     json_objects[object_idx]["x"] += +0.15
#     json_objects[object_idx]["y"] += -0.07
# json.dump(json_objects, open("objects_new.json","w"))
objects = []
for id, o in enumerate(json_objects):
    objects.append({
        "id": id,
        "type": o["type"],
        "x": o["x"],
        "y": o["y"],
        "angle": np.pi/2.-o["angle"],
        "size": [o["width"], o["depth"]]
        })

walls = json.load(open("walls.json","r"))
# for wall_idx in range(len(walls)):
#     walls[wall_idx][0][0] += +0.15
#     walls[wall_idx][1][0] += +0.15
#     walls[wall_idx][0][1] += -0.07
#     walls[wall_idx][1][1] += -0.07
# json.dump(walls, open("walls_new.json","w"))

walls = [w[0]+w[1] for w in walls]


def rotate(x, y, radians):
    xx = -x * np.sin(radians) + y * np.cos(radians)
    yy = x * np.cos(radians) + y * np.sin(radians)

    # xx = x * np.cos(radians) + y * np.sin(radians)
    # yy = -x * np.sin(radians) + y * np.cos(radians)
    return [xx, yy]

def draw_object(o, canvas, map_mult):
    if o["type"] == "table":
        draw_rectangle(o, canvas, map_mult, (63,133,205))
    if o["type"] == "shelf":
        draw_rectangle(o, canvas, map_mult, (205,133,63))
    if o["type"] == "TV":
        draw_rectangle(o, canvas, map_mult, (100,100,100))

def draw_rectangle(o, canvas, map_mult, color):
    pts = np.array([rotate(-o["size"][0]/2, -o["size"][1]/2, o["angle"]),
                    rotate(-o["size"][0]/2, +o["size"][1]/2, o["angle"]),
                    rotate(+o["size"][0]/2, +o["size"][1]/2, o["angle"]),
                    rotate(+o["size"][0]/2, -o["size"][1]/2, o["angle"]),
                    rotate(-o["size"][0]/2, -o["size"][1]/2, o["angle"])])
    offset = np.array([o["x"], o["y"]])
    pts += offset
    pts[:,0] = (-(pts[:,0])/map_mult)+MAP_COLS_HLEN+XOFFSET
    pts[:,1] = MAP_COLS_HLEN-(-(pts[:,1])/map_mult)+YOFFSET
    pts = pts.reshape((1,-1,2)).astype(np.int32)
    cv2.fillPoly(canvas, pts, color)


def draw_person(p, canvas, map_mult):
    w = 0.45 / 2.
    d = 0.2 / 2
    a = p["angle"]
    offset = np.array([p["x"], p["y"]])

    rr = 0.05
    pts = np.array([rotate( 0, -d, a),

                    rotate(-(w-rr), -d, a),
                    rotate(-w, -(d-0.05), a),

                    rotate(-w, +(d-rr), a),
                    rotate(-(w-rr), +d, a),

                    rotate(+(w-rr), +d, a),
                    rotate(+w, +(d-rr), a),

                    rotate(+w, -(d-rr), a),
                    rotate(+(w-rr), -d, a),

                    rotate(0, -d, a)])
    pts += offset
    pts[:,0] = (-(pts[:,0])/map_mult)+MAP_COLS_HLEN+XOFFSET
    pts[:,1] = MAP_COLS_HLEN-(-(pts[:,1])/map_mult)+YOFFSET
    pts = pts.reshape((1,-1,2)).astype(np.int32)
    cv2.fillPoly(canvas, pts, (20, 20, 60))

    pts = np.array(rotate(0, 0.05, a)) + offset
    pts[0] = (-(pts[0])/map_mult)+MAP_COLS_HLEN+XOFFSET
    pts[1] = MAP_COLS_HLEN-(-(pts[1])/map_mult)+YOFFSET
    pts = pts.astype(np.int32)
    cv2.circle(canvas, (pts[0], pts[1]), 6, (50,40,170), -1)
    pts = np.array(rotate(0, 0.12, a)) + offset
    pts[0] = (-(pts[0])/map_mult)+MAP_COLS_HLEN+XOFFSET
    pts[1] = MAP_COLS_HLEN-(-(pts[1])/map_mult)+YOFFSET
    pts = pts.astype(np.int32)
    cv2.circle(canvas, (pts[0], pts[1]), 2, (50,40,170), -1)


def draw_wall(w, canvas, map_mult, color=None):
    if color is None:
        c = (0,0,190)
    else:
        c = color
    pt1x = int((-(w[0])/map_mult)+MAP_COLS_HLEN+XOFFSET)
    pt1y = int(MAP_COLS_HLEN-(-(w[1])/map_mult)+YOFFSET)
    pt2x = int((-(w[2])/map_mult)+MAP_COLS_HLEN+XOFFSET)
    pt2y = int(MAP_COLS_HLEN-(-(w[3])/map_mult)+YOFFSET)
    cv2.line(canvas, (pt1x, pt1y), (pt2x, pt2y), c, thickness=4)

def json_struct_from_map(main, map):
    # print(map)
    WMAP = 350
    HMAP = 350
    OMAPY = int(-map.info.origin.position.y / map.info.resolution -HMAP/2) 
    OMAPX = int(-map.info.origin.position.x / map.info.resolution -WMAP/2) 
    ret = {
        "width": WMAP,
        "height": HMAP,
        "cell_size": map.info.resolution,
        "data": (np.asarray(map.data, dtype=np.int8).reshape(map.info.height, map.info.width)[OMAPY:OMAPY+HMAP, OMAPX:OMAPX+WMAP]).tolist()
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
        self.timestamp = None
        self.objects = []
        self.walls = walls
        self.humans = []
        self.interactions = []

        self.humans_ids = {}
        self.next_id = 0

        self.detected_objects_ids = {}
        self.detected_objects_mean_pose = {}
        for i in range(len(objects)):
            objects[i]["id"] = self.next_id
            self.next_id += 1

        self.initial_objects = objects

        self.frames_with_robot_pose = []
        self.robot_pose_updated = False

        self.video = {}


    def add_to_json_structure(self):

        ret = {
            "timestamp": self.timestamp,
            "robot": {
                    "radius": 0.25,
                    "x": self.robot_x,
                    "y": self.robot_y,
                    "angle": self.robot_yaw,
                    "speed_x": self.linear_vel[0],
                    "speed_y": self.linear_vel[1],
                    "speed_a": self.angular_vel,
                    "goal_x": self.rgx,
                    "goal_y": self.rgy,
                    "goal_angle": self.rga,
                    "goal_pos_th": 0.1,
                    "goal_angle_th": 0.1
                },
            "people": self.humans,
            "objects": self.objects,
            "walls": self.walls,
            "interactions": self.interactions # TO DO
        }
        n_data = len(data_structure["sequence"])
        if n_data > 0:
            last = data_structure["sequence"][n_data-1]["robot"]
            curr = ret["robot"]
            if last["x"]!=curr["x"] or last["y"]!=curr["y"] or last["angle"]!=curr["angle"]:
                self.frames_with_robot_pose.append(n_data)    
        # if self.robot_pose_updated:
        #     self.frames_with_robot_pose.append(len(data_structure["sequence"]))
        #     self.robot_pose_updated = False
        data_structure["sequence"].append(ret)

    def update_robot_and_goal_pose(self):
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

        if self.pose_msg is not None:
            position = self.pose_msg.position
            self.robot_x = position.x
            self.robot_y = position.y
            self.robot_yaw = euler_from_quaternion(self.pose_msg.orientation)[2]

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
            rx = int(-(self.robot_x)/self.map_mult)+MAP_COLS_HLEN+XOFFSET
            ry = MAP_COLS_HLEN-int(-(self.robot_y)/self.map_mult)+YOFFSET
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
                if DRAW_JOINTS:
                    for joint in range(18):
                        x_idx = joint*3
                        y_idx = x_idx+1
                        x = skeleton[x_idx]
                        y = skeleton[y_idx]
                        x2d = int(-(x)/self.map_mult)+MAP_COLS_HLEN+XOFFSET
                        y2d = MAP_COLS_HLEN-int(-(y)/self.map_mult)+YOFFSET
                        if joint in (2,4,6,8,10,12,14,16):
                            color = (255,150,150)
                        elif joint in (1,3,5,7,9,11,13,15):
                            color = (150,150,255)
                        else:
                            color = (120,225,225)
                        cv2.circle(self.canvas, (x2d, y2d), 4, color, 2)
                elif DRAW_SQUARE: # Draw as a polygon
                    draw_person(self.humans[skeleton_idx], self.canvas, self.map_mult)
                elif DRAW_CIRCLE: # Draw as a oriented circle
                    hx = self.humans[skeleton_idx]["x"]
                    hy = self.humans[skeleton_idx]["y"]
                    hangle = self.humans[skeleton_idx]["angle"]
                    cx = int(-(hx)/self.map_mult)+MAP_COLS_HLEN+XOFFSET
                    cy = MAP_COLS_HLEN-int(-(hy)/self.map_mult)+YOFFSET
                    x2 = cx + int(25*np.cos(-hangle+self.map_yaw+np.pi))
                    y2 = cy + int(25*np.sin(-hangle+self.map_yaw+np.pi))
                    cv2.circle(self.canvas, (cx, cy), HUMAN_WIDTH, (0,0,0), 2)
                    cv2.line(self.canvas, (cx, cy), (x2, y2), (0,0,255), 2)

                hx = self.humans[skeleton_idx]["x"]
                hy = self.humans[skeleton_idx]["y"]

                cx = int(-(hx)/self.map_mult)+MAP_COLS_HLEN+XOFFSET
                cy = MAP_COLS_HLEN-int(-(hy)/self.map_mult)+YOFFSET

                cv2.putText(self.canvas, str(self.humans[skeleton_idx]["id"]),
                org=(cx, cy),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255))


        # Draw goal
        if self.goal_msg is not None:
            status = self.goal_msg[-1]
            if status < 3:
                color = tuple((0,1,0))
            else:
                color = tuple((0,0.35,0.65))
            gx = int(-(self.rgx)/self.map_mult)+MAP_COLS_HLEN+XOFFSET
            gy = MAP_COLS_HLEN-int(-(self.rgy)/self.map_mult)+YOFFSET
            x2 = gx + int(25*np.cos(-self.rga+self.map_yaw+np.pi))
            y2 = gy + int(25*np.sin(-self.rga+self.map_yaw+np.pi))
            cv2.circle(self.canvas, (gx, gy), ROBOT_WIDTH, color, 2)
            cv2.line(self.canvas, (gx, gy), (x2, y2), (0,0,0), 2)

        rotated = np.array(self.canvas)
        # rotated = cv2.warpAffine(rotated, self.R, (rotated.shape[0], rotated.shape[1]), borderValue=(127,127,127))[120:-85, 85:-85]
        concatenate = [rotated]

        # Draw video
        if VID:
            f = np.zeros((720, 1280), dtype=np.uint8)
            for k in self.video.keys():
                if self.video[k] is not None:
                    global mapx, mapy, roi
                    f[:, 160:-160] = cv2.resize(self.video[k], (0,0), fx=1.5, fy=1.5)
                    dst = cv2.remap(f, mapx, mapy, cv2.INTER_LINEAR)
                    dst = dst[8:520, 415:1210]
                    new_width = int(float(dst.shape[1]*rotated.shape[0])/float(dst.shape[0]))
                    dst = cv2.resize(dst, (new_width, rotated.shape[0]), cv2.INTER_CUBIC)
                    concatenate.append(cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR))
        global concatenated
        concatenated = np.concatenate(concatenate, axis=1)

        # concatenated = self.video["video0"]
        # print(concatenated.shape)

        global writer
        if VID is True:
            v = "video"
        else:
            v = "novideo"
        if writer == None:
            writer = cv2.VideoWriter(
                sys.argv[1].replace(".pickle", f"_{DESC}_{v}.avi"),
                cv2.VideoWriter_fourcc('M','J','P','G'),
                FPS,
                (concatenated.shape[1], concatenated.shape[0]))
        writer.write(concatenated)

        cv2.imshow(sys.argv[1].replace(".pickle", f"_{DESC}_{v}.avi"), concatenated)



        if cv2.waitKey(1) == 27:
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


    def move_objects_rf(self, tags):
        ret = {}

        yaw_inc = 2.6 # 2.5
        x_inc = 1.2
        y_inc = 3.
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

    def listener_objects(self, msg):
        # We are hardcoding here the objects' types
        processed = []
        for objid, obj in msg.items():
            new_obj = dict()
            if objid in self.detected_objects_ids.keys():
                new_obj["id"] = self.detected_objects_ids[objid]
            else:
                new_obj["id"] = self.next_id
                self.detected_objects_ids[objid] = self.next_id
                self.next_id += 1
            new_obj['x'] = obj['t'][0]
            new_obj['y'] = obj['t'][1]
            new_obj['angle'] = obj['yaw']
            if objid in [1, 2, 3]:
                new_obj['type'] = "chair"       # small ones
                new_obj['size'] = [0.43, 0.48]
            elif objid in [4, 5]:
                new_obj['type'] = "chair"       # bigger ones
                new_obj['size'] = [0.66, 0.62]
            elif objid in [6, 7, 8]:
                new_obj['type'] = "table"
                new_obj['size'] = [0.40, 0.48]
            elif objid in [9, 10]:
                new_obj['type'] = "table"
                new_obj['size'] = [0.75, 0.42]
            else:
                print(f"objid {objid} not handled")
                sys.exit(-1)
            # Update pose dictionary for smoothing noise poses in objects
            if new_obj["id"] not in self.detected_objects_mean_pose.keys():
                self.detected_objects_mean_pose[new_obj["id"]] = {'x':[], 'y':[], 'angle': []}
            self.detected_objects_mean_pose[new_obj["id"]]['x'].append(new_obj['x'])
            self.detected_objects_mean_pose[new_obj["id"]]['y'].append(new_obj['y'])
            self.detected_objects_mean_pose[new_obj["id"]]['angle'].append(new_obj['angle'])

            processed.append(new_obj)
            
        self.detected_objects = processed
        self.objects = self.initial_objects + self.detected_objects


    def listener_map(self, msg, dont_save=False):
        self.org_map_resolution = msg.info.resolution
        self.org_map_width = msg.info.width
        self.org_map_height = msg.info.height
        origin = msg.info.origin
        self.map_yaw = euler_from_quaternion(origin.orientation)[2]
        self.map_yaw = 0 # 0.53 # float(sys.argv[2])
        
        self.x_orig = origin.position.x*self.org_map_resolution
        self.y_orig = origin.position.y*self.org_map_resolution

        msg.data[msg.data==50] = -1
        msg.data[msg.data==100] = 1
        self.map_data = np.asarray(msg.data, dtype=np.int8).reshape(self.org_map_height, self.org_map_width)
        # self.map_data[self.map_data==-1] = 50
        self.map_data = self.map_data[MAP_ROWS_OFFSET:MAP_ROWS_END , MAP_COLS_OFFSET:MAP_COLS_END][:,::-1]
        # self.map_data[self.map_data==50] = 127
        # self.map_data[self.map_data==100] = 1
        self.map_data = cv2.resize(self.map_data, None, fx=ORG_MAP_SCALE, fy=ORG_MAP_SCALE, interpolation= cv2.INTER_NEAREST)
        gray_map_data = copy.deepcopy(self.map_data).astype(np.uint8)
        gray_map_data[gray_map_data==-1] = 127
        gray_map_data[gray_map_data==1] = 255
        self.canvas_stored[:,:,0] = 255-gray_map_data[:,:]
        self.canvas_stored[:,:,1] = 255-gray_map_data[:,:]
        self.canvas_stored[:,:,2] = 255-gray_map_data[:,:]
        self.map_mult = self.org_map_resolution/ORG_MAP_SCALE

        self.map_msg = msg
        data_structure["grid"] = json_struct_from_map(self, msg)

    def get_id_for_human(self, pose, max_human_distance=1.0):
        h_id = -1
        min_dist = np.inf
        pc = np.array([pose[0], pose[1]])
        for id, id_info in self.humans_ids.items():
            lpose = id_info["pos"]
            pp = np.array([lpose[0], lpose[1]])
            dist = np.linalg.norm(pp-pc)
            if dist < min_dist:
                min_dist = dist
                h_id = id
        if h_id < 0 or min_dist>max_human_distance:
            h_id = self.next_id
            self.next_id += 1
        return h_id            
            
    def update_humans_ids(self, max_time_without_update = 3):
        for h in self.humans:
            id = h["id"]
            p = [h["x"], h["y"], h["angle"]]
            self.humans_ids[id] = {"pos": p, "timestamp": self.timestamp}

        ids = self.humans_ids.keys()
        for i in list(ids):
            if self.timestamp - self.humans_ids[i]["timestamp"] > max_time_without_update:
                del self.humans_ids[i]



    def listener_skeletons(self, data):
        self.skeletons = data
        self.humans = []
        for id, s in enumerate(self.skeletons):
            pose = get_human_pose(s)
            h_id = self.get_id_for_human(pose)
            self.humans.append({"id":h_id, "x":pose[0], "y":pose[1], "angle":pose[2]})
        # self.humans = [ get_human_pose(x) for x in self.skeletons ]
        self.update_humans_ids()

    def postprocess_detected_objects(self):
        mean_poses = {}
        for id, poses in self.detected_objects_mean_pose.items():
            mean_x = np.mean(poses['x'])
            mean_y = np.mean(poses['y'])
            mean_angle = np.mean(poses['angle'])
            mean_poses[id] = {'x':mean_x, 'y':mean_y, 'angle':mean_angle}
        for d in data_structure['sequence']:
            for o in d["objects"]:
                if o['id'] in mean_poses.keys():
                    o['x'] = mean_poses[o['id']]['x']
                    o['y'] = mean_poses[o['id']]['y']
                    o['angle'] = mean_poses[o['id']]['angle']

    def postprocess_robot_poses(self):
        if len(self.frames_with_robot_pose)>0:
            f = self.frames_with_robot_pose[0]
            last_x = data_structure['sequence'][f]["robot"]["x"]
            last_y = data_structure['sequence'][f]["robot"]["y"]
            last_angle = data_structure['sequence'][f]["robot"]["angle"]
            last_frame = -1
            for f in self.frames_with_robot_pose:
                lf = last_frame + 1
                new_x = data_structure['sequence'][f]["robot"]["x"]
                new_y = data_structure['sequence'][f]["robot"]["y"]
                new_angle = data_structure['sequence'][f]["robot"]["angle"]
                while lf < f:
                    if lf >= len(data_structure['sequence']):
                        break
                    data_structure['sequence'][lf]["robot"]["x"] = last_x + (new_x - last_x)*(lf-last_frame)/(f-last_frame)
                    data_structure['sequence'][lf]["robot"]["y"] = last_y + (new_y - last_y)*(lf-last_frame)/(f-last_frame)
                    angle = last_angle + (new_angle - last_angle)*(lf-last_frame)/(f-last_frame)
                    data_structure['sequence'][lf]["robot"]["angle"] = np.arctan2(np.sin(angle), np.cos(angle))
                    print("changing frame", lf)
                    print(data_structure['sequence'][lf]["robot"])
                    lf += 1
                print("frame", f)
                print(data_structure['sequence'][f]["robot"])
                print("------------------")
                last_frame = f
                last_x = new_x
                last_y = new_y
                last_angle = new_angle

class CustomEncoder(json.JSONEncoder):
    def encode(self, obj):
        json_str = super().encode(obj)
        return json_str
    
    def iterencode(self, obj, _one_shot=False):
        for chunk in super().iterencode(obj, _one_shot=_one_shot):
            if isinstance(obj, list) and len(obj) > 5:
                chunk = json.dumps(obj)
            yield chunk

def process_lists(data):
    if isinstance(data, dict):
        return {k: process_lists(v) for k, v in data.items()}
    

    elif isinstance(data, list):
        if len(data) > 25:
            if not isinstance(data[0], dict):
                return json.dumps(data)
        return [process_lists(i) for i in data]

    return data

def dump_custom(data, json_fd, indent=None):
    processed_data = process_lists(data)
    json_string = json.dumps(processed_data, indent=indent, cls=CustomEncoder)
    # Remove the extra quotes around the long lists
    json_string = json_string.replace('\\"', '"')
    json_string = json_string.replace(']"', ']').replace('"[', '[')
    json_fd.write(json_string)

if __name__ == "__main__":
    last_draw = None

    stuff = FileSubscriber()

    wfd = open(sys.argv[1], "rb")

    while True:
        try:
            # Load next message from file
            msg = pickle.load(wfd)
            if type(msg) != type({}):
                print(msg)
                print(type(msg))
                sys.exit(-1)
            # Update timestamp
            stuff.timestamp = msg["time"]
            # Process message
            if msg["type"] == "map":
                stuff.listener_map(msg["data"])
            elif msg["type"] == "goal":
                stuff.listener_goal(msg["data"])
            elif msg["type"] == "pose":
                stuff.robot_pose_updated = True
                stuff.listener_pose(msg["data"])
            elif msg["type"] == "laser":
                stuff.listener_laser(msg["data"])
            elif msg["type"] == "humans":
                stuff.listener_skeletons(msg["data"])
            elif msg["type"].startswith("video"):
                stuff.video[msg["type"]] = msg["data"]
            elif msg["type"] == "command":
                stuff.listener_command(msg["data"])
            elif msg["type"] == "objects":
                stuff.listener_objects(stuff.move_objects_rf(msg["data"]))
            else:
                print("Message type not handled?", msg["type"])
                sys.exit(1)
        except EOFError:
            break

        stuff.update_robot_and_goal_pose()
        
        # Process drawing and video
        if last_draw is None:
            draw = True
        elif msg["time"]-last_draw>1./float(FPS):
            draw = True
        else:
            draw = False
        if draw:
            last_draw = msg["time"]
            if NOX is False:
                stuff.draw_things()
            stuff.add_to_json_structure()

    stuff.postprocess_detected_objects()
    stuff.postprocess_robot_poses()
    json_fd = open(sys.argv[1].replace(".pickle", ".json"), "w")
    # print(data_structure)
    # json.dump(data_structure, json_fd, indent=4)
    dump_custom(data_structure, json_fd, indent=4)
    wfd.close()

    if NOX is False:
        for i in range(FPS*3):
            writer.write(concatenated)
            # concatenated[concatenated>2] -= 2


