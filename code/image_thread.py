import sys
import pickle

import cv2

from dt_apriltags import *

from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager
from pytransform3d.plot_utils import remove_frame


TAG_SIZE=0.452         # MEASURED
SMALL_TAG_SIZE=0.1588  # MEASURED


def process_image_get_tags(frame, detector, cam_prms, tm, results=None):
    if results is None:
        results = dict()
    tags = detector.detect(frame, estimate_tag_pose=True, camera_params=cam_prms, tag_size=SMALL_TAG_SIZE)
    for tag in tags:
        if tag.tag_id == 0:
            continue
        t = tag.pose_t.ravel()
        cam2tag = pt.transform_from(tag.pose_R, t)
        tm.remove_transform("object", "camera")
        tm.add_transform("object", "camera", cam2tag)
        zero2tag = tm.get_transform("object", "root")
        # print(zero2tag)
        rot = zero2tag[:3, :3]
        T = zero2tag[:3, 3]
        yaw = pr.euler_from_matrix(rot, 0, 1, 2, extrinsic=True)[2]
        # draw_transform(cmap, T, yaw)
        results[tag.tag_id] = {"id": tag.tag_id, "t": T, "yaw": yaw}

    return results


def image_thread(vid, data):
    try:
        with open('calibration.pickle', 'rb') as handle:
            ret, mtx, dist, rvecs, tvecs, h, w, mtx, dist, rvecs, tvecs = pickle.load(handle)
    except:
        print("Can't get calibration file")
        sys.exit(-1)

    print(f"{w=} {h=}")
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0, (w,h))
    fx = newcameramtx[0,0]
    fy = newcameramtx[1,1]
    cx = newcameramtx[0,2]
    cy = newcameramtx[1,2]
    print(newcameramtx)
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)


    detector = Detector(searchpath=['apriltags'], families='tagStandard41h12',
                    nthreads=5, quad_decimate=1.0, quad_sigma=0.1, refine_edges=1,
                    decode_sharpening=0.25, debug=0)
    cam_prms = [fx, fy, cx, cy]

    transformFR = pickle.load(open("objectZ.pkl", "rb"))
    print("transformFR")
    print("transformFR")
    print(transformFR)
    tm = TransformManager()
    tm.add_transform("root", "camera", transformFR)

    tags = {}

    while True:
        # Shall we stop?
        data['lock'].acquire()
        stop = data['stop']
        data['lock'].release()
        if stop:
            break

        image_ok, frame = vid.read() 
        if image_ok is True:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        else:
            print("Can't read frame!!!")

        frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        frame = frame[roi[1]:roi[1]+roi[3]+1, roi[0]:roi[0]+roi[2]+1]
        tags = process_image_get_tags(frame, detector, cam_prms, tm, tags)
        miniframe = cv2.resize(frame, None, fx=0.25, fy=0.25, interpolation= cv2.INTER_NEAREST)

        data['lock'].acquire()
        data['newdata'] = True
        data['tags'] = tags
        data['frame'] = miniframe
        data['lock'].release()


