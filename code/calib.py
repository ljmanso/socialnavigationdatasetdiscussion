#!/usr/bin/env python

import sys 
import pickle 
import numpy as np
import cv2

CALIBRATION_VIDEO = "calibration.avi"
TEST_VIDEO = "test.avi"


def do_calibration(video_fname):
    print("running calibration procedure")
    MINIMUM_FRAME_SEPARATION = 10
    CHECKERBOARD = (6,9)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objpoints = []
    imgpoints = [] 


    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    
    pickle_write = False
    frames = None
    try:
        with open('frames.pickle', 'rb') as handle:
            frames = pickle.load(handle)
    except:
        print("Can't read frame list!")
        pickle_write = True
        frames = []

    vidcap = cv2.VideoCapture(video_fname)
    last_detection_frame = -MINIMUM_FRAME_SEPARATION
    frame_number = -1
    while vidcap.isOpened():
        if frame_number > 900:
            break
        success, image = vidcap.read()
        if success is not True:
            continue
        frame_number += 1
        if pickle_write is not True:
            if frame_number not in frames:
                continue
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(grey, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if frame_number - last_detection_frame < MINIMUM_FRAME_SEPARATION:
            continue

        if ret == True:
            print(f"detect at {frame_number=}")
            last_detection_frame = frame_number
            if pickle_write:
                frames.append(frame_number)
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(grey, corners, (11,11),(-1,-1), criteria)
            imgpoints.append(corners2)
            img = cv2.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)
        cv2.imshow("distorted image", image)
        cv2.waitKey(5)

    print("finished finding checkerboards")
    cv2.destroyAllWindows()
    
    if pickle_write:
        with open('frames.pickle', 'wb') as handle:
            pickle.dump(frames, handle, protocol=pickle.HIGHEST_PROTOCOL)

    h, w = image.shape[:2]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grey.shape[::-1], None, None)

    with open('calibration.pickle', 'wb') as handle:
        pickle.dump((ret, mtx, dist, rvecs, tvecs, h, w, mtx, dist, rvecs, tvecs), handle, protocol=pickle.HIGHEST_PROTOCOL)

    print("Camera matrix : \n")
    print(mtx)
    print("dist : \n")
    print(dist)
    print("rvecs : \n")
    print(rvecs)
    print("tvecs : \n")
    print(tvecs)

    return ret, mtx, dist, rvecs, tvecs, h, w, mtx, dist, rvecs, tvecs




if __name__ == "__main__":
    calib = None
    try:
        with open('calibration.pickle', 'rb') as handle:
            ret, mtx, dist, rvecs, tvecs, h, w, mtx, dist, rvecs, tvecs = pickle.load(handle)
    except:
        ret, mtx, dist, rvecs, tvecs, h, w, mtx, dist, rvecs, tvecs = do_calibration(CALIBRATION_VIDEO)

    # Refining the camera matrix using parameters obtained by calibration
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0.5, (w,h))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)

    print(f"{roi=}")

    # ADD = 25
    # roi = [roi[0]-ADD, roi[1]-ADD, roi[2]+ADD, roi[3]+ADD]
    #   col_start    row_start    width    height
    # roi = [185,          42,       350,     215]


    writer = None

    vidcap = cv2.VideoCapture(TEST_VIDEO)
    frame_number = -1
    while vidcap.isOpened():
        success, image = vidcap.read()
        print(image.shape)
        if success is not True:
            continue
        if frame_number > 900:
            break
        frame_number += 1
        dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
        dst = dst[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
        cv2.imshow("undistorted image", dst)

        if writer is None:
            writer = cv2.VideoWriter("outpy.avi", cv2.VideoWriter_fourcc('M','J','P','G'), 10, (dst.shape[1], dst.shape[0]))
        writer.write(dst)

        k = cv2.waitKey(9)
        if k == 27:
            break

    writer.release()



