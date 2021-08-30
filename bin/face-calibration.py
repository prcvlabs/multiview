import argparse
import cv2
import numpy as np
import os
import json


def check_params(objpoints, imgpoints, mtx, dist, rvecs, tvecs):
    """Check the quality of the calibration. Result should be close to 0"""
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

    print("total error: {}".format(mean_error/len(objpoints)))


def calib(images, sensor):
    print(images)

    # Calibration board coords
    rows, cols = 15, 10
    objp = np.zeros((rows*cols,3), np.float32)
    objp[:,:2] = np.mgrid[0:rows,0:cols].T.reshape(-1,2)

    # Corner refinement settings (just using the values from the tutorial)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objpoints = []
    imgpoints = []
    for idx, im_file in enumerate(images):
        img = cv2.imread(im_file) 
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (rows, cols), None)

        if ret:
            print('Found board for {}'.format(im_file))
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            img = cv2.drawChessboardCorners(img, (rows, cols), corners2, ret)
            cv2.imwrite('/tmp/corners_{}'.format(os.path.split(im_file)[1]), img)
        else:
            print('BOO! No board for {}'.format(im_file))

    # Calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Undistort
    img = cv2.imread(images[0])
    h,  w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
    print("Usable area = {}".format(roi))

    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite('/tmp/{}_calibresult.png'.format(sensor), dst)

    # Outputs to save
    out = {'K': newcameramtx.tolist(), 'usable area': roi, 'sensor-id': sensor} 
    with open('{}.json'.format(sensor), 'w') as f:
        f.write(json.dumps(out, indent=4))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--images', nargs='*')
    parser.add_argument('--face-sensor', '-f', type=str)
    args = parser.parse_args()

    calib(args.images, args.face_sensor)


