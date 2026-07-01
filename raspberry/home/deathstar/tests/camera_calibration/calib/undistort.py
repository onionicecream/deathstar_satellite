'''
this was a test to preview the picture before and after image correction; 
since the camera doesn't have that accute distortion the difference is not that visible by naked eye
'''
import cv2 as cv
import numpy as np
from picamera2 import Picamera2
from scipy.spatial.transform import Rotation
import libcamera
import time
import glob

# data = np.load('calib.npz')
# data = np.load('calib_focus6.5.npz')
data = np.load('calib_focus_v1.3_1296x972.npz')

images = glob.glob('/home/deathstar/tests/camera_calibration/calib/images_focusv1.3-1296x972_1/*.jpg')

mtx = data['cameraMatrix']
dist = data['distCoeffs']
rvecs = data['rvecs']
tvecs = data['tvecs']

print(f"cMat:\n{mtx}\n")
print(f"dcoeff:\n{dist}\n")
# print(f"rvecs:\n{rvecs}\n")
# print(f"tvecs:\n{tvecs}\n")

img = cv.imread(images[2])
cv.imshow('original', img)
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

##### 1.Using cv.undistort() #####

# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imshow('calibresult1.png', dst)

##### 2.Using remapping #####
# undistort
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
 
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imshow('calibresult2.png', dst)

cv.waitKey(0)
cv.destroyAllWindows()

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2SQR) / len(imgpoints2)
    mean_error += error

print( "total error: {}".format(np.sqrt(mean_error/len(objpoints))) )
