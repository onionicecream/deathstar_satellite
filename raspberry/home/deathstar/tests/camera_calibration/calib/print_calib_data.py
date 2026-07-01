'''
print the camera calibration parameters stored in the .npz object
'''
import cv2
import numpy as np
import pickle

# data = np.load('calib.npz')
# data = np.load('calib_focus6.5.npz')
data = np.load('calib_focus_v1.3_1296x972.npz') #obtained error score of about 0.56


cMat = data['cameraMatrix']
dcoeff = data['distCoeffs']
rvecs = data['rvecs']
tvecs = data['tvecs']

    
print(f"cMat:\n{cMat}\n")
print(f"dcoeff:\n{dcoeff}\n")
# print(f"rvecs:\n{rvecs}\n")
# print(f"tvecs:\n{tvecs}\n")

