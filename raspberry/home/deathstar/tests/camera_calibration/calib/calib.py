# https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html

import numpy as np
import cv2
import glob

# use the number of INNER corners (squares - 1)
# for board of 9x6 squares, the inner corners are 8x5
cRow = 5  
cCol = 8  
square_size = 30.0  #mm

# prepare object points (0,0,0), (1,0,0), (2,0,0) ... scaled by square_size
objp = np.zeros((cRow * cCol, 3), np.float32)
objp[:, :2] = np.mgrid[0:cRow, 0:cCol].T.reshape(-1, 2)
objp = objp * square_size 

images = glob.glob('images_focusv1.3-1296x972/*.jpg') #source of images
imageSize = None 

# arrays to store object points and image points from all the images
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# flags for better detection
flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK

for iname in images:
    img = cv2.imread(iname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # try to find the corners
    board, corners = cv2.findChessboardCorners(gray, (cRow, cCol), flags=flags)

    # if found, add object points, image points
    if board:
        # refine the corner locations
        corners_acc = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        objpoints.append(objp)
        imgpoints.append(corners_acc)

        if not imageSize:
            imageSize = gray.shape[::-1]

		#visual feedback
        cv2.drawChessboardCorners(img, (cRow, cCol), corners_acc, board)
        cv2.imshow('Chessboard', img)
        cv2.waitKey() #how fast the images are shown
    else:
        print(f"error: Chessboard not found in {iname}")

cv2.destroyAllWindows()

#  check to prevent crash if no images are good
if len(objpoints) > 0:
    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
            objectPoints=objpoints,
            imagePoints=imgpoints,
            imageSize=imageSize,
            cameraMatrix=None,
            distCoeffs=None)
    
    np.savez('calib_focus_v1.3_1296x972_test30.npz',  #name of the final .npz file where the resulting params are saved
             cameraMatrix=cameraMatrix, 
             distCoeffs=distCoeffs, 
             rvecs=rvecs, 
             tvecs=tvecs)
    print(f"Success! Calibration saved. Used {len(objpoints)} images.")
else:
    print("Error: no images were successfully processed")
    
#reprojection error
    
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2SQR) / len(imgpoints2)
    mean_error += error
 
print( "total error: {}".format(np.sqrt(mean_error/len(objpoints))) )
print( "(the lower the better;<0.1 excellent, 0.3-0.5 good, 0.5-1.0 still acceptable probably" )
