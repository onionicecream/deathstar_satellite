'''
calculates distance to a single aruco marker, applying the camera distortion correction
'''
import cv2
import numpy as np
from picamera2 import Picamera2
import libcamera
import time

# load and extract the variables for camera calib
# data = np.load('../calib/calib.npz')
data = np.load('/home/deathstar/tests/camera_calibration/calib/calib_focus_v1.3_1296x972.npz')
cMat = data['cameraMatrix']
dcoeff = data['distCoeffs']

aruco_size_on_paper = 0.069 #in meters

picam2 = Picamera2()
# config = picam2.create_video_configuration(
    # main={"size": (1536, 864), "format": "RGB888"},
    # controls={
    # "AfMode": libcamera.controls.AfModeEnum.Continuous, #autofocus
    # "FrameDurationLimits": (10000, 10000)  # 60 fps (in microseconds)
    # }
# )

# --- config ---fixed lense on model v1.3
WIDTH = 1296
HEIGHT = 972
config = picam2.create_still_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
    raw={"size": (WIDTH, HEIGHT)},
    buffer_count=2,
)
picam2.configure(config)
picam2.start()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()

prev_time = 0
while True:
    frame = picam2.capture_array()
    
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
	
    ret = frame is not None
    if not ret: break

    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None and ids.size > 0:
        id = ids[0]
        corner = corners[0][0]

        # definition of 3D object points for the marker (in marker's local coordinate system)
        half = aruco_size_on_paper / 2
        obj_points = np.array([
            [-half,  half, 0], # top-left
            [ half,  half, 0], # top-right
            [ half, -half, 0], # bottom-right
            [-half, -half, 0], # bottom-left
        ], dtype=np.float32)

        # image points from detected corners
        img_points = corner.astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            cMat,
            dcoeff,
            flags=cv2.SOLVEPNP_IPPE_SQUARE  # best flag for square planar targets
        )

        if success:
            dist = np.linalg.norm(tvec) * 100
            cv2.drawFrameAxes(frame, cMat, dcoeff, rvec, tvec, aruco_size_on_paper / 2)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.putText(frame, f"Distance: {dist:.2f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
picam2.stop()
cv2.destroyAllWindows()
