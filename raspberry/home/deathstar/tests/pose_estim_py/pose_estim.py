import cv2
import numpy as np
from picamera2 import Picamera2
from scipy.spatial.transform import Rotation
import libcamera
import time

# data = np.load('../calib/calib_focus6.5.npz')
data = np.load('/home/deathstar/tests/camera_calibration/calib/calib_focus_v1.3_1296x972.npz')
cMat = data['cameraMatrix']
dcoeff = data['distCoeffs']

aruco_size_on_paper = 0.069  # meters

picam2 = Picamera2()
# FOCUS = 6.5
# config = picam2.create_video_configuration(
    # main={"size": (1536, 864), "format": "RGB888"},
    # controls={
        # # "AfMode": libcamera.controls.AfModeEnum.Manual,
        # # "LensPosition": FOCUS,
        # "FrameDurationLimits": (10000, 10000)  # high fps
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
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)  # init once, not in loop

half = aruco_size_on_paper / 2
obj_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0],
], dtype=np.float32)

alpha = 0.3  # low-pass filter strength
rvec_filtered = None

prev_time = 0

while True:
    frame = picam2.capture_array()

    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time

    if frame is None:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None and ids.size > 0:
        corner = corners[0][0]
        img_points = corner.astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, cMat, dcoeff,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if success:
            # low-pass filter on rvec before converting
            if rvec_filtered is None:
                rvec_filtered = rvec.copy()
            else:
                rvec_filtered = alpha * rvec + (1 - alpha) * rvec_filtered

            # rvec to quaternion [x, y, z, w]
            q = Rotation.from_rotvec(rvec_filtered.flatten()).as_quat()

            # pointing error from tvec (camera frame)
            tx, ty, tz = tvec.flatten()
            yaw_error   = np.arctan2(tx, tz) # rad, positive = marker is right
            pitch_error = np.arctan2(-ty, tz) # rad, positive = marker is up

            dist = np.linalg.norm(tvec) * 100

            cv2.drawFrameAxes(frame, cMat, dcoeff, rvec, tvec, half)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.putText(frame, f"Dist: {dist:.2f} cm",   (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            cv2.putText(frame, f"FPS: {fps:.1f}",  (10, 70),  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            cv2.putText(frame, f"Yaw err: {np.degrees(yaw_error):.2f} deg",   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            cv2.putText(frame, f"Pitch err: {np.degrees(pitch_error):.2f} deg", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            cv2.putText(frame, f"Q: [{q[0]:.2f}, {q[1]:.2f}, {q[2]:.2f}, {q[3]:.2f}]", (10, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 0), 2)
            # extract Euler angles (Rx, Ry, Rz) in degrees from the rotation vector
            rot_angles = Rotation.from_rotvec(rvec_filtered.flatten()).as_euler('xyz', degrees=True)
            # total angular deviation (magnitude of the rotation vector)
            total_rot_error = np.degrees(np.linalg.norm(rvec_filtered))

            
            cv2.putText(frame, f"Rot (Rx,Ry,Rz): [{rot_angles[0]:.2f}, {rot_angles[1]:.2f}, {rot_angles[2]:.2f}] deg", (10, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0))
            cv2.putText(frame,f"Total Rot Err:  {total_rot_error:.2f} deg", (10, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0))

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
