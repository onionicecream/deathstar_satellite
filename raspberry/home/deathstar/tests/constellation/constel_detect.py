import cv2
import numpy as np
from picamera2 import Picamera2
import libcamera
import time

# data = np.load('calib_focus6.5.npz')
data = np.load('/home/deathstar/tests/camera_calibration/calib/calib_focus_v1.3_1296x972.npz')
cMat = data['cameraMatrix']
dcoeff = data['distCoeffs']


picam2 = Picamera2()

# FOCUS = 6.5
# config = picam2.create_video_configuration(
    # main={"size": (1536, 864), "format": "RGB888"},
    # controls={
        # "AfMode": libcamera.controls.AfModeEnum.Manual,
        # "LensPosition": FOCUS,
        # "FrameDurationLimits": (10000, 10000),
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
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

aruco_size_on_paper = 0.069  # individual marker side length (meters)

half = aruco_size_on_paper / 2
obj_pts_single = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float32)

while True:
    frame = picam2.capture_array()
    if frame is None:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    if ids is not None and ids.size > 0:
        ids_flat = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners)
        
        centers_3d = {}
        centers_2d = {}

        # estimate position of each marker center
        for i in range(len(ids_flat)):
            mid = ids_flat[i]
            img_pts = corners[i][0].astype(np.float32)
            
            success, rvec, tvec = cv2.solvePnP(obj_pts_single, img_pts, cMat, dcoeff, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            
            if success:
                centers_3d[mid] = tvec.flatten()
                
                cx = int(np.mean(img_pts[:, 0]))
                cy = int(np.mean(img_pts[:, 1]))
                centers_2d[mid] = (cx, cy)

                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {mid}", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # lines between markers
        detected_ids = list(centers_3d.keys())
        for i in range(len(detected_ids)):
            for j in range(i + 1, len(detected_ids)):
                id1, id2 = detected_ids[i], detected_ids[j]
                
                p1_2d = centers_2d[id1]
                p2_2d = centers_2d[id2]
                
                cv2.line(frame, p1_2d, p2_2d, (255, 0, 0), 2)
                
                dist_3d = np.linalg.norm(centers_3d[id1] - centers_3d[id2])
                
                mx = int((p1_2d[0] + p2_2d[0]) / 2)
                my = int((p1_2d[1] + p2_2d[1]) / 2)
                cv2.putText(frame, f"{dist_3d:.3f}m", (mx, my - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Constellation Finder", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
