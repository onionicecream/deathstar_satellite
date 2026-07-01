import cv2
import numpy as np
from picamera2 import Picamera2
from scipy.spatial.transform import Rotation
import libcamera
import time

data = np.load('/home/deathstar/tests/camera_calibration/calib/calib_focus_v1.3_1296x972.npz')
cMat = data['cameraMatrix']
dcoeff = data['distCoeffs']

# FOCUS = 6.5
picam2 = Picamera2()
config = picam2.create_video_configuration(
    # main={"size": (1536, 864), "format": "RGB888"},
    # main={"size": (700, 400), "format": "RGB888"},
    main={"size": (1296, 972), "format": "RGB888"},
     # main={"size": (1920, 1080), "format": "RGB888"},
    # controls={
        # "AfMode": libcamera.controls.AfModeEnum.Continuous,
        # # "LensPosition": FOCUS,
        # # "ExposureTime": 100000,
        # "FrameDurationLimits": (10000, 10000),
    # }
)
picam2.configure(config)
picam2.start()

# ---------------------------------------------------------------------------
# constellation definition
# ---------------------------------------------------------------------------
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)


# X axis = leftwards, Y axis = downwards, Z = out of the page (right-hand).

W = -0.12 / 2   # half-width
H = -0.207 / 2   # half-height

MARKER_CENTERS_IN_CONSTELLATION = {
    4: np.array([-W,  H, 0.0]), 
    3: np.array([ W,  H, 0.0]), 
    2: np.array([ W, -H, 0.0]), 
    1: np.array([-W, -H, 0.0]),
}

aruco_size_on_paper = 0.069   #individual marker side length (metres)
half = aruco_size_on_paper / 2

def marker_obj_points(cx, cy):
    #4 corners of one marker expressed in constellation-centre coordinates
    return np.array([
        [cx - half,  cy + half, 0],
        [cx + half,  cy + half, 0],
        [cx + half,  cy - half, 0],
        [cx - half,  cy - half, 0],
    ], dtype=np.float32)

MARKER_OBJ_POINTS = {
    mid: marker_obj_points(c[0], c[1])
    for mid, c in MARKER_CENTERS_IN_CONSTELLATION.items()
}

# translation vectors from each marker center to constellation center
MARKER_TO_CENTER_T = {
    mid: -c   # vector from marker to center = negative of marker position
    for mid, c in MARKER_CENTERS_IN_CONSTELLATION.items()
}

# ---------------------------------------------------------------------------
#main loop
# ---------------------------------------------------------------------------
prev_time = time.time()

while True:
    frame = picam2.capture_array()
    curr_time = time.time()
    fps = 1.0 / max(curr_time - prev_time, 1e-6)
    prev_time = curr_time

    if frame is None:
        break

    corners, ids, _ = detector.detectMarkers(frame)

    overlay_text = []

    if ids is not None and ids.size > 0:
        obj_pts = []
        img_pts = []
        detected_ids = []

        # get points for all visible markers
        for i, mid in enumerate(ids.flatten()):
            if mid in MARKER_OBJ_POINTS:
                obj_pts.append(MARKER_OBJ_POINTS[mid])
                img_pts.append(corners[i][0])
                detected_ids.append(mid)

        if obj_pts:
            obj_pts = np.vstack(obj_pts).astype(np.float32)
            img_pts = np.vstack(img_pts).astype(np.float32)

            # solvePnP with all points from detected markers
            success, rvec_target, tvec_target = cv2.solvePnP(
                obj_pts, img_pts, cMat, dcoeff, flags=cv2.SOLVEPNP_SQPNP
            )

            if success:
                # target center coordinates relative to camera lens
                tx, ty, tz = tvec_target.flatten()
                
                # calc original line-of-sight angular errors
                yaw_error = np.arctan2(tx,  tz)
                pitch_error = np.arctan2(-ty, tz)
                dist_cm = np.linalg.norm(tvec_target) * 100

                # camera position in target world coordinates
                R_target, _ = cv2.Rodrigues(rvec_target)
                tvec_camera = -R_target.T @ tvec_target
                cx, cy, cz = tvec_camera.flatten()
                
                #roll error from the rotation matrix
                R_target, _ = cv2.Rodrigues(rvec_target)
                roll_error = np.arctan2(R_target[1, 0], R_target[0, 0])
                if roll_error > 0:
                    roll_error = np.pi - roll_error
                else:
                    roll_error = - (np.pi + roll_error)

                # visualisations
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, cMat, dcoeff, rvec_target, tvec_target, half * 2)

                overlay_text = [
                    (f"Markers Visible: {sorted(detected_ids)}",(10,  30), (200, 200,0)),
                    (f"FPS: {fps:.1f}",(10,70), (0, 255,0)),
                    (f"Dist to Center: {dist_cm:.1f} cm",  (10, 110), (0,0,255)),
                    (f"Yaw Error:   {np.degrees(yaw_error):.2f} deg",(10,150), (255,0,0)),
                    (f"Pitch Error: {np.degrees(pitch_error):.2f} deg",(10,190), (255, 0,0)),
                    (f"Roll Error:  {np.degrees(roll_error):.2f} deg",(10,230), (255, 0,0)),
                    (f"Cam Pos WRT Target XYZ: [{cx*100:.1f}, {cy*100:.1f}, {cz*100:.1f}] cm",(10,250),(0,255,255)),
                ]
    else:
        overlay_text = [
            ("No markers detected",(10,30),(0,0,255)),
            (f"FPS: {fps:.1f}",(10,70),(0,255,0)),
        ]

    for text, pos, color in overlay_text:
        cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    cv2.imshow("constellation pose", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
