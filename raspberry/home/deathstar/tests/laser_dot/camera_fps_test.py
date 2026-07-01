import cv2
import numpy as np
from picamera2 import Picamera2
from scipy.spatial.transform import Rotation
import libcamera
import time
import matplotlib.pyplot as plt

# data = np.load('../calib.npz')
data = np.load('/home/deathstar/tests/camera_calibration/calib/calib_focus_v1.3_1296x972.npz')
cMat = data['cameraMatrix']
dcoeff = data['distCoeffs']

aruco_size_on_paper = 0.069
# FOCUS = 6.5

picam2 = Picamera2()
# config = picam2.create_video_configuration(
    # main={"size": (1536, 864), "format": "RGB888"},
    # controls={
        # "AfMode": libcamera.controls.AfModeEnum.Manual,
        # "LensPosition": FOCUS,
        # "FrameDurationLimits": (10000, 10000)
    # }
# )

WIDTH = 1296
HEIGHT = 972
config = picam2.create_still_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
    raw={"size": (WIDTH, HEIGHT)},
    buffer_count=2,
)

picam2.configure(config)
picam2.start()

prev_time = time.time()
fps_list = []

while True:
    frame = picam2.capture_array()

    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    fps_list.append(fps)

    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
    print(f"FPS: {fps:.1f}")
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()

fps_array = np.array(fps_list)
mean_fps = np.mean(fps_array)
std_fps = np.std(fps_array)

print(f"Mean FPS: {mean_fps:.2f}")
print(f"Standard Deviation: {std_fps:.2f}")

plt.plot(fps_list)
plt.title(f"FPS (Mean: {mean_fps:.2f}, Std: {std_fps:.2f})")
plt.xlabel("Frame")
plt.ylabel("FPS")
plt.show()
