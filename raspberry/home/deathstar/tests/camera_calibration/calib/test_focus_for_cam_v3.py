#!/usr/bin/env python3
"""
this was for finidng the right focus when using the RPi camera v3;
not needed anymore for the fixed-lens camera
"""

import time
import collections
import cv2
from picamera2 import Picamera2

WIDTH          = 1536
HEIGHT         = 864
AVERAGE_FRAMES = 30   # rolling window size for the average

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
)
picam2.configure(config)

# continuous autofocus
picam2.set_controls({
    "AfMode":    2,   # 2 = continuous
    "AfTrigger": 0,
})

picam2.start()
time.sleep(1)

cv2.namedWindow("focus finder", cv2.WINDOW_NORMAL)
cv2.resizeWindow("focus finder", WIDTH // 2, HEIGHT // 2)

history  = collections.deque(maxlen=AVERAGE_FRAMES)
lens_pos = 0.0
avg_pos  = 0.0

while True:
    # capture frame and metadata together from the same request
    request = picam2.capture_request()
    frame = request.make_array("main")
    md = request.get_metadata()
    request.release()

    frame    = cv2.flip(frame, 1)

    lens_pos = md.get("LensPosition", 0.0)
    history.append(lens_pos)
    avg_pos  = sum(history) / len(history)

    cv2.rectangle(frame, (0, 0), (WIDTH, 80), (0, 0, 0), -1)

    cv2.putText(frame,
                f"lens position : {lens_pos:.2f}",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)

    cv2.putText(frame,
                f"avg ({AVERAGE_FRAMES}f)  : {avg_pos:.2f}",
                (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (180, 220, 255), 2)

    cv2.imshow("focus finder", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
