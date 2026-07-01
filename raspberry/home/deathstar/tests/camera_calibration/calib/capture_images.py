#!/usr/bin/env python3
"""
capture the images needed for camera calibration
"""


import os
import time
import cv2
import numpy as np
from picamera2 import Picamera2
 
# # --- config --- camera v3 with autofocus
# FOCUS = 6.5 # dioptres, roughly
# NUM_IMAGES  = 30
# COUNTDOWN_S = 2  # seconds between space press and actual capture
# WIDTH = 1536
# HEIGHT = 864

# --- config ---fixed lense on model v1.3
NUM_IMAGES  = 30
COUNTDOWN_S = 1 # seconds between space press and actual capture
WIDTH = 1296
HEIGHT = 972

FOCUS = "v1.3-1296x972_2" #this is also used as name of directory
 
FOLDER = f"images_focus{FOCUS}"
os.makedirs(FOLDER, exist_ok=True)
 
picam2 = Picamera2()
 
config = picam2.create_still_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
    raw={"size": (WIDTH, HEIGHT)},
    buffer_count=2,
)
picam2.configure(config)
 
# # lock focus (0 = manual mode), then set position
# picam2.set_controls({
    # "AfMode": 0,
    # "LensPosition": FOCUS,
# })
 
picam2.start()
time.sleep(1)  
 
print(f"focus    : {FOCUS} diopters")
print(f"saving to: {FOLDER}/")
print(f"images   : {NUM_IMAGES}")
print("press [space] to start countdown. [q] to quit.\n")
 
cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
cv2.resizeWindow("preview", WIDTH // 2, HEIGHT // 2)
 
captured = 0
countdown_end = None # timestamp when capture fires, None = idle
flash_until = None # timestamp until we show the "saved" flash
 
while captured < NUM_IMAGES:
    frame = picam2.capture_array("main")
    frame = cv2.flip(frame, 1) # mirror so rotating the board feels natural
 
    now = time.monotonic()
    
    if flash_until and now >= flash_until:
        flash_until = None
 
    if flash_until and now < flash_until:
        cv2.putText(frame, f"saved! ({captured}/{NUM_IMAGES})",
                    (WIDTH // 2 - 220, HEIGHT // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 220, 80), 8)
 
    elif countdown_end is not None:
        # start countdown
        remaining = countdown_end - now
        if remaining > 0:
            tick = int(np.ceil(remaining))
            text = str(tick)
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 8
            thick = 16
            (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
            cx = (WIDTH  - tw) // 2
            cy = (HEIGHT + th) // 2
            cv2.putText(frame, text, (cx + 5, cy + 5), font, scale, (0, 0, 0),      thick + 6)
            cv2.putText(frame, text, (cx,cy), font, scale, (255, 255, 255), thick)
        else:
            # countdown elapse
            filepath = os.path.join(FOLDER, f"calib_{captured + 1:03d}.jpg")
            picam2.capture_file(filepath)
            captured += 1
            countdown_end  = None
            flash_until = now + 0.6
            print(f"  -> {filepath}  ({captured}/{NUM_IMAGES})")
 
    else:
        # idle
        remaining_shots = NUM_IMAGES - captured
        hint = f"[space] capture  |  {captured}/{NUM_IMAGES} done | {remaining_shots} left"
        cv2.putText(frame, hint, (30, HEIGHT - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0),       4)
        cv2.putText(frame, hint, (30, HEIGHT - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (200, 200, 200), 2)
 
    cv2.imshow("preview", frame)
 
    key = cv2.waitKey(1) & 0xFF
    if key == ord(' ') and countdown_end is None and flash_until is None:
        countdown_end = time.monotonic() + COUNTDOWN_S
        print(f"[{captured + 1:02d}/{NUM_IMAGES}] countdown started...")
    elif key == ord('q'):
        print("quit.")
        break
 
picam2.stop()
cv2.destroyAllWindows()
print(f"\ndone. {captured}/{NUM_IMAGES} images captured in '{FOLDER}/'")
 
