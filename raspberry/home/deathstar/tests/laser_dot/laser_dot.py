from picamera2 import Picamera2
import libcamera
import cv2
import numpy as np

picam2 = Picamera2()
# config = picam2.create_video_configuration(
    # main={"size": (1536, 864), "format": "BGR888"}, #this is rgb
    # # controls={"AfMode": libcamera.controls.AfModeEnum.Continuous}
# )
WIDTH = 1296
HEIGHT = 972
config = picam2.create_still_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "BGR888"}, #this is rgb
    raw={"size": (WIDTH, HEIGHT)},
)
picam2.configure(config)
picam2.start()

while True:
    frame = picam2.capture_array()

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    v = hsv[:, :, 2]
    s = hsv[:, :, 1]

    # find all bright white blobs
    mask = np.where((v >= 250) & (s <= 30), 255, 0).astype(np.uint8)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=3)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dot_position = None
    best_score = -1

    h, w = frame.shape[:2]
    
    # debug view: copy of mask with redness scores drawn on it
    debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    for c in contours:
        area = cv2.contourArea(c)
        if not (5 < area < 2000):
            continue

        ((cx, cy), radius) = cv2.minEnclosingCircle(c)
        cx, cy = int(cx), int(cy)
        search_r = int(radius) + 10 # 10 pixels around

        x1 = max(0, cx - search_r)
        x2 = min(w, cx + search_r)
        y1 = max(0, cy - search_r)
        y2 = min(h, cy + search_r)

        vicinity = frame[y1:y2, x1:x2]  # BGR frame
        r_channel = vicinity[:, :, 0].astype(float)
        g_channel = vicinity[:, :, 1].astype(float)
        b_channel = vicinity[:, :, 2].astype(float)

        # mask - exclude inner blob, keep only the ring
        local_cx = cx - x1
        local_cy = cy - y1
        ys, xs = np.ogrid[0:vicinity.shape[0], 0:vicinity.shape[1]]
        dist = np.sqrt((xs - local_cx)**2 + (ys - local_cy)**2)

        ring_mask = (dist > radius) & (dist <= search_r)  # only the outer ring

        # score only the ring pixels
        r_ring = r_channel[ring_mask]
        g_ring = g_channel[ring_mask]
        b_ring = b_channel[ring_mask]
        denom = (r_ring + g_ring + b_ring + 1) 
        # red_score = (r_ring - np.maximum(g_ring , b_ring) / denom).mean() if len(r_ring) > 0 else 0
        red_score = (r_ring - (g_ring + b_ring) / 2).mean() if len(r_ring) > 0 else 0
    
            
        if red_score > best_score:
            best_score = red_score
            dot_position = (cx, cy)
            best_radius = int(radius)
            
        # draw score above each blob on debug view
        cv2.putText(debug, f"{red_score:.1f}", (cx - 20, cy - int(radius) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(debug, (cx, cy), int(radius) + 4, (0, 255, 0), 1)

    if dot_position and best_score > 0:
        cv2.circle(frame, dot_position, best_radius + 4, (0, 255, 0), 2)
        cv2.circle(frame, dot_position, 3, (0, 255, 0), -1)
        cv2.putText(frame, f"Laser: {dot_position} score={best_score}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        print(f"Dot at: {dot_position}  red_score={best_score}")
    
    debug = cv2.resize(debug, (0,0), fx=0.4,fy=0.4)
    frame = cv2.resize(frame, (0,0), fx=0.4,fy=0.4)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # cv2.imshow("mask", mask)
    cv2.imshow("debug", debug)
    cv2.imshow("Laser Detection", frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
