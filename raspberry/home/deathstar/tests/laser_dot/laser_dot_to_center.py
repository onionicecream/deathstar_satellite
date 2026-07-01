from picamera2 import Picamera2
import cv2
import numpy as np
import math


picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": (1536, 864), "format": "BGR888"} 
)
picam2.configure(config)
picam2.start()

while True:
    frame = picam2.capture_array()
    h, w = frame.shape[:2]
    
    # cam center
    cam_center = (w // 2, h // 2)
    # draw  crosshair at the center of the frame
    cv2.drawMarker(frame, cam_center, (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

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
    best_radius = 0
    
    # debug view
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

        vicinity = frame[y1:y2, x1:x2]  
        r_channel = vicinity[:, :, 0].astype(float)
        g_channel = vicinity[:, :, 1].astype(float)
        b_channel = vicinity[:, :, 2].astype(float)

        # mask
        local_cx = cx - x1
        local_cy = cy - y1
        ys, xs = np.ogrid[0:vicinity.shape[0], 0:vicinity.shape[1]]
        dist_array = np.sqrt((xs - local_cx)**2 + (ys - local_cy)**2)

        ring_mask = (dist_array > radius) & (dist_array <= search_r)  

        # score only the ring pixels
        r_ring = r_channel[ring_mask]
        g_ring = g_channel[ring_mask]
        b_ring = b_channel[ring_mask]
        red_score = (r_ring - (g_ring + b_ring) / 2).mean() if len(r_ring) > 0 else 0

        if red_score > best_score:
            best_score = red_score
            dot_position = (cx, cy)
            best_radius = int(radius)
            
        cv2.putText(debug, f"{red_score:.1f}", (cx - 20, cy - int(radius) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(debug, (cx, cy), int(radius) + 4, (0, 255, 0), 1)

    if dot_position and best_score > 0:
        # calc pixel distance from the center to the dot
        pixel_distance = math.hypot(dot_position[0] - cam_center[0], dot_position[1] - cam_center[1])
        
        # draw a line connecting center to dot
        cv2.line(frame, cam_center, dot_position, (0, 255, 255), 2)
        
        # dot highlights
        cv2.circle(frame, dot_position, best_radius + 4, (0, 255, 0), 2)
        cv2.circle(frame, dot_position, 3, (0, 255, 0), -1)
        
        # vis info
        cv2.putText(frame, f"Laser: {dot_position} score={best_score:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"Distance from Center: {pixel_distance:.1f} px", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    
        print(f"Dot at: {dot_position} | Distance: {pixel_distance:.1f} px | red_score={best_score:.1f}")

    cv2.imshow("debug", debug)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("Laser Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
