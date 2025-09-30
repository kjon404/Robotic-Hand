import cv2
import numpy as np

# MediaPipe Hands connections (21 points)
HAND_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),        # thumb
    (0,5),(5,6),(6,7),(7,8),        # index
    (0,9),(9,10),(10,11),(11,12),   # middle
    (0,13),(13,14),(14,15),(15,16), # ring
    (0,17),(17,18),(18,19),(19,20)  # pinky
]

def draw_hand_skeleton(img, landmarks21, color=(0,255,0)):
    h, w = img.shape[:2]
    pts = [(int(x*w), int(y*h)) for (x,y,_) in landmarks21]

    # bones
    for a, b in HAND_CONNECTIONS:
        cv2.line(img, pts[a], pts[b], color, 2,  cv2.LINE_AA)

    # joints
    for p in pts:
        cv2.circle(img, p, 3, color, -1, cv2.LINE_AA)

