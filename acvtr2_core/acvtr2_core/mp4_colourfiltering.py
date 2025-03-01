#!/usr/bin/python3

#Copyright [2025] [Robert Fudge]
#SPDX-FileCopyrightText: © 2025 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

import cv2
import numpy as np
import sys

def nothing(x):
    pass

cv2.namedWindow('Trackbars')
cv2.createTrackbar('H_min', 'Trackbars', 20, 180, nothing)
cv2.createTrackbar('H_max', 'Trackbars', 30, 180, nothing)
cv2.createTrackbar('S_min', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('S_max', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('V_min', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('V_max', 'Trackbars', 255, 255, nothing)

cap = cv2.VideoCapture(sys.argv[1])  # Replace with your video file
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
current_frame = 0
paused = True

while True:
    key = cv2.waitKeyEx(1)  #Use waitKeyEx for extended key codes
    
    #Handle key presses
    if key == ord('q'):
        break
    elif key == ord(' '):
        paused = not paused
    elif key == 81:  #Left arrow
        current_frame = max(0, current_frame - 30)
    elif key == 83:  #Right arrow
        current_frame = min(total_frames - 1, current_frame + 30)
    elif key == ord('r'):  #'r' key to replay from beginning
        current_frame = 0
    
    # Set video position and read frame
    cap.set(cv2.CAP_PROP_POS_FRAMES, current_frame)
    ret, frame = cap.read()
    if not ret:
        current_frame = 0
        continue
    
    # Apply HSV filtering
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos('H_min', 'Trackbars')
    h_max = cv2.getTrackbarPos('H_max', 'Trackbars')
    s_min = cv2.getTrackbarPos('S_min', 'Trackbars')
    s_max = cv2.getTrackbarPos('S_max', 'Trackbars')
    v_min = cv2.getTrackbarPos('V_min', 'Trackbars')
    v_max = cv2.getTrackbarPos('V_max', 'Trackbars')
    
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask = mask)
    
    cv2.imshow('Result', result)
    
    # Auto-advance if not paused
    if not paused:
        current_frame = min(current_frame + 1, total_frames - 1)

cap.release()
cv2.destroyAllWindows()