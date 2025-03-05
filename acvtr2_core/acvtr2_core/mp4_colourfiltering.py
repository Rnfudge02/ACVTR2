#!/usr/bin/python3

#Copyright [2025] [Robert Fudge]
#SPDX-FileCopyrightText: © 2025 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

import cv2
import numpy as np
import sys

#Program constants
VIDEO_WINDOW = 'ACVTR2 MP4 Viewer'
CONTROL_WINDOW = 'Control Panel'
NUM_TRACKBARS = 7
TRACKBAR_HEIGHT = int(28.57 * NUM_TRACKBARS)  #Height of all trackbars
COLORBAR_HEIGHT = int(TRACKBAR_HEIGHT / 2)   #Height for the HSV color display bar
BAR_HEIGHT = 20

#Empty callback function
def nothing(x):
    pass

#Load the video and retrieve information
cap = cv2.VideoCapture(sys.argv[1])
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
current_frame = 0
paused = True

#Initial frame to get dimensions
ret, frame = cap.read()
if not ret:
    print("Error: Could not read video file")
    sys.exit(1)
cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset to start

#Set up window dimensions based on video width
height, width = frame.shape[:2]

#Create a single resizable window for trackbars
cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
cv2.resizeWindow(CONTROL_WINDOW, (width, TRACKBAR_HEIGHT + COLORBAR_HEIGHT))  #Include color bar height
cv2.createTrackbar('H_min', CONTROL_WINDOW, 20, 180, nothing)
cv2.createTrackbar('H_max', CONTROL_WINDOW, 30, 180, nothing)
cv2.createTrackbar('S_min', CONTROL_WINDOW, 100, 255, nothing)
cv2.createTrackbar('S_max', CONTROL_WINDOW, 255, 255, nothing)
cv2.createTrackbar('V_min', CONTROL_WINDOW, 100, 255, nothing)
cv2.createTrackbar('V_max', CONTROL_WINDOW, 255, 255, nothing)
cv2.createTrackbar('Min_Area', CONTROL_WINDOW, 50, 1000, nothing)

#Create the video window
cv2.namedWindow(VIDEO_WINDOW, cv2.WINDOW_NORMAL)
cv2.resizeWindow(VIDEO_WINDOW, width, height)

#Main program loop
while True:
    key = cv2.waitKeyEx(1)  #Use waitKeyEx for extended key codes
    
    #Handle user input
    if key == ord('q'):
        break
    elif key == ord(' '):
        paused = not paused
    elif key == 65361:  #Left arrow
        current_frame = max(0, current_frame - 30)
    elif key == 65363:  #Right arrow
        current_frame = min(total_frames - 1, current_frame + 30)
    elif key == ord('r'):  #'r' key to replay from beginning
        current_frame = 0
    
    #Set video position and read frame
    cap.set(cv2.CAP_PROP_POS_FRAMES, current_frame)
    ret, frame = cap.read()
    if not ret:
        current_frame = 0
        continue
    
    #Apply HSV filtering
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_min = cv2.getTrackbarPos('H_min', CONTROL_WINDOW)
    h_max = cv2.getTrackbarPos('H_max', CONTROL_WINDOW)
    s_min = cv2.getTrackbarPos('S_min', CONTROL_WINDOW)
    s_max = cv2.getTrackbarPos('S_max', CONTROL_WINDOW)
    v_min = cv2.getTrackbarPos('V_min', CONTROL_WINDOW)
    v_max = cv2.getTrackbarPos('V_max', CONTROL_WINDOW)
    min_area = cv2.getTrackbarPos('Min_Area', CONTROL_WINDOW)
    
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)
    
    #Convert original frame to grayscale and back for masking
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    #Create colored regions where mask is white
    colored = cv2.bitwise_and(frame, frame, mask=mask)
    
    #Invert mask to get background areas
    mask_inv = cv2.bitwise_not(mask)
    gray_background = cv2.bitwise_and(gray_bgr, gray_bgr, mask=mask_inv)
    
    #Combine colored foreground with grayscale background
    result = cv2.add(colored, gray_background)

    #Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    #Iterate through the contours
    for contour in contours:
        #Draw bounding boxes for blobs larger than min_area
        area = cv2.contourArea(contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)
    
    #Draw background rectangle for progress bar
    cv2.rectangle(result, (0, height - BAR_HEIGHT), (width, height), (50, 50, 50), -1)
    
    #Draw progress bar based on frame position
    if total_frames > 1:
        progress_width = int((current_frame / (total_frames - 1)) * width)
    else:
        progress_width = 0
    cv2.rectangle(result, (0, height - BAR_HEIGHT), (progress_width, height), (0, 255, 0), -1)
    
    #Add frame count text
    frame_text = f"Frame: {current_frame} / {total_frames - 1}"
    text_size = cv2.getTextSize(frame_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
    text_x = (width - text_size[0]) // 2
    text_y = height - BAR_HEIGHT // 2 + text_size[1] // 2
    cv2.putText(result, frame_text, (text_x, text_y), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow(VIDEO_WINDOW, result)

    #Create color bar showing min and max HSV values
    color_bar = np.zeros((COLORBAR_HEIGHT, width, 3), dtype=np.uint8)

    #Left half: minimum HSV color
    min_color = cv2.cvtColor(np.uint8([[[h_min, s_min, v_min]]]), cv2.COLOR_HSV2BGR)
    color_bar[:, :width//2] = min_color[0, 0]

    #Right half: maximum HSV color
    max_color = cv2.cvtColor(np.uint8([[[h_max, s_max, v_max]]]), cv2.COLOR_HSV2BGR)
    color_bar[:, width//2:] = max_color[0, 0]

    #Add labels
    cv2.putText(color_bar, "Min HSV", (10, COLORBAR_HEIGHT-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(color_bar, "Max HSV", (width//2 + 10, COLORBAR_HEIGHT-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


    #Display the combined image
    cv2.imshow(CONTROL_WINDOW, color_bar)
    
    #Auto-advance if not paused
    if not paused:
        current_frame = min(current_frame + 1, total_frames - 1)

#Clean up program prior to exit
cap.release()
cv2.destroyAllWindows()