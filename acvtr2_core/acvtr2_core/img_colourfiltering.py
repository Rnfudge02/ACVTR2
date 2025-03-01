#!/usr/bin/python3

#Copyright [2025] [Robert Fudge]
#SPDX-FileCopyrightText: © 2025 Robert Fudge <rnfudge@mun.ca>
#SPDX-License-Identifier: {Apache-2.0}

#Imports
import cv2
import numpy as np

#Empty callback function
def nothing(x):
    pass

#Control Panel
cv2.namedWindow('Trackbars')
cv2.createTrackbar('H_min', 'Trackbars', 20, 180, nothing)
cv2.createTrackbar('H_max', 'Trackbars', 30, 180, nothing)
cv2.createTrackbar('S_min', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('S_max', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('V_min', 'Trackbars', 100, 255, nothing)
cv2.createTrackbar('V_max', 'Trackbars', 255, 255, nothing)

img = cv2.imread("buoy_img.png")
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

while True:
    h_min = cv2.getTrackbarPos('H_min', 'Trackbars')
    h_max = cv2.getTrackbarPos('H_max', 'Trackbars')
    s_min = cv2.getTrackbarPos('S_min', 'Trackbars')
    s_max = cv2.getTrackbarPos('S_max', 'Trackbars')
    v_min = cv2.getTrackbarPos('V_min', 'Trackbars')
    v_max = cv2.getTrackbarPos('V_max', 'Trackbars')

    lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
    upper = np.array([h_max, s_max, v_max], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow('Result', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()