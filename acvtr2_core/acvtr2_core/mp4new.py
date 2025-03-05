#!/usr/bin/python3

import cv2
import numpy as np
import sys

# Program constants
VIDEO_WINDOW = 'ACVTR2 MP4 Viewer'
CONTROL_WINDOW = 'Control Panel'
COLORBAR_WINDOW = 'Color Bar'
NUM_FILTERS = 4  # Increased to 4 as per your maximum requirement
COLORS = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]  # Red, Green, Blue, Yellow
TRACKBAR_PARAMS = 8  # H_min, H_max, S_min, S_max, V_min, V_max, Min_Area, Max_Area
COLORBAR_HEIGHT = 50  # Fixed height for simplicity
BAR_HEIGHT = 20

def nothing(x):
    pass

# Initialize video capture
cap = cv2.VideoCapture(sys.argv[1])
if not cap.isOpened():
    print("Error: Could not open video file.")
    sys.exit(1)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
current_frame = 0
paused = True

# Initialize filters
filters = []
for i in range(NUM_FILTERS):
    filters.append({
        'H_min': 20 + i*10,
        'H_max': 30 + i*10,
        'S_min': 100,
        'S_max': 255,
        'V_min': 100,
        'V_max': 255,
        'min_area': 50,
        'max_area': 10000,
        'color': COLORS[i]
    })

# Create control window (size managed by OpenCV based on trackbars)
cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)

# Create trackbars
cv2.createTrackbar('Filter', CONTROL_WINDOW, 0, NUM_FILTERS-1, nothing)
cv2.createTrackbar('H_min', CONTROL_WINDOW, filters[0]['H_min'], 180, nothing)
cv2.createTrackbar('H_max', CONTROL_WINDOW, filters[0]['H_max'], 180, nothing)
cv2.createTrackbar('S_min', CONTROL_WINDOW, filters[0]['S_min'], 255, nothing)
cv2.createTrackbar('S_max', CONTROL_WINDOW, filters[0]['S_max'], 255, nothing)
cv2.createTrackbar('V_min', CONTROL_WINDOW, filters[0]['V_min'], 255, nothing)
cv2.createTrackbar('V_max', CONTROL_WINDOW, filters[0]['V_max'], 255, nothing)
cv2.createTrackbar('Min_Area', CONTROL_WINDOW, filters[0]['min_area'], 10000, nothing)
cv2.createTrackbar('Max_Area', CONTROL_WINDOW, filters[0]['max_area'], 10000, nothing)

# Create color bar window
cv2.namedWindow(COLORBAR_WINDOW, cv2.WINDOW_NORMAL)

# Video window
cv2.namedWindow(VIDEO_WINDOW, cv2.WINDOW_NORMAL)
ret, frame = cap.read()
if not ret:
    print("Error: Could not read first frame.")
    sys.exit(1)
height, width = frame.shape[:2]
cv2.resizeWindow(VIDEO_WINDOW, width, height)
cv2.resizeWindow(COLORBAR_WINDOW, width, COLORBAR_HEIGHT)

prev_filter_idx = 0

while True:
    key = cv2.waitKeyEx(1)

    # Handle controls
    if key == ord('q'):
        break
    elif key == ord(' '):
        paused = not paused
    elif key == 65361:  # Left arrow
        current_frame = max(0, current_frame - 30)
    elif key == 65363:  # Right arrow
        current_frame = min(total_frames - 1, current_frame + 30)
    elif key == ord('r'):
        current_frame = 0

    # Update frame
    cap.set(cv2.CAP_PROP_POS_FRAMES, current_frame)
    ret, frame = cap.read()
    if not ret:
        current_frame = 0
        continue

    # Update filter parameters
    current_filter_idx = cv2.getTrackbarPos('Filter', CONTROL_WINDOW)
    if current_filter_idx != prev_filter_idx:
        cv2.setTrackbarPos('H_min', CONTROL_WINDOW, filters[current_filter_idx]['H_min'])
        cv2.setTrackbarPos('H_max', CONTROL_WINDOW, filters[current_filter_idx]['H_max'])
        cv2.setTrackbarPos('S_min', CONTROL_WINDOW, filters[current_filter_idx]['S_min'])
        cv2.setTrackbarPos('S_max', CONTROL_WINDOW, filters[current_filter_idx]['S_max'])
        cv2.setTrackbarPos('V_min', CONTROL_WINDOW, filters[current_filter_idx]['V_min'])
        cv2.setTrackbarPos('V_max', CONTROL_WINDOW, filters[current_filter_idx]['V_max'])
        cv2.setTrackbarPos('Min_Area', CONTROL_WINDOW, filters[current_filter_idx]['min_area'])
        cv2.setTrackbarPos('Max_Area', CONTROL_WINDOW, filters[current_filter_idx]['max_area'])
        prev_filter_idx = current_filter_idx

    # Update current filter values from trackbars
    filters[current_filter_idx]['H_min'] = cv2.getTrackbarPos('H_min', CONTROL_WINDOW)
    filters[current_filter_idx]['H_max'] = cv2.getTrackbarPos('H_max', CONTROL_WINDOW)
    filters[current_filter_idx]['S_min'] = cv2.getTrackbarPos('S_min', CONTROL_WINDOW)
    filters[current_filter_idx]['S_max'] = cv2.getTrackbarPos('S_max', CONTROL_WINDOW)
    filters[current_filter_idx]['V_min'] = cv2.getTrackbarPos('V_min', CONTROL_WINDOW)
    filters[current_filter_idx]['V_max'] = cv2.getTrackbarPos('V_max', CONTROL_WINDOW)
    filters[current_filter_idx]['min_area'] = cv2.getTrackbarPos('Min_Area', CONTROL_WINDOW)
    filters[current_filter_idx]['max_area'] = cv2.getTrackbarPos('Max_Area', CONTROL_WINDOW)

    # Process frame
    result = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for filt in filters:
        lower = np.array([filt['H_min'], filt['S_min'], filt['V_min']])
        upper = np.array([filt['H_max'], filt['S_max'], filt['V_max']])

        # Handle hue wrap-around
        if lower[0] > upper[0]:
            mask1 = cv2.inRange(hsv, np.array([lower[0], lower[1], lower[2]]), np.array([180, upper[1], upper[2]]))
            mask2 = cv2.inRange(hsv, np.array([0, lower[1], lower[2]]), np.array([upper[0], upper[1], upper[2]]))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if filt['min_area'] <= area <= filt['max_area']:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(result, (x, y), (x + w, y + h), filt['color'], 2)

    # Draw progress bar
    cv2.rectangle(result, (0, height - BAR_HEIGHT), (width, height), (50, 50, 50), -1)
    progress_width = int((current_frame / (total_frames - 1)) * width) if total_frames > 1 else 0
    cv2.rectangle(result, (0, height - BAR_HEIGHT), (progress_width, height), (0, 255, 0), -1)

    # Add frame text
    frame_text = f"Frame: {current_frame} / {total_frames - 1}"
    text_size = cv2.getTextSize(frame_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
    text_x = (width - text_size[0]) // 2
    text_y = height - BAR_HEIGHT // 2 + text_size[1] // 2
    cv2.putText(result, frame_text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow(VIDEO_WINDOW, result)

    # Update color bar
    color_bar = np.zeros((COLORBAR_HEIGHT, width, 3), dtype=np.uint8)
    current_filter = filters[current_filter_idx]

    # Min color
    min_hsv = np.array([[current_filter['H_min'], current_filter['S_min'], current_filter['V_min']]], dtype=np.uint8).reshape(1, 1, 3)
    min_color = cv2.cvtColor(min_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    color_bar[:, :width//2] = min_color

    # Max color
    max_hsv = np.array([[current_filter['H_max'], current_filter['S_max'], current_filter['V_max']]], dtype=np.uint8).reshape(1, 1, 3)
    max_color = cv2.cvtColor(max_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    color_bar[:, width//2:] = max_color

    cv2.putText(color_bar, "Min HSV", (10, COLORBAR_HEIGHT-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(color_bar, "Max HSV", (width//2 + 10, COLORBAR_HEIGHT-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imshow(COLORBAR_WINDOW, color_bar)

    if not paused:
        current_frame = min(current_frame + 1, total_frames - 1)

cap.release()
cv2.destroyAllWindows()