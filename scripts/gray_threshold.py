from __future__ import print_function
import cv2 as cv
import argparse
max_value = 255
min_value = 0
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_name = 'Low'
high_name = 'High'
high = max_value
low = min_value

def on_low_thresh_trackbar(val):
    global low
    global high
    low = val
    low = min(high-1, low)
    cv.setTrackbarPos(low_name, window_detection_name, low)

def on_high_thresh_trackbar(val):
    global low
    global high
    high = val
    high = max(high, low+1)
    cv.setTrackbarPos(high_name, window_detection_name, high)

parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('filepath', help='Image path', type=str)

args = parser.parse_args()
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)
cv.createTrackbar(low_name, window_detection_name , low, max_value, on_low_thresh_trackbar)
cv.createTrackbar(high_name, window_detection_name , high, max_value, on_high_thresh_trackbar)

filepath = args.filepath
frame = cv.imread(filepath, cv.IMREAD_COLOR)
if frame is None:
    raise ValueError("Cannot load file {}".format(filepath))


frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

while True:
    frame_threshold = cv.inRange(frame_gray, low, high)


    cv.imshow(window_capture_name, frame_gray)
    cv.imshow(window_detection_name, frame_threshold)

    key = cv.waitKey(1)
    if key == ord('q'):
        break

cv.destroyAllWindows()
