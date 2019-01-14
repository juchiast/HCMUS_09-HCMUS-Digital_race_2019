import cv2
import numpy as np

image = cv2.imread("image.jpg", cv2.IMREAD_COLOR)
image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
image_thresh = cv2.inRange(image_hsv, (0, 0, 180), (179, 65, 255))

cv2.imshow("image", image_thresh)

cv2.waitKey(0)
cv2.destroyAllWindows()
