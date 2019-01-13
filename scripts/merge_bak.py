import cv2
import numpy as np

image = cv2.imread("image.jpg", cv2.IMREAD_COLOR)

sobely = cv2.Sobel(image,cv2.CV_8U,0,1,ksize=3)
sobely_hsv = cv2.cvtColor(sobely, cv2.COLOR_BGR2HSV)
sobel_thresh = cv2.inRange(sobely_hsv, (0,0,0), (180, 255, 170))

sobel_thresh = cv2.bitwise_not(sobel_thresh)

image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
image_thresh = cv2.inRange(image_hsv, (0, 0, 180), (179, 65, 255))
merge = cv2.max(image_thresh, sobel_thresh)

cv2.imshow("result", sobel_thresh)
cv2.imshow("image", image_thresh)
cv2.imshow("merge", merge)
cv2.waitKey(0)
cv2.destroyAllWindows()
