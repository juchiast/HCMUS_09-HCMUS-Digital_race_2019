import cv2
import numpy as np
import math
def getPoints(x0, y0, x1, y1):
  points = []
  start, end = [(x0,y0), (x1, y1)] if y0 < y1 else [(x1, y1), (x0, y0)]
  for y in range(start[1], end[1] + 10, 10):
    dx = (y - start[1]) * abs(x0-x1) // abs(y0-y1)
    x = start[0] - dx if start[0] > end[0] else start[0] + dx
    points.append((x, y))

  start, end = [(x0,y0), (x1, y1)] if x0 < x1 else [(x1, y1), (x0, y0)]
  for x in range(start[0], end[0] + 10, 10):
    dy = (x - start[0]) * abs(y0 - y1) // abs(x0 - x1)
    y = start[1] - dy if start[1] > end[1] else start[1] + dy
    points.append((x,y))

  return points

image = cv2.imread("image.jpg", cv2.IMREAD_COLOR)

sobely = cv2.Sobel(image,cv2.CV_8U,0,1,ksize=3)
sobely_hsv = cv2.cvtColor(sobely, cv2.COLOR_BGR2HSV)
sobel_thresh = cv2.inRange(sobely_hsv, (0,0,0), (180, 255, 170))

sobel_thresh = cv2.bitwise_not(sobel_thresh)

image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
image_thresh = cv2.inRange(image_hsv, (0, 0, 180), (179, 65, 255))
merge = cv2.max(image_thresh, sobel_thresh)

cv2.imshow("merge", merge)

edges = cv2.Canny(merge, 200, 240, 3)
edges = edges[85:,:]
cv2.imshow("edges", edges)

lines_img = image[85:,:]

lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)
# print(lines)
if lines is not None:
  for line in lines:
    x1, y1, x2, y2 = line[0]
    print("Line: ", x1, y1, x2, y2)
    points = getPoints(x1, y1, x2, y2)
    print(points)
    for point in points:
        cv2.circle(lines_img, point, 1, (0,0,255), -1)
    cv2.line(lines_img, (x1, y1), (x2, y2), (0, 255, 0), 1)

#cv2.circle(lines_img, (50, 100), 3, (255, 0, 0), -1)
cv2.imshow("lines", lines_img)


cv2.waitKey(0)
cv2.destroyAllWindows()
