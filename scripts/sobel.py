import cv2
import numpy as np

img = cv2.imread('image.jpg',0)

laplacian = cv2.Laplacian(img,cv2.CV_64F)
sobelx = cv2.Sobel(img,cv2.CV_64F,1,0,ksize=3)
sobely = cv2.Sobel(img,cv2.CV_64F,0,1,ksize=3)

cv2.imwrite('sobelx.jpg', sobelx)
cv2.imwrite('sobely.jpg', sobely)
cv2.imwrite('laplacian.jpg', laplacian)

