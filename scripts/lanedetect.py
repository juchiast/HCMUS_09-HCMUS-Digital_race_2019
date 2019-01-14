#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

def lane_detect(image: np.ndarray):

    # img = np.zeros((400, 400), np.uint8)
    # img[100:300, 100:300] = 255

    # im2, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # cntImage = np.zeros((400, 400, 3), dtype=np.uint8)
    # cv2.drawContours(cntImage, contours, -1, (0,255,0), 3)

    # cv2.imshow("test", im2)
    # cv2.imshow("contour", cntImage)

    # roi = 

    grayImage = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    minthreshold, maxthreshold = 125, 255
    binaryImage = cv2.inRange(grayImage, minthreshold, maxthreshold)

    sobelx = cv2.Sobel(binaryImage,cv2.CV_64F,1,0,ksize=5)
    sobely = cv2.Sobel(binaryImage,cv2.CV_64F,0,1,ksize=5)

    
    im2, contours, hierarchy = cv2.findContours(binaryImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    cntImage = np.zeros(image.shape, dtype=np.float)
    area = [cv2.contourArea(contour) for contour in contours]
    index = area.index(max(area))
    cv2.drawContours(cntImage, contours, index, (0, 255, 0), 1)


    cv2.imshow("Original", image)
    cv2.imshow("Binary", binaryImage)
    cv2.imshow("SobelX", sobelx)
    cv2.imshow("SobelY", sobely)
    cv2.imshow("Contour", cntImage)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    pass

if __name__ == "__main__":
    image = cv2.imread("image_sign.jpg", cv2.IMREAD_COLOR)
    lane_detect(image)