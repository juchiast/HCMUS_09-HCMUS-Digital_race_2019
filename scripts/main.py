#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from drivecar import drive

def imageCallback(ros_msg):
    np_arr = np.fromstring(ros_msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    #drive(image)

    # cv2.imshow("Image", image)
    # cv2.waitKey(1)



def signCallback(ros_msg):
    sign = ros_msg.data
    pass

def main():
    rospy.init_node("team405_node")
    
    imageSubcriber = rospy.Subscriber("/team405_image/compressed", CompressedImage, imageCallback)
    signSubscriber = rospy.Subscriber("/team405_sign", String, signCallback)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()