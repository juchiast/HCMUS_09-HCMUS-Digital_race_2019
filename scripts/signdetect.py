#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def detect():
    
    sign_publisher = rospy.Publisher("team405_control", String)
    rospy.init_node("car_control")

    while not rospy.is_shutdown():
        sign = input(">> ")
        if sign in ["R", "L", "S", "C"]:
            sign_publisher.publish(sign)


if __name__ == "__main__":
    detect()

