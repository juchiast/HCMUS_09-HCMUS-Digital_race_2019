import rospy
import numpy as np
import math
from std_msgs.msg import Float32

carPos = (120, 300)
steer_publisher = rospy.Publisher("Team1_steerAngle", Float32, queue_size=10)
speed_publisher = rospy.Publisher("Team1_speed", Float32, queue_size=10)

def errorAngle(dst: tuple):
    if dst[0] == carPos[0]:
        return 0
    if dst[1] == carPos[1]:
        return -90 if dst[0] < carPos[0] else 90
    dx = dst[0] - carPos[0]
    dy = carPos[1] - dst[1]
    if (dx < 0):
        return -math.atan(-dx / dy) * 180 / math.pi
    else:
        return math.atan(dx / dy) * 180 / math.pi
    

def drive(image):
    print("Drive.............")
    error = 0.0
    velocity = 50.0


    angle = error
    speed = velocity

    #steer_publisher.publish(angle)
    #speed_publisher.publish(speed)

    pass


def turn(image):
    pass

