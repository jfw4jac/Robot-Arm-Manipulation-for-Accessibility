#!/usr/bin/env python
'''
File: device_stream.py

Author  : James Wilson
Created : 04-11-2021
Updated : 05-02-2021

Course  : EAS 561 - Robotics Project

Description : Take screenshot of desktop and passes it as ROS topic.

'''
# Miscellaneous python imports
import numpy as np
import pyautogui

# Ros imports
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Opencv import
import cv2


# Function : main
# Description : Takes screenshot of portion of screen to send as ROS topic
#               message to the tracking node.
def main():
    rospy.init_node('device_stream', anonymous=False)
    image_pub = rospy.Publisher('device_image', Image, queue_size=1)
    try:
        while not rospy.is_shutdown():
            box = (0, 295, 960, 570)
            image_PIL = pyautogui.screenshot("delete.png", region=box)
            image_cv = cv2.cvtColor(np.asarray(image_PIL), cv2.COLOR_RGB2BGR)

            dim = (640, 480)
            image_cv = cv2.resize(image_cv, dim, interpolation=cv2.INTER_AREA)

            bridge = CvBridge()
            ros_frame = bridge.cv2_to_imgmsg(image_cv, 'bgr8')
            try:
                image_pub.publish(ros_frame)
            except CvBridgeError as error:
                print(error)

    except IOError:
        pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
