#!/usr/bin/env python
'''
File: moving_tools.py

Author  : James Wilson
Created : 04-24-2021
Updated : 05-02-2021

Course  : EAS 561 - Robotics Project

Description : Provide classes and functions to tracking.py

'''
# Miscellaneous python imports
import time
from threading import Thread
import numpy as np  # pip

# Opencv import
import cv2

# Ros imports
import rospy
from sensor_msgs.msg import Image
import rosnode
from cv_bridge import CvBridge


# Object : Webcam
# Description : Creates camera object and open camera stream.
class Webcam:
    def __init__(self, cam_number=0, resolution=(640, 480), multi_thread=False):
        # Initialize arguments
        self.cam_number = cam_number
        self.resolution = resolution
        self.multi_thread = multi_thread

        # Initialize camera stream
        self.camera = cv2.VideoCapture(self.cam_number, cv2.CAP_DSHOW)
        self.camera.open(self.cam_number)

        # Get first frame
        self.bool_val, self.frame = self.camera.read()

        # Acceptable camera resolutions
        #       (160,120), (320,240), (640,480), (1280,720)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.camera.set(cv2.CAP_PROP_FPS, 15)

        # Threading
        self.stopped = False
        if self.multi_thread:
            self.thread = Thread(target=self.update, args=())
            self.thread.daemon = True
            self.thread.start()

    def camera_release(self):  # Release camera and thread
        self.camera.release()
        self.stopped = True
        cv2.destroyAllWindows()

    def camera_test_open(self):  # Test if camera is open
        if self.camera.isOpened():
            camera_state = True
        elif not self.camera.isOpened():
            camera_state = False
        return camera_state

    def update(self):  # For multithreading frame updating
        while True:
            if self.stopped:
                return

            self.bool_value, self.frame = self.camera.read()

    def get_camera_frame(self, flip=False):  # Returns camera frame
        if self.multi_thread:
            if flip:
                self.frame = cv2.flip(self.frame, 1)
            return self.frame

        self.bool_value, self.frame = self.camera.read()
        if flip:
            self.frame = cv2.flip(self.frame, 1)
        return self.frame


# Function : eu_distance
# Description : Calculate euclidean distance of two points.
def eu_distance(pt1, pt2):
    pt1 = np.asarray(pt1)
    pt2 = np.asarray(pt2)
    diff_pt = pt1 - pt2
    dist = np.sqrt(np.dot(diff_pt.T, diff_pt))
    return dist


# Function : blend_images
# Description : Blend two images together dependent on blend ratio (ratio_1)
def blend_images(frame1, frame2, ratio_1=0.5):
    ratio_2 = 1 - ratio_1
    frame_blend = cv2.addWeighted(frame1, ratio_1, frame2, ratio_2, 0.0)
    return frame_blend


# Function : put_text
# Description : Places text on image with opencv function "putText"
def put_text(image, text, location, color_name, fontScale=1):
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 2

    if color_name == 'blue':
        color = (255, 0, 0)
    elif color_name == 'green':
        color = (0, 255, 0)
    elif color_name == 'red':
        color = (0, 0, 255)
    elif color_name == 'white':
        color = (255, 255, 255)
    else:
        color = (255, 255, 255)

    image_with_text = cv2.putText(image, text, location, font,
                                  fontScale, color, thickness, cv2.LINE_AA)
    return image_with_text


# Object : FrameRate
# Description : Calcuates the fps.
class FrameRate:
    def __init__(self, frame_avg_num=10):
        self.frame_avg_num = frame_avg_num
        self.prev_frame_time = time.time()
        self.frame_count = 0
        self.frame_rate = 15

    def increase_framerate_counter(self):
        self.frame_count += 1

    def update_framerate(self):
        if self.frame_count >= self.frame_avg_num:
            new_frame_time = time.time()
            self.frame_rate = int(self.frame_avg_num*(1/(new_frame_time - self.prev_frame_time)))
            self.prev_frame_time = new_frame_time
            self.frame_count = 0

    def get_frame_rate(self):
        self.update_framerate()
        return self.frame_rate


# Object : DeviceImageSubscriber
# Description : Creates object which will subscribe to device node ROS topic
class DeviceImageSubscriber:
    def __init__(self, ros_node='/device_stream_node'):
        # Initialize arguments
        self.ros_node = ros_node

        # Check if node is open
        self.node_open = False
        if self.ros_node in rosnode.get_node_names():
            self.node_open = True

        self.image = np.zeros((480, 640, 3)).astype('uint8')

    def node_test_open(self):
        return self.node_open

    def callback_image(self, data):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(data).astype('uint8')

    def subscribe(self):
        if self.node_open:
            rospy.Subscriber('device_image', Image, self.callback_image)
            return self.image
        else:
            print('    Error: Node %s not open' % (self.ros_node))
            return self.image
