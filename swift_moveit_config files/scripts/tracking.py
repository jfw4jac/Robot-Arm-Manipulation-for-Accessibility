#!/usr/bin/env python
'''
File: tracking.py

Author  : James Wilson
Created : 03-05-2021
Updated : 05-02-2021

Course  : EAS 561 - Robotics Project

Description :

    1) Takes input from camera (webcam).

    2) Tracks a green object within the camera frame.

    3) Publish tracking coordinate to ROS topic.

'''
# Miscellaneous python imports
import time

# Opencv import
import cv2

# Ros imports
import rospy
from geometry_msgs.msg import Point

# File imports
from tracking_tools import *


# Function : main
# Description : Uses camera frames to determine if a green object is on screen.
#               If so, object center of mass positon is published to ROS topic.
def main():
    # Initialize ROS
    rospy.init_node('tracking', anonymous=True)
    pub_coord = rospy.Publisher('/tracking_coord', Point, queue_size=2)

    # Initialize FrameRate object
    FPS = FrameRate()
    show_fps = True  # Switch to toggle fps
    fps_color = 'green'

    # Initialize Webcam object
    cam = Webcam(multi_thread=False)  # Creates video opencv object
    cam_state = cam.camera_test_open()  # Boolean of camera open state

    # Tracking
    tracking_show = True
    track_color = (0, 255, 0)  # Green
    tracking_state = False  # Boolean of tracking state
    count_see = 0
    count_dont = 0
    pt_prev = (100000, 100000)

    # Publishing
    prev_pub_time = time.time()

    # Device Ghosting
    DEVICE = DeviceImageSubscriber()
    device_node_open = DEVICE.node_test_open()

    # Background subtraction
    background_sub = cv2.createBackgroundSubtractorMOG2(
        history=500, varThreshold=50, detectShadows=False)
    learning_rate = 0

    try:
        while cam_state and not rospy.is_shutdown():
            # Get camera frame
            frame = cam.get_camera_frame(flip=True)

            FPS.increase_framerate_counter()
            fps = FPS.get_frame_rate()

            # Blur BGR frame
            blur_gauss = 19
            frame_blur = cv2.GaussianBlur(frame, (blur_gauss, blur_gauss), 0)

            blur_bilat = 15
            frame_blur = cv2.bilateralFilter(frame_blur, blur_bilat, 50, 50)

            # Convert BGR to HSV
            HSV_blur = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

            # Green Mask
            green_L = (29, 86, 6)
            green_H = (64, 255, 255)
            green_mask_pre = cv2.inRange(HSV_blur, green_L, green_H)

            # Background subtraction
            fg_mask = background_sub.apply(frame_blur, learning_rate)
            double_mask = cv2.bitwise_and(green_mask_pre, fg_mask)

            # Morphological closing
            close_size = 15
            Kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT,
                                                     (close_size, close_size))
            double_mask_morph = cv2.morphologyEx(double_mask, cv2.MORPH_CLOSE, Kernel_close)

            # Find contours of final mask
            contours, _ = cv2.findContours(double_mask_morph, cv2.RETR_LIST,
                                           cv2.CHAIN_APPROX_SIMPLE)

            # Process contours
            if len(contours) != 0:
                largest_contour = max(contours, key=cv2.contourArea)

                # Check largest contour's area
                area = cv2.contourArea(largest_contour)
                min_contour_area = 425
                if area < min_contour_area:
                    obj_found = False
                elif area >= min_contour_area:
                    obj_found = True

                    # Compute center of mass
                    Moment = cv2.moments(largest_contour)
                    COM_X = int(Moment['m10'] / Moment['m00'])
                    COM_Y = int(Moment['m01'] / Moment['m00'])
                    pt_COM = (COM_X, COM_Y)

            elif len(contours) == 0:
                obj_found = False

            # Count sequential found object and misses
            if obj_found:
                count_see += 1
                count_dont = 0
            elif not obj_found:
                count_see = 0
                count_dont += 1

            # Initialize start and stop delays
            tracking_count_on = int(1*fps)  # Start delay
            tracking_count_off = int(0.75*fps)  # Stop delay

            # Enables tracking state
            if count_see >= tracking_count_on:
                tracking_state = True
                track_color = (255, 0, 0)  # Blue
                fps_color = 'blue'

            # Disables tracking state
            if tracking_state:
                if count_dont >= tracking_count_off:
                    tracking_state = False
                    count_see = 0
                    count_dont = 0
                    track_color = (0, 255, 0)  # Green
                    fps_color = 'green'

            # Check distance of current and previous COM and time difference
            if tracking_state and obj_found:
                dist_thresh = 4
                points_dist = eu_distance(pt_prev, pt_COM)

                time_thresh = 0.5
                new_pub_time = time.time()
                diff_pub_time = new_pub_time - prev_pub_time

                if points_dist >= dist_thresh or diff_pub_time >= time_thresh:
                    point_to_pub = Point()
                    point_to_pub.x = pt_COM[0]
                    point_to_pub.y = pt_COM[1]
                    point_to_pub.z = 1

                    pub_coord.publish(point_to_pub)  # Publish to ROS topic
                    pt_prev = pt_COM
                    prev_pub_time = new_pub_time

            # Video display section
            if tracking_show:
                img_contours = frame

                if device_node_open:
                    img_sub = DEVICE.subscribe()
                    amount_frame = 0.5
                    img_contours = blend_images(img_contours, img_sub, amount_frame)

                if show_fps:
                    img_contours = put_text(img_contours, str(fps),
                                            (0, 30), fps_color)
                if obj_found:
                    cv2.drawContours(img_contours, largest_contour,
                                     -1, track_color, 3)
                    cv2.circle(img_contours, pt_COM, 3, (0, 0, 255), -1)

                cv2.imshow('Tracking', img_contours)

            # Break
            key = cv2.waitKey(1)
            if key == 27:
                break

    # Clean up
    finally:
        # Key_Logger.stop()
        cam.camera_release()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
