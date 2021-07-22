#!/usr/bin/env python
'''
File: moving.py

Author  : James Wilson
Created : 03-03-2021
Updated : 05-02-2021

Course  : EAS 561 - Robotics Project

Description :

    1) Subscribes to ROS topic from tracking.py node (tracking coordinate).

    2) Compute and manage conversion of tracking coordinate from ROS topic.

    2) Manage user inputs

    3) Plan and execute Moveit command to robot.

'''
# Ros imports
import rospy

# File imports
from moving_tools import *


# Function : main
# Description : Gets physical coords from image_to_physical_coord function,
#               plans moveit cartesian path, and executes path.
def main():
    # Create Ros node
    rospy.init_node('moving_arm', anonymous=True)
    rate = rospy.Rate(60)

    # Create instance of Robot controller
    ROB = Robot("arm", initial_height=0.175, delta=0.015, delta_event=-0.005)
    ROB.set_touch_heights(new_touch_height=0.163)
    ROB.set_endeffector_offsets(x_end_offset=12.70, y_end_offset=19.05)

    # Create instance of UserInterface
    UI = UserInterface(delay=0.1)

    # Create instance of track subscriber
    SUB = TrackSubscriber()
    track_data = None

    try:
        while not rospy.is_shutdown():
            # Get UI mode, event, and command
            touch_state = UI.get_touch_state()
            key_event = UI.get_event()
            key_command = UI.get_command()
            key_mode = UI.get_mode()

            # Initialize cartesian path
            ROB.create_waypoints()

            # Subscribe to tracker ROS topic
            track_data_prev = track_data
            track_data = SUB.subscribe()
            is_track_data_new = SUB.is_data_new(track_data_prev, track_data)

            # Calibration mode
            if key_mode == 'calibration':

                # Calibration event handling
                if key_event is None:
                    pass
                elif key_event == 'add_calibration_point':
                    ROB.add_calibration_coord_current()
                    UI.clear_event()
                    num_points, max_points = ROB.get_calibration_counter()
                    print('\n    Calibration: Point %d of %d' % (num_points, max_points))
                elif key_event == 'restart_calibration':
                    ROB.clear_calibration_list()
                    UI.clear_event()
                    print('\nCALIBRATION RESTARTED!')
                elif key_event == 'reset_calibration':
                    max_rect_params = ROB.get_calibration_max_rectangular()
                    ROB.manual_calibration_update(max_rect_params)
                    ROB.clear_calibration_list()
                    UI.clear_event()
                    UI.reset_mode()
                    print('\nCALIBRATION RESET!')

                # Calibration state handling
                calibration_complete = ROB.is_calibration_complete()
                if calibration_complete:
                    ROB.automatic_four_point_calibration()
                    UI.clear_event()
                    UI.reset_mode()
                    UI.clear_move_command()
                    print('\nCALIBRATION COMPLETE')
                elif not calibration_complete:

                    # Calibration command handling
                    if key_command is not None:
                        teleop_pose = ROB.get_teleop_goal_pose(key_command)
                        ROB.append_waypoints(teleop_pose)
                        UI.clear_move_command()
                    elif key_command is None and is_track_data_new:
                        x_c, y_c = ROB.image_to_physical_coord(
                            track_data[0], track_data[1], range='max_rectangle')
                        _, _, z_c = ROB.get_current_coord_method()
                        calib_pose = ROB.quick_pose(x_c, y_c, z_c)
                        ROB.append_waypoints(calib_pose)

            # Normal mode
            elif key_mode == 'normal':

                # Calibration reset
                if key_event == 'reset_calibration':
                    max_rect_params = ROB.get_calibration_max_rectangular()
                    ROB.manual_calibration_update(max_rect_params)
                    ROB.clear_calibration_list()
                    UI.clear_event()
                    UI.reset_mode()
                    print('\nCALIBRATION RESET!')

                # Update goal poses based on track data
                ROB.update_track_poses(track_data, range='current_calibration')
                goal_touch, goal_above, goal_above_event = ROB.get_track_poses()

                # Normal event handling
                if key_event is None:
                    desired_goal = goal_touch if touch_state else goal_above
                    ROB.append_waypoints(desired_goal)
                elif key_event == 'tap':
                    ROB.append_waypoints(goal_above_event)
                    ROB.append_waypoints(goal_above)
                    UI.clear_event()
                elif key_event == 'double':
                    ROB.append_waypoints(goal_above_event)
                    ROB.append_waypoints(goal_above)
                    ROB.append_waypoints(goal_above_event)
                    ROB.append_waypoints(goal_above)
                    UI.clear_event()

            ROB.plan_execute(wait_cart=True)
            rate.sleep()

    # Clean up
    finally:
        # Return the Robot to default location
        ROB.create_waypoints()
        track_x, track_y, _ = SUB.return_default_coord()
        x_d, y_d = ROB.image_to_physical_coord(track_x, track_y, range='max_rectangle')
        z_d = ROB.above_height
        default_pose = ROB.quick_pose(x_d, y_d, z_d)
        ROB.append_waypoints(default_pose)
        ROB.plan_execute(wait_cart=True)
        UI.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
