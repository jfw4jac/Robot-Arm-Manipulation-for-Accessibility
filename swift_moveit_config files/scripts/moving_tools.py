#!/usr/bin/env python
'''
File: moving_tools.py

Author  : James Wilson
Created : 04-24-2021
Updated : 05-03-2021

Course  : EAS 561 - Robotics Project

Description : Provide classes to moving.py

'''
# Miscellaneous python imports
import math
from threading import Thread
import tty
import sys
import termios
import select
import collections
import numpy as np  # pip

# Moveit imports
import moveit_commander

# Ros imports
import rospy
from geometry_msgs.msg import Pose, Point


# Object : UserInterface
# Description : Creates object which will handle user's inputs
class UserInterface:
    def __init__(self, delay=0.1, num_calibration=4):
        # Initialize object variables
        self.delay = delay
        self.num_calibration = num_calibration
        self.stopped = False

        # Begin thread
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

        self.default = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Initialize user variables
        self.key = None
        self.Event = None
        self.move_command = None
        self.touch_state = False
        self.Mode = 'normal'

        # Print possible user interface keys
        self.print_possible_UserInterface()

    def update(self):  # For multithreading updating
        while True:
            if self.stopped:
                return

            # tty.setcbreak(sys.stdin.fileno())
            terminal_ready = sys.stdin in select.select([sys.stdin],
                                                        [], [], self.delay)[0]
            if terminal_ready:
                self.key = sys.stdin.read(1).lower()
            elif not terminal_ready:
                self.key = None

            # Manage modes
            if self.key == 'c' and self.Mode == 'normal':
                self.Mode = 'calibration'
                self.Event = None
                print('\nCALIBRATION STARTED')
                self.print_calibration_point_commands()
            elif self.key == 'n' and self.Mode == 'calibration':
                self.reset_mode(show_print=True)
                self.Event = None

            # Calibration mode
            if self.Mode == 'calibration':
                if self.Event is None and self.key == 'g':
                    self.Event = 'add_calibration_point'
                elif self.Event is None and self.key == 'r':
                    self.Event = 'reset_calibration'
                elif self.Event is None and self.key == 'y':
                    self.Event = 'restart_calibration'

                # Move Comand
                if self.key == 'w':
                    self.move_command = 'forward'
                elif self.key == 'a':
                    self.move_command = 'left'
                elif self.key == 's':
                    self.move_command = 'back'
                elif self.key == 'd':
                    self.move_command = 'right'
                elif self.key == 'e':
                    self.move_command = 'up'
                elif self.key == 'q':
                    self.move_command = 'down'

            # Normal mode
            elif self.Mode == 'normal':
                if self.Event is None and self.key == ' ':
                    self.toggle_touch_state()
                elif self.Event is None and self.key == 'r':
                    self.Event = 'reset_calibration'
                elif self.Event is None and self.key == 'v':
                    if not self.touch_state:
                        self.Event = 'tap'
                    elif self.touch_state:
                        self.toggle_touch_state()
                elif self.Event is None and self.key == 'b':
                    if not self.touch_state:
                        self.Event = 'double'
                    elif self.touch_state:
                        self.toggle_touch_state()

    '''--- Mode ---'''

    def get_mode(self):
        return self.Mode

    def set_mode(self, new_mode):
        if new_mode == 'normal' or new_mode == 0:
            self.Mode = new_mode
        elif new_mode == 'calibration' or new_mode == 1:
            self.Mode = new_mode
        else:
            self.Mode = 'normal'

    def reset_mode(self, show_print=False):
        self.Mode = 'normal'
        if show_print:
            print('\nCALIBRATION ABORTED')

    '''--- Touch State ---'''

    def get_touch_state(self):
        return self.touch_state

    def toggle_touch_state(self):
        self.touch_state = not self.touch_state

    '''--- Event ---'''

    def get_event(self):
        return self.Event

    def set_event(self, new_event):
        if new_event is None or new_event == 0:
            self.Event = None
        elif new_event == 'tap' or new_event == 1:
            self.Event = new_event
        elif new_event == 'double' or new_event == 1:
            self.Event = new_event
        else:
            self.Event = None

    def clear_event(self):
        self.Event = None

    '''--- Teleoperation Command ---'''

    def get_command(self):
        return self.move_command

    def clear_move_command(self):
        self.move_command = None

    '''--- Miscellaneous---'''

    def get_key(self):
        return self.key

    def set_delay(self, delay):
        self.delay = delay

    def list_possible_UserInterface(self):
        UserInterface_dict = collections.OrderedDict()

        Modes = collections.OrderedDict([('c', 'calibration'), ('n', 'normal')])
        Events = collections.OrderedDict([('v', 'tap'), ('b', 'double'), ('g', 'add_calibration_point'),
                                          ('r', 'reset_calibration'), ('y', 'restart_calibration')])
        Commands = collections.OrderedDict([('w', 'forward'), ('a', 'left'), ('s', 'back'),
                                           ('d', 'right'), ('e', 'up'), ('q', 'down')])
        Toggle = collections.OrderedDict([('spacebar', 'touch')])

        UserInterface_dict['Modes'] = Modes
        UserInterface_dict['Events'] = Events
        UserInterface_dict['Commands'] = Commands
        UserInterface_dict['Toggle'] = Toggle
        return UserInterface_dict

    def print_possible_UserInterface(self):
        UserInterface_dict = self.list_possible_UserInterface()
        print('\n\nUSER INTERFACE: ')
        print('\nMove green object on screen to move robot.')
        print('Note: Keyboard teleoperation (WASD) of robot is only usable during calibration.')
        for UI_struct_key, UI_struct_vals in UserInterface_dict.items():
            print('\n    %s: ' % (UI_struct_key))
            for key, val in UI_struct_vals.items():
                print('        %s = %s' % (key, val))

    def print_calibration_point_commands(self):
        if self.num_calibration == 4:
            print('\n---- Calibration Instructions ----')
            print('    1) Add TWO points on the bottom CORNERS of desired rectangle.')
            print('    2) Add ONE point anywhere on top LINE of desired rectangle.')
            print('    3) Add ONE point anywhere which TOUCHES the desired surface.')
        elif self.num_calibration == 3:
            print('\n---- Calibration Instructions ----')
            print('    1) Add TWO points on opposing CORNERS of desired rectangle.')
            print('    2) Add ONE point anywhere which TOUCHES the desired surface.')
        else:
            print('ERROR: Number of calibration points invalid.')

    def stop(self):
        self.stopped = True
        termios.tcsetattr(sys.stdin, termios.TCSANOW, self.default)


# Object : TrackSubscriber
# Description : Creates object which will subscribe to tracking node ROS topic
class TrackSubscriber:
    def __init__(self):
        self.track_data_default = [int(640/2), int(480/2), 0]
        self.track_data = self.track_data_default

    def callback_track_method(self, data):
        self.track_data = [data.x, data.y, 1]

    def subscribe(self):
        rospy.Subscriber('/tracking_coord', Point, self.callback_track_method, queue_size=2)
        return self.track_data

    def is_data_new(self, old_coord, new_coord):
        is_new_coord = True
        if new_coord == old_coord:
            is_new_coord = False
        return is_new_coord

    def return_default_coord(self):
        return self.track_data_default


# Object : Robot
# Description : Creates object which will manage robot operation and variables
class Robot:
    def __init__(self, move_group, initial_height,  delta, delta_event=0):
        # Initialize moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander(move_group)

        # Initialize robot physical parameters (millimeters)
        self.initial_height = initial_height
        self.delta = delta
        self.delta_event = delta_event
        self.touch_height = self.initial_height
        self.above_height = initial_height + self.delta
        self.touch_height_event = self.touch_height + self.delta_event
        self.current_pose = self.get_current_pose_method()
        self.current_pose_non_drift = self.current_pose
        self.x_end_offset = 0
        self.y_end_offset = 0

        # Initialize calibration
        self.calibration_list = None
        self.calibrate_now = False
        self.calibration_counter = 0
        self.num_calibration_points = 4

        # Initialize plan
        self.plan_type = 'cartesian'
        self.goal_pose = self.current_pose

        # Initialize maximum reachable space and maximum calibration
        # self.radius_min = 0.033
        self.radius_min = 0.025
        self.radius_max = 0.25
        self.angle_min = -math.pi/2
        self.angle_max = math.pi/2
        self.z_min = 0.125
        self.z_max = 0.277
        self.update_calibration_max_rectangular(
            self.radius_min, self.radius_max, self.angle_min, self.angle_max, self.z_min, self.z_max)
        max_calibration_data = self.get_calibration_max_rectangular()
        self.manual_calibration_update(max_calibration_data)

    '''--- Conversions and Simple Operations ---'''

    def eu_distance(self, pt1, pt2):
        pt1 = np.asarray(pt1)
        pt2 = np.asarray(pt2)
        diff_pt = pt1 - pt2
        dist = np.sqrt(np.dot(diff_pt.T, diff_pt))
        return dist

    def magnitude(self, vector):
        return np.sqrt(np.dot(vector.flatten(), vector.flatten()))

    def normalize_vector(self, vector):
        mag = self.magnitude(vector)
        return vector/mag

    def distance_point_line(self, line_pt1, line_pt2, point=[0, 0, 0]):
        if len(line_pt1) == 3:
            x1, y1, _ = line_pt1.flatten().tolist()
            x2, y2, _ = line_pt2.flatten().tolist()
            x0, y0, _ = point
        elif len(line_pt1) == 2:
            x1, y1 = line_pt1.flatten().tolist()
            x2, y2 = line_pt2.flatten().tolist()
            x0, y0 = point

        distance_num = abs((x2 - x1)*(y1 - y0) - (x1 - x0)*(y2 - y1))
        distance_denom = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        if distance_denom == 0.0:
            return 0
        distance = distance_num/distance_denom
        return distance

    def within_range(self, val, a, b):
        min_ = min(a, b)
        max_ = max(a, b)
        if min_ <= val <= max_:
            return True
        return False

    def quick_pose(self, x, y, z, w=1.0):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = w
        return pose

    def quick_cart(self, pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        return (x, y, z)

    def quick_list(self, numpy_array):
        return numpy_array.flatten().tolist()

    def quick_array(self, list, flat=False):
        if not flat:
            return np.array(list, dtype=float).reshape(len(list), 1)
        return np.array(list, dtype=float).reshape(len(list), 1).flatten()

    def pose_to_cylindrical_coords(self, pose):
        x, y, z = self.quick_cart(pose)
        radius = math.sqrt(x**2 + y**2)
        angle = math.atan2(y, x)
        return (radius, angle, z)

    def cylindrical_coords_to_pose(self, radius, angle, z):
        x = radius*math.cos(angle)
        y = radius*math.sin(angle)
        return self.quick_pose(x, y, z)

    def millimeters_to_robot_dim(self, milli):
        robot_dist = .28
        real_dist = .335
        ratio = (0.001)*(robot_dist/real_dist)
        return ratio*milli

    '''--- Current Coordinates and Pose ---'''

    def get_current_pose_method(self):
        return self.arm.get_current_pose().pose

    def get_current_coord_method(self):
        return self.quick_cart(self.arm.get_current_pose().pose)

    '''--- Endeffector ---'''

    def set_endeffector_offsets(self, x_end_offset, y_end_offset):
        self.x_end_offset = self.millimeters_to_robot_dim(x_end_offset)
        self.y_end_offset = self.millimeters_to_robot_dim(y_end_offset)

    def pose_convert_arm_to_endeffector(self, arm):
        x_a, y_a, z = self.quick_cart(arm)
        v_arm = self.quick_array([x_a, y_a])
        r, ang, z = self.pose_to_cylindrical_coords(arm)

        ct, st = (math.cos(ang),  math.sin(ang))
        rot_matrix = np.array([[ct, -st], [st, ct]])

        v_offsets_end = self.quick_array([self.x_end_offset, self.y_end_offset])
        v_end = v_arm + np.matmul(rot_matrix, v_offsets_end)
        x_e, y_e = self.quick_list(v_end)

        endeffector = self.quick_pose(x_e, y_e, z)
        return endeffector

    def pose_convert_endeffector_to_arm(self, endeffector):
        r_end, ang_end, z = self.pose_to_cylindrical_coords(endeffector)
        r_off = math.sqrt(self.x_end_offset**2 + self.y_end_offset**2)
        ang_off = math.atan2(self.y_end_offset, self.x_end_offset)

        theta_1 = math.pi - abs(ang_off)
        theta_2 = math.asin((r_off/r_end)*math.sin(theta_1))
        delta_theta = np.sign(ang_off)*theta_2
        ang_arm = ang_end - delta_theta

        x_e, y_e, z = self.quick_cart(endeffector)
        v_end = self.quick_array([x_e, y_e])

        ct, st = (math.cos(ang_arm),  math.sin(ang_arm))
        rot_matrix = np.array([[ct, -st], [st, ct]])

        v_offsets_end = self.quick_array([self.x_end_offset, self.y_end_offset])
        v_arm = v_end - np.matmul(rot_matrix, v_offsets_end)
        x_a, y_a = self.quick_list(v_arm)

        arm = self.quick_pose(x_a, y_a, z)
        return arm

    '''--- Reachable Workspace ---'''

    def check_pose_reachable(self, pose_to_check):
        radius, angle, z = self.pose_to_cylindrical_coords(pose_to_check)

        radius_in_range = self.within_range(radius, self.radius_min, self.radius_max)
        angle_in_range = self.within_range(angle, self.angle_min, self.angle_max)
        z_in_range = self.within_range(z, self.z_min, self.z_max)

        in_range = True if radius_in_range and angle_in_range and z_in_range else False
        return (in_range, radius_in_range, angle_in_range, z_in_range)

    def get_closest_reachable_pose(self, unreachable_pose):
        in_rang, rad_in_rang, ang_in_rang, z_in_rang = self.check_pose_reachable(
            unreachable_pose)

        if in_rang:
            return unreachable_pose

        print('ERROR: Pose out or range. Next closest point found.')

        buff_r, buff_ang, buff_z = (0.01, math.pi/180, 0.01)

        radius_min_buff = self.radius_min + buff_r
        radius_max_buff = self.radius_max - buff_r
        angle_min_buff = self.angle_min + buff_ang
        angle_max_buff = self.angle_max - buff_ang
        z_min_buff = self.z_min + buff_z
        z_max_buff = self.z_max - buff_z

        radius, angle, z = self.pose_to_cylindrical_coords(unreachable_pose)

        if not rad_in_rang:
            radius = np.clip(radius, radius_min_buff, radius_max_buff)

        if not ang_in_rang:
            angle = np.clip(angle, angle_min_buff, angle_max_buff)
            print('test')

        if not z_in_rang:
            z = np.clip(z, z_min_buff, z_max_buff)

        print(in_rang, rad_in_rang, ang_in_rang, z_in_rang)
        print(radius, angle, z)

        return self.cylindrical_coords_to_pose(radius, angle, z)

    def check_corners_inner_circle(self, pt1, pt2, pt3, pt4):
        mag_1 = self.distance_point_line(pt1, pt2)
        mag_2 = self.distance_point_line(pt2, pt3)
        mag_3 = self.distance_point_line(pt3, pt4)
        mag_4 = self.distance_point_line(pt4, pt1)
        if mag_1 and mag_2 and mag_3 and mag_4 > self.radius_min:
            return True
        return False

    def set_reachable_space(self, radius_min, radius_max, angle_min, angle_max, z_min, z_max):
        self.radius_min = radius_min
        self.radius_max = radius_max
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.z_min = z_min
        self.z_max = z_max

    '''--- Tracking Data Conversion and Update ---'''

    def image_to_physical_coord(self, x_vid, y_vid, range='current_calibration'):
        if range == 'current_calibration':
            width, length = (self.width, self.length)
            x_offset, y_offset = (self.x_offset, self.y_offset)
            angle = self.angle
        elif range == 'max_rectangle':
            width, length = (self.width_max, self.length_max)
            x_offset, y_offset = (self.x_offset_max, self.y_offset_max)
            angle = self.angle_max_tilt

        # Tracking resolution and coord direction compensation
        x_res, y_res = (640, 480)
        self.x_sign, self.y_sign = (-1, -1)

        # Physical coords about origin
        x_prev = self.x_sign*(y_vid*(width/y_res) - width/2)
        y_prev = self.y_sign*(x_vid*(length/x_res) - length/2)
        xy_prev = self.quick_array([x_prev, y_prev])

        # Rotation matrix and offset arrays
        ct = math.cos(angle)
        st = math.sin(angle)
        rotation_matrix = np.array([[ct, -st], [st, ct]])
        offsets = self.quick_array([x_offset, y_offset])

        # Physical coords with offset
        xy_array = (np.matmul(rotation_matrix, xy_prev) + offsets)
        xy_list = self.quick_list(xy_array)
        return xy_list

    def update_track_poses(self, track_coords, range='current_calibration'):
        x, y, _ = track_coords
        goal_x, goal_y = self.image_to_physical_coord(x, y, range)

        self.goal_touch = self.quick_pose(goal_x, goal_y, self.touch_height)
        self.goal_above = self.quick_pose(goal_x, goal_y, self.above_height)
        self.goal_above_event = self.quick_pose(goal_x, goal_y,
                                                self.touch_height_event)

    def get_track_poses(self):
        return self.goal_touch, self.goal_above, self.goal_above_event

    def get_pose_based_touch(self, touch_state):
        if touch_state:
            return self.goal_touch
        return self.goal_above

    '''--- Calibration ---'''

    def add_calibration_coord_current(self, endeffector=True):
        if self.calibration_list is None:
            self.calibration_list = []

        curr_arm = self.get_current_pose_method()
        if endeffector:
            x_e, y_e, z_e = self.quick_cart(self.pose_convert_arm_to_endeffector(curr_arm))
            self.calibration_list.append([x_e, y_e, z_e])
        elif not endeffector:
            x_a, y_a, z_a = self.quick_cart(curr_arm)
            self.calibration_list.append([x_a, y_a, z_a])

    def add_calibration_coord_manual(self, x, y, z):
        if self.calibration_list is None:
            self.calibration_list = []
        self.calibration_list.append([x, y, z])

    def clear_calibration_list(self):
        self.calibration_list = None
        self.calibrate_now = False

    def get_calibration_counter(self):
        if self.calibration_list is None:
            self.calibration_counter = 0
        elif self.calibration_list is not None:
            self.calibration_counter = len(self.calibration_list)
        return self.calibration_counter, self.num_calibration_points

    def is_calibration_complete(self):
        _ = self.get_calibration_counter()
        if self.calibration_counter >= self.num_calibration_points:
            self.calibrate_now = True
            return self.calibrate_now
        self.calibrate_now = False
        return self.calibrate_now

    def update_calibration_max_rectangular(self, radius_min, radius_max, angle_min, angle_max, z_min, z_max):
        self.set_reachable_space(radius_min, radius_max, angle_min, angle_max, z_min, z_max)

        k = (radius_min + math.sqrt(radius_min**2 + 8*radius_max**2))/(4*radius_max)

        angle_intersec_max = abs(math.acos(radius_min/radius_max))
        angle_intersection = np.clip(math.acos(k), -angle_intersec_max, angle_intersec_max)

        eps = 0.01
        x_max = abs((radius_max - eps)*math.cos(angle_intersection))
        y_max = abs((radius_max - eps)*math.sin(angle_intersection))
        self.width_max = abs(x_max - radius_min)
        self.length_max = abs(2*y_max)
        self.x_offset_max = self.width_max/2 + radius_min
        self.y_offset_max = 0
        self.angle_max_tilt = 0

    def get_calibration_max_rectangular(self):
        return [self.width_max, self.length_max, self.x_offset_max,
                self.y_offset_max, self.angle_max_tilt, self.touch_height]

    def set_touch_heights(self, new_touch_height):
        self.touch_height = new_touch_height
        self.above_height = self.touch_height + self.delta
        self.touch_height_event = self.touch_height + self.delta_event

    def get_calibration_data(self):
        return [self.width, self.length, self.x_offset, self.y_offset,
                self.angle, self.touch_height]

    def automatic_three_point_calibration(self):
        point1, point2, point3 = self.calibration_list
        x1, y1, _ = point1
        x2, y2, _ = point2
        _, _, z3 = point3
        z_reach = True if self.z_min < z3 < self.z_max else False

        if z_reach:
            x_max, x_min = (max([x1, x2]), min([x1, x2]))
            y_max, y_min = (max([y1, y2]), min([y1, y2]))

            # Check calibration is valid
            safe_from_circle = True if x_min > self.radius_min else False
            pt1_reach = self.check_pose_reachable(self.quick_pose(x_min, y_min, z3))
            pt2_reach = self.check_pose_reachable(self.quick_pose(x_min, y_max, z3))
            pt3_reach = self.check_pose_reachable(self.quick_pose(x_max, y_max, z3))
            pt4_reach = self.check_pose_reachable(self.quick_pose(x_max, y_min, z3))

            if safe_from_circle and pt1_reach and pt2_reach and pt3_reach and pt4_reach:
                self.width = abs(x_max - x_min)
                self.length = abs(y_max - y_min)
                self.x_offset = x_min + self.width/2
                self.y_offset = y_min + self.length/2
                self.angle = 0
                self.set_touch_heights(z3)
            else:
                print('    ERROR: Bounding box did not fall within reachable area.')
        else:
            print('    ERROR: Touch height was out of bounds.')

        self.clear_calibration_list()

    def automatic_four_point_calibration(self):
        new_z = self.calibration_list[3][2]
        z_reach = True if self.z_min < new_z < self.z_max else False

        if z_reach:
            a, b, c, _ = [np.array([x[0], x[1], self.touch_height], dtype=float)
                          for x in self.calibration_list]

            # Directional vectors
            c_direction_vec = self.normalize_vector(np.cross((a-b), np.cross((c-b), (a-b))))
            ab_direction_vec = self.normalize_vector(np.cross(c_direction_vec, np.array([0, 0, 1])))

            width = abs(np.dot((c-a), c_direction_vec))
            length = abs(self.magnitude(a-b))

            # Rectangle side vectors
            c_vec = width*c_direction_vec
            ab_vec = length*ab_direction_vec

            # Corner points
            midpt_ab = 0.5*(a+b)
            pt1 = midpt_ab + 0.5*ab_vec
            pt2 = pt1 + c_vec
            pt3 = pt2 - ab_vec
            pt4 = pt3 - c_vec

            x1, y1, z1 = pt1
            x2, y2, z2 = pt2
            x3, y3, z3 = pt3
            x4, y4, z4 = pt4

            # check calibration is valid
            safe_from_circle = self.check_corners_inner_circle(pt1, pt2, pt3, pt4)
            pt1_reach = self.check_pose_reachable(self.quick_pose(x1, y1, z1))
            pt2_reach = self.check_pose_reachable(self.quick_pose(x2, y2, z2))
            pt3_reach = self.check_pose_reachable(self.quick_pose(x3, y3, z3))
            pt4_reach = self.check_pose_reachable(self.quick_pose(x4, y4, z4))

            if safe_from_circle and pt1_reach and pt2_reach and pt3_reach and pt4_reach:
                angle_rotation = math.atan2(c_vec[1], c_vec[0])

                self.width = width
                self.length = length
                self.x_offset = (x1 + x3)/2
                self.y_offset = (y1 + y3)/2
                self.angle = angle_rotation
                self.set_touch_heights(new_z)
            else:
                print('    ERROR: Bounding box did not fall within reachable area.')
        else:
            print('    ERROR: Touch height was out of bounds.')

        self.clear_calibration_list()

    def manual_calibration_update(self, calibration_data):
        # check calibration is valid
        self.width = calibration_data[0]
        self.length = calibration_data[1]
        self.x_offset = calibration_data[2]
        self.y_offset = calibration_data[3]
        self.angle = calibration_data[4]
        self.set_touch_heights(calibration_data[5])

        self.clear_calibration_list()

    '''--- Teleoperation ---'''

    def get_teleop_goal_pose(self, command, end_effector=True):
        if end_effector:
            cur_arm_pose = self.current_pose_non_drift
            final_pose = self.pose_convert_arm_to_endeffector(cur_arm_pose)
        elif not end_effector:
            final_pose = self.current_pose_non_drift

        x_f, y_f, z = self.quick_cart(final_pose)

        x_dir_sign, x_adjust = (+1, 0.01)
        y_dir_sign, y_adjust = (+1, 0.01)
        z_dir_sign, z_adjust = (+1, 0.007)

        if command == 'forward':
            x_f += x_dir_sign*x_adjust
        elif command == 'back':
            x_f -= x_dir_sign*x_adjust
        elif command == 'left':
            y_f += y_dir_sign*y_adjust
        elif command == 'right':
            y_f -= y_dir_sign*y_adjust
        elif command == 'up':
            z += z_dir_sign*z_adjust
        elif command == 'down':
            z -= z_dir_sign*z_adjust

        arm_pose = self.quick_pose(x_f, y_f, z)
        return arm_pose

    '''--- Goal and Waypoint Planning ---'''

    def set_goal_pose(self, goal_pose, end_effector=True):
        if end_effector:
            arm_goal = self.pose_convert_endeffector_to_arm(goal_pose)
            self.goal_pose = self.get_closest_reachable_pose(arm_goal)
        if not end_effector:
            self.goal_pose = self.get_closest_reachable_pose(goal_pose)

    def create_waypoints(self):
        self.clear_waypoints()
        self.waypoints.append(self.get_current_pose_method())

    def append_waypoints(self, new_waypoint, end_effector=True):
        if end_effector:
            arm_waypoint = self.pose_convert_endeffector_to_arm(new_waypoint)
            self.waypoints.append(self.get_closest_reachable_pose(arm_waypoint))
        elif not end_effector:
            self.waypoints.append(self.get_closest_reachable_pose(new_waypoint))

    def clear_waypoints(self):
        self.waypoints = []

    def set_plan_type(self, new_plan_type):  # Only accepts 'cartesian' or 'pose_goal'
        self.plan_type = new_plan_type

    '''--- Moveit Execution ---'''

    def plan_execute(self, wait_cart=True, wait_pose_goal=True):
        if self.plan_type == 'cartesian':
            if len(self.waypoints) >= 2:
                self.current_pose_non_drift = self.waypoints[-1]
            elif len(self.waypoints) < 2:
                self.append_waypoints(self.current_pose_non_drift, end_effector=False)

            (plan, frac) = self.arm.compute_cartesian_path(self.waypoints,
                                                           0.005, 0.0)
            self.arm.execute(plan, wait_cart)
            self.clear_waypoints()
        elif self.plan_type == 'pose_goal':
            pass
