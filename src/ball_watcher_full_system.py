#!/usr/bin/env python
"""
Chainatee Tanakulrungson
Mar 2017

This file subscribes to the ball frame, and estimates the position of the ball on a plane 

SUBSCRIPTIONS:
    - /base to /ball (LookUpTransform)
    - 

PUBLISHERS:

SERVICES:

PARAMS:

"""

###############
# ROS IMPORTS #
###############
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped, PoseStamped
import tf
import tf.transformations as tr
from pykdl_utils.kdl_kinematics import KDLKinematics
import intera_interface 

##################
# PYTHON IMPORTS #
##################
import numpy as np
import threading
from urdf_parser_py.urdf import URDF
# import matplotlib.pyplot as plt

#################
# LOCAL IMPORTS #
#################
import kbhit
import sawyer_catch_ball_calc as sawyer_calc
import sawyer_catch_ball_markers as sawyer_mk
from ik_controller_full_system import IKController

# GLOBAL VARS
BASE = "base"
EE_FRAME = "right_hand"
# TARGET_FRAME = "ball"
# SOURCE_FRAME = "base"
TARGET_FRAME = "base"
SOURCE_FRAME = "ball"
# X_KINECT_CALIBR = 0.
# Y_KINECT_CALIBR = -0.7
# Z_KINECT_CALIBR = 0.6
# X_KINECT_CALIBR = -0.60
# Y_KINECT_CALIBR = -0.60
# Z_KINECT_CALIBR = 0.628
X_KINECT_CALIBR = -0.31394
X_KINECT_CALIBR_2 = X_KINECT_CALIBR + 0.15
Y_KINECT_CALIBR = -0.35392
Z_KINECT_CALIBR = 0.65747489
# ROLL_KINECT_CALIBR = -0.436332 # -25 deg
# PITCH_KINECT_CALIBR = 1.5708 
# YAW_KINECT_CALIBR = 0
# ROLL_KINECT_CALIBR = 0.349066 #10 deg
ROLL_KINECT_CALIBR = 0
PITCH_KINECT_CALIBR = -1.5708 
YAW_KINECT_CALIBR = 0
Z_CENTER = 0.317

class BallWatcher(object):
    def __init__(self):
        rospy.loginfo("Creating BallWatcher class")
        self.print_help()

        # flags and vars
        # for ball dropping position
        self.kinect_calibrate_flag = False
        self.running_flag = False
        self.start_calc_flag = False
        self.ball_marker = sawyer_mk.MarkerDrawer("/base", "ball", 500)
        self.drop_point = Point()
        self.drop_point_arr = []
        self.drop_point_marker = sawyer_mk.MarkerDrawer("/base", "dropping_ball", 500)
        self.ik_cont = IKController()

        self.robot = URDF.from_parameter_server()  
        self.kin = KDLKinematics(self.robot, BASE, EE_FRAME)
        self.limb_interface = intera_interface.Limb()
        self.pos_rec = [PointStamped() for i in range(2)] # array 1x2 with each cell storing a PointStamped with cartesian position of the ball in the past two frames
        self.old_pos_rec = [PointStamped() for i in range(2)]
        self.last_tf_time = rospy.Time()
        self.tf_listener = tf.TransformListener()
        self.tf_bc = tf.TransformBroadcaster()

        # kbhit instance
        self.kb = kbhit.KBHit()
        rospy.on_shutdown(self.kb.set_normal_term)

        # publishers and timers
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)
        self.tf_timer = rospy.Timer(rospy.Duration(0.01), self.tf_update_cb)
        self.final_x_total = 0
        self.final_y_total = 0


    def check_start_throw(self):
        # check if the ball is being thrown, by comparing between z of the first and second frame
        z_past = self.pos_rec[0].point.z
        z_present = self.pos_rec[1].point.z
        if (z_present - z_past > 0.05):
            self.start_calc_flag = True

    def check_stop_throw(self):
        # check if the ball reach the end of trajectory, if so, raise down the start_calc_flag
        x_past = self.pos_rec[0].point.x
        x_present = self.pos_rec[1].point.x
        if (x_present - x_past > 0):
            self.start_calc_flag = False

    def roll_mat(self, mat):
        row_num = len(mat)
        for i in range(0,row_num - 1):
            mat[i] = mat[i+1]
        return


    def tf_update_cb(self, tdat):
        # update ball position in real time
        # Filter ball outside x = 0 - 3.0m relative to base out
        try:
            self.tf_listener.waitForTransform(TARGET_FRAME, SOURCE_FRAME, rospy.Time(), rospy.Duration(0.5))
        except tf.Exception:
            rospy.loginfo("No frame of /ball from /base received, stop calculation. self.start_calc_flag = False")
        p, q = self.tf_listener.lookupTransform(TARGET_FRAME, SOURCE_FRAME, rospy.Time())
        pos = PointStamped()
        # pos.header.stamp = rospy.get_time()
        pos.header.stamp = rospy.Time.now()
        pos.point.x  = p[0]
        pos.point.y  = p[1]
        pos.point.z  = p[2]
        # filter repeated received tf out
        if (self.pos_rec[-1].header.stamp != pos.header.stamp) and (self.pos_rec[-1].point.x != pos.point.x):
            self.roll_mat(self.pos_rec)
            self.pos_rec[-1] = pos
        # choose only a non-repeated pos_rec by comparing between the timestamped of the present and past pos_rec
        if (self.last_tf_time != self.pos_rec[0].header.stamp):
            # If running_flag is True, start detecting
            if self.running_flag:
                # check if the ball is being thrown yet
                if not self.start_calc_flag:
                    self.check_start_throw()
                    self.ball_marker.draw_spheres([0, 0.7, 0, 1], [0.03, 0.03,0.03], self.pos_rec[0].point)
                    self.ball_marker.draw_line_strips([5.7, 1, 4.7, 1], [0.01, 0,0], self.pos_rec[0].point, self.pos_rec[1].point)
                if self.start_calc_flag:
                    self.check_stop_throw()
                    self.ik_cont.running_flag = True
                    if not self.start_calc_flag:
                        return
                    # draw markers
                    self.ball_marker.draw_spheres([0.7, 0, 0, 1], [0.03, 0.03,0.03], self.pos_rec[0].point)
                    self.ball_marker.draw_line_strips([1, 0.27, 0.27,1], [0.01, 0,0], self.pos_rec[0].point, self.pos_rec[1].point)
                    self.ball_marker.draw_numtxts([1, 1, 1, 1], 0.03, self.pos_rec[0].point, 0.03)
                    # calculate the dropping position based on 2 points
                    self.drop_point = sawyer_calc.projectile_calc(self.pos_rec[0], self.pos_rec[1], Z_CENTER)
                    # self.drop_point = sawyer_calc.projectile_calc(self.pos_rec[0], self.pos_rec[1], 0.00)
                    self.drop_point_arr.append(self.drop_point)
                    self.drop_point_arr = sawyer_calc.point_msg_reject_outliers_xAxis(self.drop_point_arr, 1.5)
                    self.drop_point = sawyer_calc.point_msg_avg(self.drop_point_arr)
                    # average drop point
                    input_posestamped = PoseStamped()
                    input_posestamped.pose.position = self.drop_point
                    input_posestamped.pose.orientation = Quaternion(0.0392407571798, 0.664506667783, -0.0505321422468, 0.744538483926)
                    self.ik_cont.set_goal_from_pose(input_posestamped)
                    self.drop_point_marker.draw_spheres([0, 0, 0.7, 1], [0.03, 0.03,0.03], self.drop_point)
                    self.drop_point_marker.draw_numtxts([1, 1, 1, 1], 0.03, self.drop_point, 0.03)
                    # self.drop_point_marker.draw_line_strips([0, 1, 1,1], [0.01, 0,0], self.pos_rec[0].point, self.pos_rec[1].point)
            self.last_tf_time = self.pos_rec[0].header.stamp


    def keycb(self, tdat):
        # check if there was a key pressed, and if so, check it's value and
        # toggle flag:
        if self.kb.kbhit():
            c = self.kb.getch()
            if ord(c) == 27:
                rospy.signal_shutdown("Escape pressed!")
            else:
                print c
            if c == 's':
                rospy.loginfo("You pressed 's', Program starts. Sawyer is waiting for the ball to be thrown.")
                self.running_flag = not self.running_flag
                if not self.running_flag:
                    self.ball_marker.delete_all_mks()
            elif c == 'c':
                rospy.loginfo("You pressed 'c', Start calibration\nrunning_flag = False")
                self.running_flag = False
                self.kinect_pos_calib()
            elif c == 'h':
                rospy.loginfo("You pressed 'h', Go to home position\nrunning_flag = False")
                self.running_flag = False
                self.home_pos()
            elif c == 'p':
                rospy.loginfo("You pressed 'h', Plotting")
                self.plot_flag = True
            else:
                self.print_help()
            self.kb.flush()
        return

    def kinect_pos_calib(self):
        # 
        if not self.kinect_calibrate_flag:
            # input x,y,z and x,y,z
            p = [X_KINECT_CALIBR, Y_KINECT_CALIBR, Z_KINECT_CALIBR]
            self.kinect_calibrate_flag = True
            rospy.loginfo("Press C again for calibrate the height of the camera")
        else:
            p = [X_KINECT_CALIBR_2, Y_KINECT_CALIBR, Z_KINECT_CALIBR]            
            self.kinect_calibrate_flag = False
        G = tr.euler_matrix(ROLL_KINECT_CALIBR, PITCH_KINECT_CALIBR, YAW_KINECT_CALIBR)
        for i in range(3):
            G[i][3] = p[i]
        q_seed = self.kin.random_joint_angles()
        j_names = self.kin.get_joint_names()
        q = self.kin.inverse(G, q_seed)
        while (q is None):
            q_seed = self.kin.random_joint_angles()
            q = self.kin.inverse(G, q_seed)
        j_req = dict(zip(j_names, q))
        self.limb_interface.move_to_joint_positions(j_req)

    def home_pos(self):
        self.limb_interface.move_to_joint_positions({'head_pan':0.379125, 'right_j0':-0.020025390625 , 'right_j1':0.8227529296875, 'right_j2':-2.0955126953125, 'right_j3':2.172509765625, 'right_j4':0.7021171875, 'right_j5' : -1.5003603515625, 'right_j6' : -2.204990234375})

    def print_help(self):
        help_string = \
        """
        's'   ~  Start the program
        'c'   ~  Calibration the kinect position
        'h'   ~  Move to home position
        'p'   ~  Plot the trajectory of each Cartesian coordinate
        'ESC' ~  Quit
        """
        print help_string
        return



def main():
    rospy.init_node('ball_catcher_system', log_level=rospy.INFO)

    try:
        ballwatcher = BallWatcher()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()





