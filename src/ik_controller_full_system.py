#!/usr/bin/env python
"""
Chainatee Tanakulrungson
Apr 2017

This file take input of desired position and Sawyer's arm to that point instantly

SUBSCRIPTIONS:
    - ref_pose (PoseStamped)

PUBLISHERS:
    - joint_states (JointState)

SERVICES:
    - toggle_controller (SetBool)
    - reset (Empty)
    - random (Empty)

PARAMS:
    - ~base_frame (frame to express target in)
    - ~target_frame (target frame)
    - ~arm (which arm to use)
    - ~freq (controller frequency)
"""

###############
# ROS IMPORTS #
###############
import rospy
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import tf.transformations as tr
import tf
import intera_interface 

##################
# PYTHON IMPORTS #
##################
import numpy as np
import threading

#################
# LOCAL IMPORTS #
#################
import sawyer_MR_description as smr
import modern_robotics as mr
import custom_logging as cl

# global params
FREQ = 100
DAMPING = 0.02
ARM = "right"
STEP_SCALE = 1.0
JOINT_VEL_LIMIT = 1.5

# once the ball is thrown
# 1.) running flag = True
# 2.) input self.goal

class IKController( object ):
    def __init__(self):
        rospy.loginfo("Creating IK Controller class")
        
        # setup flags and useful vars
        self.running_flag = False
        self.step_scale = rospy.get_param("~scale", STEP_SCALE)
        self.freq = rospy.get_param("~freq", FREQ)
        self.arm = rospy.get_param("~arm", ARM)
        self.q = np.zeros(7)
        with cl.suppress_stdout_stderr():
            self.urdf = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.urdf, "base", "%s_hand"%self.arm)
        self.goal = np.array(self.kin.forward(self.q))
        self.mutex = threading.Lock()
        self.joint_vel_limit = rospy.get_param("~joint_vel_limit", JOINT_VEL_LIMIT)
        self.q_sim = np.zeros(7)
        self.damping = rospy.get_param("~damping", DAMPING)
        self.joint_names = self.kin.get_joint_names() 
        self.qdot_dict = {}
        self.limb_interface = intera_interface.Limb()


        # create all subscribers, publishers, and timers
        self.int_timer = rospy.Timer(rospy.Duration(1/float(self.freq)), self.ik_step_timercb)
        self.actual_js_sub = rospy.Subscriber("robot/joint_states", JointState, self.actual_js_cb)

    def set_goal_from_pose(self, pose):
    # set goal from input posestamped
        with self.mutex:
            p = np.array([pose.pose.position.x, pose.pose.position.y,
                          pose.pose.position.z])
            q = np.array([pose.pose.orientation.x, pose.pose.orientation.y,
                          pose.pose.orientation.z, pose.pose.orientation.w])
            self.goal = tr.euler_matrix(*tr.euler_from_quaternion(q))
            self.goal[0:3,-1] = p
        return


    def actual_js_cb(self, js):
    # update present robot/joint_states to self.q
        qtmp = np.zeros(7)
        names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        i = 0
        for j,n in enumerate(js.name):
            if n in names:
                qtmp[i] = js.position[j]
                i += 1
        with self.mutex:
            self.q = qtmp
        return


    def step_ik(self):
        # grab current joint angles and SE(3) target:
        q = self.q
        with self.mutex:
            g_sd = self.goal
        # Calculate transform from current EE pose to desired EE pose
        err = np.dot(mr.TransInv(mr.FKinBody(smr.M, smr.Blist, q)), g_sd)
        # now convert this to a desired twist
        Vb = mr.se3ToVec(mr.MatrixLog6(err))
        # calculate body Jacobian at current config
        J_b = mr.JacobianBody(smr.Blist, q)
        # now calculate an appropriate joint angle velocity:
        # qdot = np.dot(np.linalg.pinv(J_b), Vb)
        idim = J_b.shape[-1]
        qdot = np.dot(np.dot(np.linalg.inv(np.dot(J_b.T,J_b) + self.damping*np.eye(idim)), J_b.T), Vb)
        # increase velocity in each joint
        multiplier = 4
        qdot = qdot*multiplier
        # clip down velocity in each joint
        if np.amax(qdot) > self.joint_vel_limit:
            qdot = qdot/np.amax(qdot)*self.joint_vel_limit
        self.qdot_dict = dict(zip(self.joint_names, qdot))
        # names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        # print "max vel joint : ", names[qdot.index(np.amax(qdot))], " : ", qdot[qdot.index(np.amax(qdot))]
        # print "max vel joint : ", names[np.where(qdot == np.amax(qdot))[0]], " : ", qdot[np.where(qdot == np.amax(qdot))[0]]

        if np.amax(qdot) > self.joint_vel_limit:
            print "joint limit reach !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    
        # self.q_sim += self.step_scale*qdot
        # self.qdot = qdot
        # self.joint_cmd.velocity = qdot 
        # return


    def ik_step_timercb(self, tdat):
    # perform step inverse kinematics if running_flag is true
        if self.running_flag:
            self.step_ik()
            self.limb_interface.set_joint_velocities(self.qdot_dict)
            print self.qdot_dict
        #     # publish joint command timeout
        #     self.joint_cmd_timeout_pub.publish(JOINT_CMD_FREQ)
        #     # publish joint command message
        #     self.joint_cmd.header.stamp = tdat.current_expected
        #     self.joint_cmd_pub.publish(self.joint_cmd)
        #     # print self.joint_cmd
        # self.js.header.stamp = tdat.current_expected
        # self.js.position = self.qsim
        # self.js_pub.publish(self.js)
        # # publish pose
        # self.fwd_kin()
        # self.pose.header.stamp = tdat.current_expected
        # self.pose.header.frame_id = BASE
        # self.pose_pub.publish(self.pose)
        # return
