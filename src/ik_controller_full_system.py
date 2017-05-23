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
from intera_core_msgs.msg import EndpointState

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
JOINT_VEL_LIMIT = 1.328

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
        self.ep = EndpointState()

        # create all subscribers, publishers, and timers
        self.int_timer = rospy.Timer(rospy.Duration(1/float(self.freq)), self.ik_step_timercb)
        self.actual_js_sub = rospy.Subscriber("robot/joint_states", JointState, self.actual_js_cb)
        self.endpoint_update_cb = rospy.Subscriber("robot/limb/right/endpoint_state", EndpointState, self.endpoint_upd_cb)

    def endpoint_upd_cb(self, ep_in):
        self.ep = ep_in

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
        ########Clip down algorithm
        # joint vel lim dict
        vel_lim = {'right_j0': 1.74, 'right_j1': 1.328, 'right_j2': 1.957, 'right_j3': 1.957, 'right_j4': 3.485, 'right_j5': 3.485, 'right_j6': 4.545}
        qdot_unclip = dict(zip(self.joint_names, qdot))
        qd_over_vel = [qd for qd in qdot_unclip if qdot_unclip[qd] > vel_lim[qd]]
        if qd_over_vel != []:
            print "joint limit reach in: ", qd_over_vel
            qd_vel_lim_min = min(qd_over_vel, key=qd_over_vel.get)
            f = 0.7
            qdot = qdot/qdot_unclip[qd_vel_lim_min]*vel_lim[qd_vel_lim_min]*f
            print "reduce to: ", qdot
            print "with limit: ", vel_lim
        self.qdot_dict = dict(zip(self.joint_names, qdot))

        # clip down velocity in each joint
        # if np.amax(qdot) > self.joint_vel_limit:
        #     print "joint limit reach !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        #     qdot = qdot/np.amax(qdot)*self.joint_vel_limit
        # self.qdot_dict = dict(zip(self.joint_names, qdot))

    def ik_step_timercb(self, tdat):
    # perform step inverse kinematics if running_flag is true
        if self.running_flag:
            self.step_ik()
            self.limb_interface.set_joint_velocities(self.qdot_dict)
            # print self.qdot_dict
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
    
    #######################################################################################


    def gen_joints_list(self, input_pose):
        self.limb_interface.move_to_joint_positions({'head_pan':0.379125, 'right_j0':-0.020025390625 , 'right_j1':0.8227529296875, 'right_j2':-2.0955126953125, 'right_j3':2.172509765625, 'right_j4':0.7021171875, 'right_j5' : -1.5003603515625, 'right_j6' : -2.204990234375}) 

        # self.running_flag = True
        tol = 0.001 #1 mm
        ret_list = []
        while (abs(self.ep.pose.position.x - input_pose.position.x) > tol) and (abs(self.ep.pose.position.y - input_pose.position.y) > tol):
            # set_goal_from_pose part
            p = np.array([input_pose.position.x, input_pose.position.y,
                          input_pose.position.z])
            q = np.array([input_pose.orientation.x, input_pose.orientation.y,
                          input_pose.orientation.z, input_pose.orientation.w])
            goal = tr.euler_matrix(*tr.euler_from_quaternion(q))
            goal[0:3,-1] = p   
            # actual_js_cb: get the actual joint state
            q = self.q
            with self.mutex:
                g_sd = goal        

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
            ########Clip down algorithm
            # joint vel lim dict
            vel_lim = {'right_j0': 1.74, 'right_j1': 1.328, 'right_j2': 1.957, 'right_j3': 1.957, 'right_j4': 3.485, 'right_j5': 3.485, 'right_j6': 4.545}
            qdot_unclip = dict(zip(self.joint_names, qdot))
            qd_over_vel = [[qd, qdot_unclip[qd]] for qd in qdot_unclip if qdot_unclip[qd] > vel_lim[qd]]
            if qd_over_vel != []:
                print "joint limit reach in: ", qd_over_vel
                # qd_over_vel = dict(zip(qd_over_vel, qdot_unclip[qd]))
                # qd_vel_lim_min = min(qd_over_vel, key=qd_over_vel.get)
                qd_vel_lim_val = [qd_over_vel[i][1] for i in range(len(qd_over_vel))]
                qd_vel_lim_min = min(qd_vel_lim_val)
                f = 0.7
                # qdot = qdot/qdot_unclip[qd_vel_lim_min]*vel_lim[qd_vel_lim_min]*f
                qdot = qdot/qd_vel_lim_min*qd_vel_lim_min*f
                print "reduce to: ", qdot
                print "with limit: ", vel_lim
            qdot_dict = dict(zip(self.joint_names, qdot))
            self.limb_interface.set_joint_velocities(qdot_dict)
            print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1\n", qdot_dict
            ret_list.append(qdot_dict)
            

        self.limb_interface.move_to_joint_positions({'head_pan':0.379125, 'right_j0':-0.020025390625 , 'right_j1':0.8227529296875, 'right_j2':-2.0955126953125, 'right_j3':2.172509765625, 'right_j4':0.7021171875, 'right_j5' : -1.5003603515625, 'right_j6' : -2.204990234375}) 
        
        return ret_list

    def home(self):
        self.limb_interface.move_to_joint_positions({'head_pan':0.379125, 'right_j0':-0.020025390625 , 'right_j1':0.8227529296875, 'right_j2':-2.0955126953125, 'right_j3':2.172509765625, 'right_j4':0.7021171875, 'right_j5' : -1.5003603515625, 'right_j6' : -2.204990234375}) 

    def runjoints(self, jlist):
        for i in jlist[0]:
            print i
            self.limb_interface.set_joint_velocities(i)             

