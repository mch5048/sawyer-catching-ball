#!/usr/bin/env python
"""
Jarvis Schultz
Feb 2016

This file subscribes to the reference poses, and then 

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
import rospkg
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from std_srvs.srv import EmptyRequest
from std_srvs.srv import EmptyResponse
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest
from std_srvs.srv import SetBoolResponse
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand, EndpointState
import tf.transformations as tr
import tf
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
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
TARGET = "target"
BASE = "base"
ARM = "right"
EE_FRAME = "right_hand"
FREQ = 100
STEP_SCALE = 1.0
DAMPING = 0.02
TOLERANCE = 1e-4
JOINT_CMD_FREQ = 100
JOINT_VEL_LIMIT = 1.5
CENTER_Z = 0.317
CENTER_X = 0.7
CENTER_Y = 0.0
RANGE_X = 0.6
RANGE_Y = 0.6
RANGE_Z = 0.3

class IKController( object ):
    def __init__(self):
        rospy.loginfo("Creating IK Controller class")


        # setup flags and useful vars:
        self.running_flag = False
        self.freq = rospy.get_param("~freq", FREQ)
        self.arm = rospy.get_param("~arm", ARM)
        self.base = rospy.get_param("~base", BASE)
        self.target = rospy.get_param("~target", TARGET)
        self.step_scale = rospy.get_param("~scale", STEP_SCALE)
        self.tolerance = rospy.get_param("~tolerance", TOLERANCE)
        self.damping = rospy.get_param("~damping", DAMPING)
        self.center_x = rospy.get_param("~center_x", CENTER_X)
        self.center_y = rospy.get_param("~center_y", CENTER_Y)
        self.center_z = rospy.get_param("~center_z", CENTER_Z)
        self.range_x = rospy.get_param("~range_x", RANGE_X)
        self.range_y = rospy.get_param("~range_y", RANGE_Y)
        self.joint_vel_limit = rospy.get_param("~joint_vel_limit", JOINT_VEL_LIMIT)
        with cl.suppress_stdout_stderr():
            self.urdf = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.urdf, "base", "%s_hand"%self.arm)
        self.q = np.zeros(7)
        self.q_sim = np.zeros(7)
        self.limb_interface = intera_interface.Limb()
        self.center_pos()
        self.goal = np.array(self.kin.forward(self.q))
        self.js = JointState()
        self.js.name = self.kin.get_joint_names()
        self.pose = PoseStamped()
        self.mutex = threading.Lock()
        self.joint_cmd = JointCommand()
        self.joint_cmd.names = self.kin.get_joint_names()
        self.joint_cmd.mode = 2


        # create all services:
        self.toggle_server = rospy.Service("toggle_controller", SetBool, self.toggle_handler)
        self.reset_server = rospy.Service("reset", Empty, self.reset_handler)
        self.random_server = rospy.Service("random", Empty, self.random_handler)

        # create all subscribers, timers, and publishers:
        self.ref_sub = rospy.Subscriber("ref_pose", PoseStamped, self.refcb)
        self.actual_js_sub = rospy.Subscriber("robot/joint_states", JointState, self.actual_js_cb)
        self.js_pub = rospy.Publisher("joint_states", JointState, queue_size=3)
        self.pose_pub = rospy.Publisher("pose", PoseStamped, queue_size=3)
        self.joint_cmd_timeout_pub = rospy.Publisher("robot/limb/right/joint_command_timeout", Float64, queue_size=3)
        self.center_pos()
        self.joint_cmd_pub = rospy.Publisher("robot/limb/right/joint_command", JointCommand, queue_size=3)
        self.int_timer = rospy.Timer(rospy.Duration(1/float(self.freq)), self.ik_step_timercb)
        return
        
    def center_pos(self):
        # center_tf = tr.euler_matrix(0, np.pi/2, 0)
        # center_tf[0][3] = self.center_x + 0.2
        # center_tf[1][3] = self.center_y
        # center_tf[2][3] = self.center_z
        # (qgoal, result) = mr.IKinBody(smr.Blist, smr.M, center_tf, np.zeros(7), 0.01, 0.001)

        # publish joint position for start position
        rospy.loginfo("in center_pos now")
        # center_joint_cmd = JointCommand()
        # center_joint_cmd.position = [0.379125, -0.020025390625, 0.8227529296875, -2.0955126953125, 2.172509765625, 0.7021171875, -1.5003603515625, -2.204990234375, 0.0]
        # center_joint_cmd.names = ['head_pan', 'right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6', 'torso_t0']
        self.limb_interface.move_to_joint_positions({'head_pan':0.379125, 'right_j0':-0.020025390625 , 'right_j1':0.8227529296875, 'right_j2':-2.0955126953125, 'right_j3':2.172509765625, 'right_j4':0.7021171875, 'right_j5' : -1.5003603515625, 'right_j6' : -2.204990234375})
        rospy.loginfo("finish set joint")
        # center_joint_cmd.names = self.kin.get_joint_names()
        # center_joint_cmd.mode = 1
        # center_joint_cmd.position = qgoal
        # center_joint_cmd.header.stamp = rospy.Time.now()
        # center_joint_cmd_pub = rospy.Publisher("robot/limb/right/joint_command", JointCommand, queue_size=3)
        # center_joint_cmd_pub.publish(center_joint_cmd)
        return 


    def refcb(self, pose):
        with self.mutex:
            p = np.array([pose.pose.position.x, pose.pose.position.y,
                          pose.pose.position.z])
            q = np.array([pose.pose.orientation.x, pose.pose.orientation.y,
                          pose.pose.orientation.z, pose.pose.orientation.w])
            self.goal = tr.euler_matrix(*tr.euler_from_quaternion(q))
            self.goal[0:3,-1] = p
        return

    def actual_js_cb(self, js):
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

    def toggle_handler(self, request):
        rospy.loginfo("Requesting to set running_flag to {0:}".format(request.data))
        self.running_flag = request.data
        if request.data:
            resp = SetBoolResponse(True, "Running")
        else:
            resp = SetBoolResponse(True, "Stopped")
        return resp

    def reset_handler(self, request):
        rospy.loginfo("Requesting to reset arm and stop controller")
        self.running_flag = False
        self.q_sim = np.zeros(7)
        return EmptyResponse()

    def random_handler(self, request):
        rospy.loginfo("Requesting to move arm to random config")
        self.q_sim = self.kin.random_joint_angles()
        return EmptyResponse()

    def ik_step_timercb(self, tdat):
        if self.running_flag:
            self.step_ik()
            # publish joint command timeout
            self.joint_cmd_timeout_pub.publish(JOINT_CMD_FREQ)
            # publish joint command message
            self.joint_cmd.header.stamp = tdat.current_expected
            self.joint_cmd_pub.publish(self.joint_cmd)
            # print self.joint_cmd
        self.js.header.stamp = tdat.current_expected
        self.js.position = self.q_sim
        self.js_pub.publish(self.js)
        # publish pose
        self.fwd_kin()
        self.pose.header.stamp = tdat.current_expected
        self.pose.header.frame_id = BASE
        self.pose_pub.publish(self.pose)
        return

    def fwd_kin(self):
        self.robot = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.robot, BASE, EE_FRAME)
        g = self.kin.forward(self.q)
        p = np.array(g[0:3,-1]).ravel()
        self.pose.pose.position = Point(*p.ravel())
        self.pose.pose.orientation = Quaternion(*tr.quaternion_from_matrix(g))
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
        # print np.amax(qdot), " --to--> ", np.amax(qdot*multiplier)
        qdot = qdot*multiplier
        # clip down velocity in each joint
        if np.amax(qdot) > self.joint_vel_limit:
            qdot = qdot/np.amax(qdot)*self.joint_vel_limit
        names = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        # print "max vel joint : ", names[qdot.index(np.amax(qdot))], " : ", qdot[qdot.index(np.amax(qdot))]
        # print "max vel joint : ", names[np.where(qdot == np.amax(qdot))[0]], " : ", qdot[np.where(qdot == np.amax(qdot))[0]]

        if np.amax(qdot) > self.joint_vel_limit:
            print "joint limit reach !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11"
    
        self.q_sim += self.step_scale*qdot
        self.joint_cmd.velocity = qdot 
        return

class Safety ( object ):

    def __init__(self):
        self.center_x = rospy.get_param("~center_x", CENTER_X)
        self.center_y = rospy.get_param("~center_y", CENTER_Y)
        self.center_z = rospy.get_param("~center_z", CENTER_Z)
        self.range_x = rospy.get_param("~range_x", RANGE_X)
        self.range_y = rospy.get_param("~range_y", RANGE_Y)
        self.range_z = rospy.get_param("~range_z", RANGE_Z)
        #uncomment either pose_sub or ep_sub for working with either rviz or real Sawyer
        # self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.pose_cb)
        self.ep_sub = rospy.Subscriber("robot/limb/right/endpoint_state", EndpointState, self.ep_cb)
        #check if the endpoint state is in range or not, if not toggle global flag as False
        rospy.wait_for_service("reset")
        self.reset_client = rospy.ServiceProxy("reset", Empty)
    
    def ep_cb(self, ep):
        self.x_pres = ep.pose.position.x 
        self.y_pres = ep.pose.position.y
        self.z_pres = ep.pose.position.z
        self.safety_check()

    def pose_cb(self, pose):
        self.x_pres = pose.pose.position.x 
        self.y_pres = pose.pose.position.y
        self.z_pres = pose.pose.position.z
        self.safety_check()


    def safety_check(self):
        if (self.y_pres < self.center_y - self.range_y) or \
           (self.y_pres > self.center_y + self.range_y) or \
           (self.x_pres < self.center_x - self.range_x) or \
           (self.x_pres > self.center_x + self.range_x) or \
           (self.z_pres < self.center_z - self.range_z) or \
           (self.z_pres > self.center_z + self.range_z) :
            rospy.loginfo("out of threshold")
            self.reset_client(EmptyRequest())

def main():
    rospy.init_node('ik_controller_kinematics', log_level=rospy.INFO)

    try:
        controller = IKController()
        safety = Safety()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()

