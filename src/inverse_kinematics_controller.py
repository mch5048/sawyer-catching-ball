#!/usr/bin/env python
"""
Chainatee Tanakulrungson
Feb 2016

SUBSCRIPTIONS:
    - 

PUBLISHERS:
    - ref_pose (PoseStamped)
    - joint_state (JointState)
SERVICES:

PARAMS:
    - freq ~ frequency that we should be publishing at (sets controller frequency)
"""

# ROS import:
import rospy
# import trac_ik.hpp as trac_ik
# import tf2
import tf.transformations as tr
from pykdl_utils.kdl_kinematics import KDLKinematics
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool,Empty,SetBoolResponse, EmptyResponse

# misc import:
import numpy as np



# local imports:
import modern_robotics as mr
import sawyer_MR_description as smr

###################
# global constant #
###################
#default publish frequency
FREQ = 100
BASE_FRAME = 'base' 
EE_FRAME = 'right_hand'
TOLERANCE = 1e-5
MAXTIME = 0.005
STEP_SIZE = 0.01

class IKController:

    def __init__(self):
        rospy.loginfo("Create the IKController class")
        
        # set flags
        self.ik_running_flag = False
        
        # set variables
        self.base = rospy.get_param("~base", BASE_FRAME)
        self.ee = rospy.get_param("~ee", EE_FRAME)
        self.urdf_string = rospy.get_param("robot_description")
        self.tolerance = rospy.get_param("~tolerance", TOLERANCE)
        self.max_time = rospy.get_param("~max_time", MAXTIME)
        self.step_size = rospy.get_param("~step_size", STEP_SIZE) 
        self.urdf = URDF.from_parameter_server()
        self.kin = KDLKinematics(self.urdf, self.base, self.ee)
        self.freq = rospy.get_param("~freq", FREQ)
        self.q = np.zeros(7) 
        self.js = JointState()

        # create services
        self.toggle_server = rospy.Service("toggle_controller", SetBool, self.toggle_handler)
        self.reset_server = rospy.Service("reset_controller", Empty, self.reset_handler)


        # create subscribers and publishers
        self.ref_pose_sub = rospy.Subscriber("reference/pose", PoseStamped, self.pose_cb)
        self.js_pub = rospy.Publisher("joint_state", JointState, queue_size=10)
        self.js_pub_timer = rospy.Timer(1/float(self.freq), self.js_pub_timer_cb)
        return

        # begin the IK call if being asked by main node, set the IK_request flag
        # get first and desired position
        # ask for trac_ik service : ik_solver(KDL::Chain chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed)

    def toggle_handler(self,request):
        self.ik_running_flag = request.data
        if request.data():
            resp = SetBoolResponse(True, "IK service : run")
        else:
            resp = SetBoolResponse(False, "IK service : stop")
        return resp

    def reset_handler(self,request):
        self.ik_running_flag = False
        return EmptyResponse()

    def js_pub_timer_cb(self, event):
        if self.ik_running_flag:
            self.IK_calc()
        self.js.header.stamp = event.current_expected
        self.js.position = self.q
        self.js_pub(self.js)
        return

    def pose_cb(self, ref_pos):
        # self.pose_des = tf2.convert.fromMsg(ref_pos)
        q = np.array([ref_pos.pose.orientation.x, ref_pos.pose.orientation.y, ref_pos.pose.orientation.z, ref_pos.pose.orientation.w])
        p = np.array([ref_pos.pose.position.x, ref_pos.pose.position.y, ref_pos.pose.position.z])
        self.pose_des = tr.euler_matrix(*tr.euler_from_quaternion(q))
        self.pose_des[:3,3] = p
        return


    def IK_calc(self):
        # grab current joint angles and SE(3) target:
        q = self.q
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
        self.q += self.step_size*qdot
        return


def main():
    rospy.init_node('IK_controller')

    try:
        posepub = IKController()
    except rospy.ROSInterruptException: pass

    rospy.spin()



if __name__=='__main__':
    main()
