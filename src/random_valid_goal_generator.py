#!/usr/bin/env python
"""
Jarvis Schultz
Feb 2016

SUBSCRIPTIONS:

PUBLISHERS:
    - ref_pose (PoseStamped)

SERVICES:

PARAMS:
    - freq ~ frequency that we should be publishing at (sets controller frequency)
"""

# ROS imports
import rospy
import tf
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from std_srvs.srv import EmptyRequest
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest

# misc imports:
import numpy as np
import warnings

# local imports:
import kbhit
import custom_logging as cl
import modern_robotics as mr
import sawyer_MR_description as smr


####################
# global constants #
####################
# default publish frequency
FREQ = 100
# frame that the desired pose should be published in
FRAME = "base"
TARGET = "target"
EE_FRAME = "right_hand"
CENTER_Z = 0.317
CENTER_X = 0.7
CENTER_Y = 0.0
RANGE_X = 0.3
RANGE_Y = 0.3


class ReferencePublisher( object ):
    def  __init__(self):
        rospy.loginfo("Creating ReferencePublisher class")

        # create flags
        self.pub_flag = False
        self.controller_flag = False
        self.cube_flag = False
        self.area_flag = False

        # get params:
        self.get_and_set_params()

        # kbhit instance
        self.kb = kbhit.KBHit()
        rospy.on_shutdown(self.kb.set_normal_term)
        
        # create publisher and timers:
        self.ref_pub = rospy.Publisher("ref_pose", PoseStamped, queue_size=3)
        self.joints_pub = rospy.Publisher("joint_states", JointState, queue_size=3)
        self.br = tf.TransformBroadcaster()
        self.pub_timer = rospy.Timer(rospy.Duration(self.dt), self.pubtimercb)
        self.base_time = rospy.Time.now()
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)

        # create service clients
        rospy.wait_for_service("reset")
        self.reset_client = rospy.ServiceProxy("reset", Empty)
        rospy.wait_for_service("random")
        self.random_client = rospy.ServiceProxy("random", Empty)
        rospy.wait_for_service("toggle_controller")
        self.toggle_controller_client = rospy.ServiceProxy("toggle_controller", SetBool)

        self.print_help()
        return
    

    def keycb(self, tdat):
        # check if there was a key pressed, and if so, check it's value and
        # toggle flag:
        if self.kb.kbhit():
            c = self.kb.getch()
            if ord(c) == 27:
                rospy.signal_shutdown("Escape pressed!")
            else:
                print c
            if c == 'p':
                rospy.loginfo("You pressed 'p', toggling publishing")
                self.pub_flag = not self.pub_flag
                rospy.loginfo("Publishing?\t"+str(self.pub_flag))
                if self.pub_flag:
                    self.base_time = rospy.Time.now()
            elif c == 'n':
                rospy.loginfo("You pressed 'n', generating new goal")
                self.generate_new_target()
            elif c == 'r':
                rospy.loginfo("You pressed 'r', generating random config for IK controller")
                self.random_client(EmptyRequest())
            elif c == 's':
                rospy.loginfo("You pressed 's', stopping and resetting IK controller")
                self.controller_flag = False
                self.reset_client(EmptyRequest())
            elif c == 't':
                self.controller_flag = not self.controller_flag
                rospy.loginfo("You pressed 't', toggling IK controller... Running: %s", self.controller_flag)
                self.toggle_controller_client(SetBoolRequest(self.controller_flag))
            elif c == 'c':
                self.cube_flag = not self.cube_flag
                rospy.loginfo("Toggling random pose in cube... In cube: %s", self.cube_flag)
            elif c == 'a':
                self.area_flag = not self.area_flag
                rospy.loginfo("Toggling random pose in area... In area: %s", self.area_flag)
            else:
                self.print_help()
            self.kb.flush()
        return


    def print_help(self):
        help_string = \
        """
        'p'   ~  Toggle publishing of reference SE(3) pose
        'n'   ~  Generate a new reference SE(3) pose
        'r'   ~  Choose random new config for IK controller
        's'   ~  Stop and reset IK controller
        't'   ~  Toggle IK controller functionality
        'c'   ~  Toggle whether random targets should be restricted to catching cube
        'a'   ~  Toggle whether random targets should be restricted to catching area        
        'ESC' ~  Quit
        """
        print help_string
        return
    

    def generate_new_target(self):
        if self.cube_flag:
            p = np.array([[np.random.uniform(.5, 1.0), np.random.uniform(-0.25,0.25), np.random.uniform(-0.05, 0.05)]]).T
            R = np.array([
                [0,0,1],
                [1,0,0],
                [0,1,0]])
            g = np.vstack(
                (np.hstack((R,p)),
                 np.hstack((np.zeros((1,3)),[[1]]))))
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                qtmp = self.kin.inverse_search(g)
            if qtmp is not None:
                q = qtmp
        elif self.area_flag:
            # x_home = 1.0155
            # y_home = 0.1603
            # z_home = 0.317
            g = tr.euler_matrix(0, np.pi/2, 0)
            g[0][3] = np.random.uniform(CENTER_X - RANGE_X, CENTER_X + RANGE_X)
            g[1][3] = np.random.uniform(CENTER_Y - RANGE_Y, CENTER_Y + RANGE_Y)
            g[2][3] = CENTER_Z
            p = np.array(g[0:3,-1]).ravel()
            (q, result) = mr.IKinBody(smr.Blist, smr.M, g, np.zeros(7), 0.01, 0.001)
        else:
            q = self.kin.random_joint_angles()
            g = self.kin.forward(q)
            p = np.array(g[0:3,-1]).ravel()
        self.p.pose.position = Point(*p.ravel())
        self.p.pose.orientation = Quaternion(*tr.quaternion_from_matrix(g))
        self.js.position = q
        return

        
    def pubtimercb(self, tdat):
        if self.pub_flag:
            self.p.header.stamp = tdat.current_expected
            self.js.header.stamp = tdat.current_expected
            self.ref_pub.publish(self.p)
            self.joints_pub.publish(self.js)
            p = np.array([self.p.pose.position.x, self.p.pose.position.y, self.p.pose.position.z])
            q = np.array([self.p.pose.orientation.x, self.p.pose.orientation.y, self.p.pose.orientation.z, self.p.pose.orientation.w])
            self.br.sendTransform(p, q, tdat.current_expected, TARGET, FRAME)
        return


    def get_and_set_params(self):
        self.freq = rospy.get_param("freq", FREQ)
        self.dt = 1/float(self.freq)
        if rospy.has_param("robot_description"):
            with cl.suppress_stdout_stderr():
                self.robot = URDF.from_parameter_server()
        else:
            rospy.logwarn("Need to add robot model to parameter server")
            rospy.sleep(2.)
            rospy.signal_shutdown("No robot model")
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.kin = KDLKinematics(self.robot, FRAME, EE_FRAME)
        # build empty JointState message:
        self.js = JointState()
        self.js.name = self.kin.get_joint_names()
        # build PoseStamped message:
        self.p = PoseStamped()
        self.p.header.frame_id = FRAME
        # generate initial target:
        self.generate_new_target()
        return




def main():
    rospy.init_node('random_reference_pose_publisher', log_level=rospy.INFO)

    try:
        posepub = ReferencePublisher()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()

