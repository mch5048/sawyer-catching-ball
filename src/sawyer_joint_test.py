#!/usr/bin/env python
from robot_calc_functions import MatrixLog6, matmult, TransInv, Magnitude, Adjoint, IKinBody, JacobianBody, FKinBody
import numpy as np
from math import radians, cos, sin
import matplotlib.pyplot as plt
import rospy 
from intera_core_msgs.msg import EndpointState, JointCommand
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from sensor_msgs.msg import (
    JointState
)
from sawyer_catching_ball.srv import (
    PoseToSO3
)


class SO3_TF(object):
    
    def __init__(self):
        # self.SE3 = np.array([[0.,0.,0.,0.],[0.,0.,0.,0.],[0.,0.,0.,0.],[0.,0.,0.,1.]])
        self.SO3 = np.zeros((4,4),dtype=np.float64)

class ThetaList(object):
    
    def __init__(self):
        # self.th_list = np.array([0,0,0,0,0,0,0], dtype=np.float64)
        self.th_list = np.zeros((7), dtype=np.float64)
    
def get_pres_joint_states(js_msg, thetalist):

    """
    obtain the present joint states from right_j0 to right_j6 for kinematics control law calculation
    """
    joint_name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']

    for i in range(7):
        # thetalist.th_list[i] = js_msg.position(joint_name[i])
        # print "js_msg",js_msg.position
        # thetalist.th_list[i] = js_msg.position[joint_name[i]]
        thetalist.th_list[i] = js_msg.position[i+1]
    # print js_msg.position
    # print thetalist.th_list
    # return thetalist

def joint_move_test(j_command_pub, j_command):

    print "into the loop"
    #timestep in secs
    timestep = 0.01

    #initialize loop counter and time used in calculation of the whole process
    publish_rate = 5  #Hz

    #publish joint command for each joint
    i = 0;
    
    while (i < 7):
        
        #record in joint command and publish
            

        #publish joint command
        j_command.mode = 2
        j_command.names = ['torso_t0' ,'right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6', 'head_camera']
        j_command.velocity = np.zeros((9), dtype = np.float64)
    
        j_command.velocity[i+1] = 0.034
        
        time = 0
        while (time < 10000):
            now = rospy.get_rostime()
            time_secs = now.secs
            time_nsecs = now.nsecs    
            j_command.header.stamp.secs = time_secs
            j_command.header.stamp.nsecs = time_nsecs
            j_command_pub.publish(j_command)
            #rate = rospy.Rate(5) #publish at 5 Hz suggested in http://sdk.rethinkrobotics.com/intera/Arm_Control_Systems#Joint_Command_Overview        
            rate = rospy.Rate(publish_rate) #for visual satisfction in rviz)
            time += 1
        i += 1
            


def pres_SO3(msg, x_pres):
    """get present SO3 of EE by converting from quarternion """

    #ask for service to convert quarternion to SO3
    rospy.wait_for_service('pose_to_SO3')
    try:
        pose_to_SO3 = rospy.ServiceProxy('pose_to_SO3', PoseToSO3)
        SO3_resp_1row = pose_to_SO3(msg.pose)  
        #reshape SO3 from flattened array 1*16 to 4*4
        # SO3_rs = np.zeros((4,4), dtype=np.float64)
        count = 0
        for i in range(4):
            for j in range(4):
                x_pres.SO3[i][j] = SO3_resp_1row.SO3[count]
                count+=1

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():

    rospy.init_node('sawyer_joint_test')


    #declare itself as a publisher
    j_command_pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 10)    #For real sawyer, the rate at which the command should be published is at /robot/limb/right/joint_command_timeout
    j_command = JointCommand()

    # kin_cont_law(x_pres.SO3, x_des, j_command_pub, j_command)

    joint_move_test(j_command_pub, j_command)


    #print "End"

    rospy.spin()

if __name__=="__main__":
    main()
