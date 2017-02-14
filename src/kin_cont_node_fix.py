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
    PoseToSO3,
    FwdKin
)


class SO3_TF(object):
    
    def __init__(self):
        self.SO3 = np.zeros((4,4),dtype=np.float64)

class ThetaList(object):
    
    def __init__(self):
        self.th_list = np.zeros((7), dtype=np.float64)
    
def get_pres_joint_states(js_msg, thetalist):

    """
    obtain the present joint states from right_j0 to right_j6 for kinematics control law calculation
    """
    joint_name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']

    for i in range(7):
        thetalist.th_list[i] = js_msg.position[i+1]

def kin_cont_law(x_start, x_des, j_command_pub, j_command):

    #Controller gain
    Kp = 0
    Ki = 0

    #timestep in secs
    timestep = 0.01

    #set the limit of precision between the start and desired position
    lim_w_bd = 0.01 #in rad
    lim_v_bd = 0.001 #in m


    #calculate twist for moving from present pose to desired pose
    twist_bd = MatrixLog6(matmult(TransInv(x_start),x_des))
    w_bd = Magnitude([twist_bd[0], twist_bd[1], twist_bd[2]])
    v_bd = Magnitude([twist_bd[3], twist_bd[4], twist_bd[5]])
    print "error magnitude"
    print "w_bd = ", w_bd
    print "v_bd = ", v_bd

    #Resources for pseudo inverse Jacobian
    c10 = cos(radians(10))
    s10 = sin(radians(10))
    #numbers from urdf opened in vrep
    #New Blist from position of right_arm_mount and orientation of base_link_visual to right_hand according to vrep
    Blist = np.array([[s10, -c10, 0., -1.0155*c10, -1.0155*s10, -0.1603],
                      [-c10, -s10, 0., -0.9345*s10, 0.9345*c10, 0.],
                      [0. , 0., 1., -0.0322*s10, 0.0322*c10, 0.],
                      [-c10, -s10, 0., -0.5345*s10, 0.5345*c10, 0.],
                      [0., 0., 1., 0.1363*s10, -0.1363*c10, 0.],
                      [-c10, -s10, 0., -0.1345*s10, 0.1345*c10, 0.],
                      [0., 0., 1., 0., 0., 0.]], 
                     dtype=np.float64)

    #get present joint_state
    thetalist_pres_class = ThetaList() 
    rospy.Subscriber("/robot/joint_states", JointState, get_pres_joint_states, thetalist_pres_class)
    rospy.sleep(0.07) #make sure joint_state is received
    

    #set the present trans to be the start one
    x_pres = x_start 

    #set the present feed-fwd twist to be between start and des
    # twist_ffwd = twist_bd
    #set the summation variable of error for Ki
    # x_error_sum = np.array([0,0,0,0,0,0], dtype=np.float64)
    x_error_sum = np.zeros((6), dtype=np.float64)
    x_error = np.zeros((6), dtype=np.float64)
    # x_error = np.array([0,0,0,0,0,0], dtype=np.float64)

    #initialize loop counter and time used in calculation of the whole process
    i = 0
    time_used = 0
    publish_rate = 5  #publish at 5 Hz suggested in http://sdk.rethinkrobotics.com/intera/Arm_Control_Systems#Joint_Command_Overview
    time_limit = 1000 #secs


 

    #continue the loop if the twist is bigger than limit and the counter is within range
    while ((w_bd > lim_w_bd or v_bd > lim_v_bd) and time_used < time_limit):
    

        #feed-forward twist between present and desired one
        ffw_portion = matmult(Adjoint(matmult(TransInv(x_pres),x_des)),twist_bd)
        #print "ffw_portion : ", ffw_portion

        #proportional gain multiplication
        kp_portion = [Kp * x for x in x_error]
        print "kp_portion : ", kp_portion

        #sum error for Pi
        x_error_sum = x_error + x_error_sum
        ki_portion = [Ki * x for x in x_error_sum]
        print "ki_portion : ", ki_portion

        #get twist b(t)
        twist_input = ffw_portion + kp_portion + ki_portion
        print "twist input : ", twist_input

        #get a set of joint velocity from twist body using pseudo inverse
        vel_set = matmult(np.linalg.pinv(JacobianBody(Blist, thetalist_pres_class.th_list)), twist_input)
        print "vel_set : ", vel_set
      
        #publish joint command
        j_command.mode = 2
        j_command.names = ['torso_t0' ,'right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6', 'head_camera']
        j_command.velocity = np.zeros((9), dtype = np.float64)
        for k in range(7):
            j_command.velocity[k+1] = vel_set[k]
        now = rospy.get_rostime()
        time_secs = now.secs
        time_nsecs = now.nsecs    
        j_command.header.stamp.secs = time_secs
        j_command.header.stamp.nsecs = time_nsecs
        #print j_command
        
        j_command_pub.publish(j_command)
        rate = rospy.Rate(publish_rate)
        
        #get x_pre
        rospy.Subscriber("/robot/joint_states", JointState, get_pres_joint_states, thetalist_pres_class)
        rospy.sleep(0.07)
        #print "Updated thetalist : ", thetalist_pres_class.th_list
        
        #change form 7 joints array to 9 joints array for FwdKin service 
        vel_set_dummy = np.zeros((9), dtype = np.float64)
        for m in range(9):
            if m == 0 or m == 8:
                pass
            else:
                vel_set_dummy[m] = thetalist_pres_class.th_list[m-1]
        #print "vel_set_dummy for fwd_kin : ", vel_set_dummy

        #ask for service 'FwdKin' to convert from joint_state to X
        rospy.wait_for_service('fwd_kin')
        try:
            fwd_kin = rospy.ServiceProxy('fwd_kin', FwdKin)
            x_pres_pose = fwd_kin(vel_set_dummy).pose #always extract the output out of response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        #print "x_pres_pose : ", x_pres_pose
        

        #convert pose to SE3    
        rospy.wait_for_service('pose_to_SO3')
        try:
            pose_to_SO3 = rospy.ServiceProxy('pose_to_SO3', PoseToSO3)
            x_pres_flatten_resp = pose_to_SO3(x_pres_pose) #always extract the output out of response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        #reshape x_pres_flatten to x_pres
        #reshape 1*16 tuple to 4*4 array
        x_pres = np.zeros((4,4), dtype=np.float64)
        count = 0
        for j1 in range(4):
            for j2 in range(4):
                x_pres[j1][j2] = x_pres_flatten_resp.SO3[count]
                count+=1
        #print "x_pres : ", x_pres


        #calculate x_error, the error twist, from log
        x_error = MatrixLog6(matmult(TransInv(x_pres),x_des))
        print "x_error : ", x_error

        #update nest twist_bd for ffw_portion
        twist_bd = x_error

        #Collect the precision
        w_bd = Magnitude([x_error[0], x_error[1], x_error[2]])
        v_bd = Magnitude([x_error[3], x_error[4], x_error[5]])
        print "error magnitude"
        print "w_bd = ", w_bd
        print "v_bd = ", v_bd

        time_used = time_used + (1.0/publish_rate)
        

        #print "w_bd : ", w_bd, "v_bd : ", v_bd
        print "i = ",i, "  :  ", j_command.velocity, "\n"
        print "time_used : ", time_used
        #print "x_pres\n", x_pres, "\n"
        #print vel_set, "\n"
        # print "x_error : \n", x_error
        print "-----------------------------------------------------------------------------\n"
        i = i + 1

    # for m in joint_vel_list:
    #     print m
    print "used time : ", time_used, " secs"


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

    rospy.init_node('kin_cont_law')

    # x_start = np.array([[0.,0.,1.,0.5],[0.,-1.,0.,0.],[1.,0.,0.,0.081],[0.,0.,0.,1.]])

    #subscribe to the EndpointState to get the present SE(3)
    x_pres = SO3_TF()
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, pres_SO3, x_pres, queue_size = 1)


    #need rospy.sleep to make sure the callback is run
    rospy.sleep(0.08)
    # print x_pres.SO3

    #the desired pose
    #x_des = np.array([[0.,0.,1.,-0.681105053298],[-1.,0.,0.,0.331671411775],[0.,-1.,0.,0.707913021448],[0.,0.,0.,1.]], dtype=np.float64)
    c10 = cos(radians(10))
    s10 = sin(radians(10))
    x_des = np.array([[0.,   0.,  1.,  0.2],
                      [-c10, -s10,0.,  0.],
                      [s10,  -c10,0.,  0.317],
                      [0.,  0.,  0.,  1.]], dtype=np.float64)
    # home EE tf 
    # M = np.array([[0., 0., 1., 1.0155],
    #               [-c10, -s10, 0., 0.1603],
    #               [s10, -c10, 0., 0.317],
    #               [0., 0., 0., 1.]], dtype=np.float64)




    #declare itself as a publisher
    j_command_pub = rospy.Publisher("/robot/limb/right/joint_command", JointCommand, queue_size = 10)    #For real sawyer, the rate at which the command should be published is at /robot/limb/right/joint_command_timeout
    j_command = JointCommand()

    kin_cont_law(x_pres.SO3, x_des, j_command_pub, j_command)



    #print "End"

    rospy.spin()

if __name__=="__main__":
    main()
