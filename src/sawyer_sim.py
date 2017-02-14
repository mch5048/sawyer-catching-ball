#!/usr/bin/env python
import rospy
from intera_core_msgs.msg import (
    JointCommand,
    EndpointState
)
from geometry_msgs.msg import (
    Pose,
    Wrench,
    Twist
)
from std_msgs.msg import (
    Header
)
from sensor_msgs.msg import (
    JointState
)
from sawyer_catching_ball.srv import (
    FwdKin
)
from math import atan2, asin

"""
This node acts as sawyer_trajectory_server publishing joint_states and endpoint_state for sawyer simulation in rviz.
It can receive velocity input and move him.

"""

def js_eps_init(pres_joint_state, pres_endpoint, request_pos):
    """
    a list of joints position for each named position
    """

    #joint state initialization
    req_list = ['home']
    joint_sources = {'home': [0., 0., 0., 0., 0., 0., 0.]}
    joint_names = ['torso_t0' ,'right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6', 'head_camera']
    pres_joint_state.name = joint_names
    pres_joint_state.position = range(9)
    for i in req_list:
        if request_pos == i: 
            for j in range(9):
                if (j == 0 or j == 8):
                    pres_joint_state.position[j] = 0.
                else:
                    pres_joint_state.position[j] = (joint_sources[i])[j - 1]
    

    #endpoint state initialization
    
    # print "1"
    #ask for fwd kinematics service (input : thetalist, output : pose)
    rospy.wait_for_service('fwd_kin')
    # print "2"
    try:
        fwd_kin = rospy.ServiceProxy('fwd_kin', FwdKin)
        # print pres_joint_state.position
        #print pres_endpoint.pose
        pres_endpoint.pose = fwd_kin(pres_joint_state.position).pose #always extract the output out of response
        print pres_endpoint.pose
        return pres_joint_state, pres_endpoint
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e




# def velocity_control_server(joints_vel_command, pres_joint_state):
def velocity_control_server(joints_vel_command, (pres_joint_state, pres_endpoint)):
    """
    change the currect joint position based on the velocity command with respect to time_step
    We care only joint0 to joint 6.
    """
    #time step in sec
    dt = 0.2 #5 Hz = 0.2 secs 
    
    #change joint_state according to dt and JointCommand
    for i in range(9):        
        if (i == 0 or i == 8):
            pres_joint_state.position[i] = 0.
        else:
            pres_joint_state.position[i] += joints_vel_command.velocity[i] * dt
            
    
    #change endpoint according to dt and JointCommand
    #ask for fwd kinematics service (input: thetalist of joinstate, output : pose)
    rospy.wait_for_service('fwd_kin')
    try:
        fwd_kin = rospy.ServiceProxy('fwd_kin', FwdKin)
        #print "before", pres_endpoint.pose
        pres_endpoint.pose = fwd_kin(pres_joint_state.position).pose #always extract the output out of response
        #print "After", pres_endpoint.pose
        # return pres_joint_state, pres_endpoint
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    #print the angle of x y and z according to the base frame
    # q0 = pres_endpoint.pose.orientation.w
    # q1 = pres_endpoint.pose.orientation.x
    # q2 = pres_endpoint.pose.orientation.y
    # q3 = pres_endpoint.pose.orientation.z
    # print "roll : ", atan2(2*(q0*q1 + q2*q3), (1-2*(q1**2 + q2**2)))
    # print "pitch : ", asin(2*(q0*q2 - q3*q1))
    # print "yaw : ", atan2(2*(q0*q3 + q2*q1), (1-2*(q2**2 + q3**2)))


def main():
    rospy.init_node("sawyer_sim")
    pres_joint_state = JointState() 
    pres_endpoint = EndpointState()
    pres_joint_state, pres_endpoint = js_eps_init(pres_joint_state, pres_endpoint,'home')    

    #print pres_joint_state, pres_endpoint

    #if there's a published msg of velocity controller then move joint
    # rospy.Subscriber("/robot/limb/right/joint_command", JointCommand, velocity_control_server, pres_joint_state) 
    # print pres_joint_state
    # print pres_endpoint
    rospy.Subscriber("/robot/limb/right/joint_command", JointCommand, velocity_control_server, (pres_joint_state, pres_endpoint)) 

    pub_ep = rospy.Publisher("/robot/limb/right/endpoint_state", EndpointState, queue_size = 10)
    pub_js = rospy.Publisher("/robot/joint_states", JointState, queue_size = 10)
    rate = rospy.Rate(200) #200 Hz

    while not rospy.is_shutdown():

        now = rospy.get_rostime()
        time_secs = now.secs
        time_nsecs = now.nsecs

        pres_joint_state.header.stamp.secs = time_secs
        pres_joint_state.header.stamp.nsecs = time_nsecs
        pres_endpoint.header.stamp.secs = time_secs
        pres_endpoint.header.stamp.nsecs = time_nsecs

        pub_js.publish(pres_joint_state)
        pub_ep.publish(pres_endpoint)
        rate.sleep()
        #rospy.spin() #seems like rospy.spin() leads to error in sending, it might be from conflict between rospy.spin() and rate.sleep()


if __name__=="__main__":
    main()

