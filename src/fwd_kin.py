#!/usr/bin/env python
from sawyer_catching_ball.srv import FwdKin, SO3ToPose
import rospy
from robot_calc_functions import FKinBody
import numpy as np
from math import cos, sin, radians

def fwd_kin_calc(req):

    #check if there is 9 joint angles
    if len(req.position) != 9:
        print "Please input an array with 9 angles to fwd_kin_srv"
        exit()

    #extract 7 joints angle from 9 angles in thetalist
    th_list = range(7)
    for i in range(8):
        if i != 0 or i != 8:
            th_list[i-1] = req.position[i]

    #specify M and Blist
    s10 = sin(radians(10)) 
    c10 = cos(radians(10))
    # Blist = np.array([[s10, -c10, 0., -1.0155*c10, -1.0155*s10, 0.1603],
    #                   [-c10, -s10, 0., -0.9345*s10, 0.9345*c10, 0.],
    #                   [0. , 0., 1., -0.0322*s10, 0.0322*c10, 0.],
    #                   [-c10, -s10, 0., -0.5345*s10, 0.5345*c10, 0.],
    #                   [0., 0., 1., 0.1363*s10, -0.1363*c10, 0.],
    #                   [-c10, -s10, 0., -01345*s10, 0.1345*c10, 0.],
    #                   [0., 0., 1., 0., 0., 0.]], 
    #                  dtype=np.float64)
    #check at Mon Feb 6 change[0][5] and [5][3]
    Blist = np.array([[s10, -c10, 0., -1.0155*c10, -1.0155*s10, -0.1603],
                      [-c10, -s10, 0., -0.9345*s10, 0.9345*c10, 0.],
                      [0. , 0., 1., -0.0322*s10, 0.0322*c10, 0.],
                      [-c10, -s10, 0., -0.5345*s10, 0.5345*c10, 0.],
                      [0., 0., 1., 0.1363*s10, -0.1363*c10, 0.],
                      [-c10, -s10, 0., -0.1345*s10, 0.1345*c10, 0.],
                      [0., 0., 1., 0., 0., 0.]], 
                     dtype=np.float64)

    M = np.array([[0., 0., 1., 1.0155],
                  [-c10, -s10, 0., 0.1603],
                  [s10, -c10, 0., 0.317],
                  [0., 0., 0., 1.]], dtype=np.float64)
    
    # get SO3 of EE
    EE_SO3 = FKinBody(M, Blist, th_list)

    # ask for service to convert SE3 to Pose, position and Quarternion
    rospy.wait_for_service('SO3_to_pose')
    print "SO3 TO pose is available"
    try:
        SO3_to_Pose = rospy.ServiceProxy('SO3_to_pose', SO3ToPose)
        pose_output = SO3_to_Pose(EE_SO3.flatten())  #flatten array to be passable for float64[] type of msg
        # return FwdKinResponse(pose_output)
        print pose_output.pose
        return pose_output.pose
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def fwd_kin_server():
    rospy.init_node('fwd_kin_server')
    s = rospy.Service('fwd_kin', FwdKin, fwd_kin_calc)
    rospy.spin()


if __name__ == "__main__":
    fwd_kin_server()
