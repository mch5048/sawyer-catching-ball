#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import (
    Pose
)
from sawyer_catching_ball.srv import (
    PoseToSO3
)
from robot_calc_functions import matmult

""" service for converting SO3 to geometry_msgs/Pose"""

# from Jarvis Schultz's quarternion calculation
def quat_to_axis_angle(Q):
    th = 2*np.arccos(Q[0])
    if np.abs(th) < 1e-12:
        w = np.zeros(3)
    else:
        w = Q[1:]/np.sin(th/2.0)
    return w, th

def hat(w):
    return np.array([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]])


def axis_angle_to_so3(w, th):
    return np.eye(3) + matmult(hat(w),np.sin(th)) + matmult(hat(w),hat(w))*(1-np.cos(th))

def quat_to_so3(Q):
    w, th = quat_to_axis_angle(Q)
    return axis_angle_to_so3(w, th)


def pose_to_SO3_calc(req):
    
    qx = req.pose.orientation.x
    qy = req.pose.orientation.y
    qz = req.pose.orientation.z
    qw = req.pose.orientation.w


    #convert quarternion to SE3
    Q = [qw, qx, qy, qz]
    R = quat_to_so3(Q)

    SE3 = np.zeros((4,4),dtype=np.float64)

    #fill in SE3
    SE3[0][3] = req.pose.position.x
    SE3[1][3] = req.pose.position.y    
    SE3[2][3] = req.pose.position.z
    SE3[3][3] = 1.
    for i in range(3):
        for j in range(3):
            SE3[i][j] = R[i][j]
    
    #flatten SE3 for output
    return SE3.flatten()


    #reshape 1*16 tuple to 4*4 array
    SO3_rs = np.zeros((4,4), dtype=np.float64)
    count = 0
    for i in range(4):
        for j in range(4):
            SO3_rs[i][j] = req.SO3[count]
            count+=1

    #extract the rotational matrix portion out of SE3
    R = np.zeros((3,3),dtype=np.float64)
    for row in range(3):
        for column in range(3):
            R[row][column] = SO3_rs[row][column]

    pose = Pose()


    #get quat and put in Pose
    quat = so3_to_quat(R)
    pose.orientation.w = quat[0] 
    pose.orientation.x = quat[1]
    pose.orientation.y = quat[2]
    pose.orientation.z = quat[3]

    #put position on Pose
    pose.position.x = SO3_rs[0][3]
    pose.position.y = SO3_rs[1][3]
    pose.position.z = SO3_rs[2][3]

    return pose


def pose_to_SO3_server():
    rospy.init_node('pose_to_SO3_server')
    s = rospy.Service('pose_to_SO3', PoseToSO3, pose_to_SO3_calc)
    rospy.spin()


if __name__ == "__main__":
    pose_to_SO3_server()
