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

""" service for converting geometry_msgs/Pose to SO3 """

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


    #convert quarternion to SO3
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


def pose_to_SO3_server():
    rospy.init_node('pose_to_SO3_server')
    s = rospy.Service('pose_to_SO3', PoseToSO3, pose_to_SO3_calc)
    rospy.spin()


if __name__ == "__main__":
    pose_to_SO3_server()
