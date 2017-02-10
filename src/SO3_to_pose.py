#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import (
    Pose
)
from sawyer_catching_ball.srv import (
    SO3ToPose
)

""" service for converting SO3 to geometry_msgs/Pose"""

# from Jarvis Schultz's quarternion calculation
def so3_to_axis_angle(R):
    th = np.arccos((R.trace() - 1)/2)
    if (th <= 1e-12):
        th = 0.0
        w = 3*[1/np.sqrt(3)]
    else:
        w = 1/(2*np.sin(th))*np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
        if any(map(np.isinf, w)) or any(map(np.isnan, w)):
            th = 0.0
            w = 3*[1/np.sqrt(3)]
    return w, th

def axis_angle_to_quat(w, th):
    return np.array(np.hstack((np.cos(th/2.0), np.array(w)*np.sin(th/2.0))))

def so3_to_quat(R):
    w,th = so3_to_axis_angle(R)
    return axis_angle_to_quat(w, th)

def SO3_to_pose_calc(req):

    # SO3 = req.SO3.reshape(4, 4)
    
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


def SO3_to_pose_server():
    rospy.init_node('SO3_to_pose_server')
    s = rospy.Service('SO3_to_pose', SO3ToPose, SO3_to_pose_calc)
    rospy.spin()


if __name__ == "__main__":
    SO3_to_pose_server()
