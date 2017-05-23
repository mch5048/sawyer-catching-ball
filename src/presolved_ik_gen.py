#!/usr/bin/env python
"""
generate presolved IK file
"""

###############
# ROS IMPORTS #
###############
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point

##################
# PYTHON IMPORTS #
##################
import os

#################
# LOCAL IMPORTS #
#################
import ik_controller_full_system as ik_cont
import sawyer_presolved_ik_list as sy_jls

# GLOBAL VARS
FILEPATH = os.getcwd()
Z_CENTER = 0.3164
CENTER_X = 0.56034
CENTER_Y = -0.1295634
RANGE_X = 0.15
RANGE_Y = 0.15
# RANGE_X = 0.6
# RANGE_Y = 0.6


class PresolvedIK( object ):
    def __init__(self):
        rospy.loginfo("Creating PresolvedIK class")

        # create IKController class
        self.ikcont = ik_cont.IKController()

        # input the point into the IKController
        self.makeFile("sawyer_presolved_ik_list.py")
        # self.home()
        # self.test()

    def home(self):
        self.ikcont.home()

    def test(self):
        b = [value for key, value in sy_jls.jlist.items() if key[0] == 0.71]
        self.ikcont.runjoints(b)
            

    def makeFile(self, file_name):
        temp_path = FILEPATH + '/' + file_name
        
        a = {}
        #iterate through every 1 cm in the plane
        # from x = [CENTER_X - RANGE_X, CENTER_X + RANGE_X]
        input_pose = Pose()

        input_pose.position = Point(round(RANGE_X + CENTER_X, 3), round(RANGE_Y + CENTER_Y, 3), Z_CENTER)
        input_pose.orientation = Quaternion(0.0392407571798, 0.664506667783, -0.0505321422468, 0.744538483926)
        a[(round(RANGE_X + CENTER_X, 3), round(RANGE_Y + CENTER_Y, 3), Z_CENTER)] = self.ikcont.gen_joints_list(input_pose)
        
        # for x in range(int(RANGE_X*100)):
        #     for y in range(int(RANGE_Y*100)):
        #         input_pose.position = Point(x/100 + CENTER_X, y/100 + CENTER_Y, Z_CENTER)
        #         input_pose.orientation = Quaternion(0.0392407571798, 0.664506667783, -0.0505321422468, 0.744538483926)
        #         print input_pose
        #         a[(x/100 + CENTER_X, y/100 + CENTER_Y, Z_CENTER)] = self.ikcont.gen_joints_list(input_pose)

        with open(temp_path, 'w') as f:
            f.write("jlist = {")
            for i in a:
                t = str(i) + ":" + str(a[i]) + ",\n"
                f.write(t)
            f.write("}")
        print 'Execution completed.'



def main():
    rospy.init_node("Presolved_ik")
    presolvedik = PresolvedIK()

if __name__=='__main__':
    main()


