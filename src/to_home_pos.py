#!/usr/bin/env python
import rospy
from sawyer_catching_ball.srv import IsHome

def srv_callback(ss):
    # access to sawyer joint velocity command

    # get the present transformation 

    # tell the desired transformation

    # ask for ik_cont_service to move from the present to the desired position

    # check if the processs finish or the tf is at home

    # return boolean after finish

    # End



def main():
    rospy.init_node("to_home_pos")
    s = rospy.Service("to_home_pos", IsHome, srv_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
