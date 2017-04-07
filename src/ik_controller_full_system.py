#!/usr/bin/env python
"""
Chainatee Tanakulrungson
Mar 2017

This file subscribes to the ball frame, and estimates the position of the ball on a plane 

SUBSCRIPTIONS:
    - /base to /ball (LookUpTransform)
    - 

PUBLISHERS:

SERVICES:

PARAMS:

"""

###############
# ROS IMPORTS #
###############
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped
import tf
import tf.transformations as tr
from pykdl_utils.kdl_kinematics import KDLKinematics
import intera_interface 

##################
# PYTHON IMPORTS #
##################
import numpy as np
import threading
from urdf_parser_py.urdf import URDF
import matplotlib.pyplot as plt

#################
# LOCAL IMPORTS #
#################
import kbhit
import sawyer_catch_ball_calc as sawyer_calc


# GLOBAL VARS
BASE = "base"
EE_FRAME = "right_hand"
# TARGET_FRAME = "ball"
# SOURCE_FRAME = "base"
TARGET_FRAME = "base"
SOURCE_FRAME = "ball"
# X_KINECT_CALIBR = 0.
# Y_KINECT_CALIBR = -0.7
# Z_KINECT_CALIBR = 0.6
X_KINECT_CALIBR = -0.60
Y_KINECT_CALIBR = -0.60
Z_KINECT_CALIBR = 0.628
# ROLL_KINECT_CALIBR = -0.436332 # -25 deg
# PITCH_KINECT_CALIBR = 1.5708 
# YAW_KINECT_CALIBR = 0
# ROLL_KINECT_CALIBR = 0.349066 #10 deg
ROLL_KINECT_CALIBR = 0
PITCH_KINECT_CALIBR = -1.5708 
YAW_KINECT_CALIBR = 0
Z_CENTER = 0.317

class IKController( object ):
    def __init__(self):
        rospy.loginfo("Creating IK Controller class")
        self.print_help()

        # flags and vars
        self.kinect_calibrate_flag = False
        self.running_flag = False
        # self.start_throw = False
        self.start_calc = False
        self.robot = URDF.from_parameter_server()  
        self.kin = KDLKinematics(self.robot, BASE, EE_FRAME)
        self.limb_interface = intera_interface.Limb()
        self.pos_rec = [PointStamped() for i in range(2)] # array 1x5 storing x,y,z in past five frames
        self.old_pos_rec = [PointStamped() for i in range(2)]
        self.tf_listener = tf.TransformListener()
        self.tf_bc = tf.TransformBroadcaster()
        self.counter = 0 
        self.loop_counter = 0
        # self.des_point = Point()
        self.des_point = PointStamped()
        # kbhit instance
        self.kb = kbhit.KBHit()
        rospy.on_shutdown(self.kb.set_normal_term)

        # publishers and timers
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)
        self.tf_timer = rospy.Timer(rospy.Duration(0.01), self.tf_update_cb)
        self.calc_timer = rospy.Timer(rospy.Duration(0.01), self.calc_cb)
        self.starting_calc_timer = rospy.Timer(rospy.Duration(0.05), self.starting_flag_trigger_cb)
        self.final_x_total = 0
        self.final_y_total = 0

        # plotting assistant
        self.plotter = rospy.Timer(rospy.Duration(0.01), self.plotter_cb)
        self.plot_flag = False
        self.plot_x = []
        self.plot_y = []
        self.plot_z = []
        self.plot_t = []

        # self.pos_sub = rospy.Subscriber("tracked_obj/position", Point, self.obj_pos_cb)

    def plotter_cb(self, tdat):
        if self.plot_flag:
            self.running_flag = False
            self.start_calc = False
            plt.plot(self.plot_t, self.plot_x, 'ro', label='x(depth)')
            plt.plot(self.plot_t, self.plot_y, 'g.', label='y')
            plt.plot(self.plot_t, self.plot_z, 'bo', label='z')
            plt.show()
            self.plot_x = []
            self.plot_y = []
            self.plot_z = []
            self.plot_t = []
            self.plot_flag = False
            
    def roll_mat(self, mat):
        row_num = len(mat)
        for i in range(0,row_num - 1):
            mat[i] = mat[i+1]
        return

    def ball_move(self):
        # if self.counter < 10:
        #     print self.pos_rec[1]
        #     self.counter += 1
        # z_diff = abs((self.pos_rec[1])[2] - (self.pos_rec[0])[2])        
        z_diff = abs((self.pos_rec[1]).point.z - (self.pos_rec[0]).point.z)        
        # print "z_diff : ", z_diff
        if z_diff > 0.15 and z_diff < 1: 
            # self.counter = 20
            self.start_calc = True
            rospy.loginfo("Start calculation")
            self.loop_counter = 0
        # x_diff = abs((self.pos_rec[1])[0] - (self.pos_rec[0])[0]) 
        # if (x_diff > 0.2): #0.1 - 0.15
        #     # self.counter = 20
        #     self.start_calc = False
        #     rospy.loginfo("Stop calculation") 
        

    # check if the ball has been thrown yet
    def starting_flag_trigger_cb(self,tdat):
        if self.running_flag:
            # check the distance in Z to know if the bal is being thrown or not
            if not self.start_calc:
                # z_diff = abs((self.pos_rec[1])[2] - (self.pos_rec[0])[2])   
                z_diff = abs((self.pos_rec[1]).point.z - (self.pos_rec[0]).point.z) 
                # x_diff = ((self.pos_rec[1]).point.x - (self.pos_rec[0]).point.x)
                # print "test : ", (self.pos_rec[1]).header.stamp
                # dt = (self.pos_rec[1]).header.stamp - (self.pos_rec[0]).header.stamp
                # dt1 = (self.pos_rec[1]).header.stamp 
                # dt2 = (self.pos_rec[0]).header.stamp 
                # dt = dt1 - dt2
                # x_dot_diff = 1.0*x_diff/dt
                print "z_diff : ", z_diff
                if z_diff > 0.1: #5 cm
                # if z_diff > 0.2 and abs(z_diff) < 1: #0.1 - 0.15
                # if x_diff < -0.1: #0.1 - 0.15
                    # self.counter = 20
                    self.start_calc = True
                    # rospy.loginfo("Start calculation")
                    self.loop_counter = 0
                    self.final_x_total = 0
                    self.final_y_total = 0
            if self.start_calc:    
                x_diff = (self.pos_rec[1]).point.x - (self.pos_rec[0]).point.x
                if x_diff > 0.1: # 1 cm
                    # print "\n\ncheck why after first trial, it starts and stops suddenly."
                    print "pos_rec_old : ", self.pos_rec[0].point
                    print "pos_rec_new : ", self.pos_rec[1].point
                    print "x_diff : ", x_diff
                    #     # self.counter = 20
                    self.running_flag = False
                    self.start_calc = False
                    rospy.loginfo("Stop calculation")
                    print "final point : ",self.des_point.point
                    


    def calc_cb(self, tdat):
        # calculating the final position of the ball and sending tf between /base and /ball_final
        self.tf_bc.sendTransform((self.des_point.point.x, self.des_point.point.y, self.des_point.point.z), [0,0,0,1], rospy.Time.now(), "ball_final", "base")
        if self.running_flag:
            # if not self.start_calc:
            # self.ball_move()
            # wait until start_calc flag is ran and there's no repetitive pos_rec by omparing between self.old_pos_rec and self.pos_rec
            if self.start_calc and (self.old_pos_rec != self.pos_rec):        
                rospy.loginfo("\n\nCalculating....")      
                # calculate x and y fromm two points in array
                self.loop_counter += 1
                print "loop_counter", self.loop_counter
                # self.des_point = sawyer_calc.projectile_calc(self.pos_rec, Z_CENTER, 30);
                print "pos_rec[0]", self.pos_rec[0].header.stamp, " and pos_rec[1]", self.pos_rec[1].header.stamp
                # if self.pos_rec[0].header.stamp > rospy.Duration(0) and self.pos_rec[1].header.stamp > rospy.Duration(0):
                # if self.pos_rec[1].header.stamp > rospy.Duration(0):
                # if self.pos_rec[1].header.stamp > 0:
                self.des_point = sawyer_calc.projectile_calc(self.pos_rec, Z_CENTER);
                print "des_point output : ", self.des_point.point
                self.final_x_total += self.des_point.point.x
                self.des_point.point.x = self.final_x_total / self.loop_counter
                self.final_y_total += self.des_point.point.y
                self.des_point.point.y = self.final_y_total / self.loop_counter
                print "final_x_total : ", self.final_x_total
                print "final_y_total : ", self.final_y_total
                print "des_point.point.x avg : ", self.des_point.point.x
                print "des_point.point.y avg : ", self.des_point.point.y
                # print self.des_point
                # self.tf_bc.sendTransform((self.des_point.x, self.des_point.y, self.des_point.z), [0,0,0,1], rospy.Time.now(), "ball_final", "base")
                self.tf_bc.sendTransform((self.des_point.point.x, self.des_point.point.y, self.des_point.point.z), [0,0,0,1], rospy.Time.now(), "ball_final", "base")
                self.old_pos_rec = self.pos_rec
                # send x,y,z to ik controller and move

                # x_diff = (self.pos_rec[1])[0] - (self.pos_rec[0])[0]   
                # # print "x_diff : ",x_diff
                # if x_diff > 0: #0.1 - 0.15
                # #     # self.counter = 20
                #     self.start_calc = False
                #     rospy.loginfo("Stop calculation")


    def tf_update_cb(self, tdat):
        # update ball position in real time
        # Filter ball outside x = 0 - 3.0m relative to base out
        self.tf_listener.waitForTransform(TARGET_FRAME, SOURCE_FRAME, rospy.Time(), rospy.Duration(20))
        p, q = self.tf_listener.lookupTransform(TARGET_FRAME, SOURCE_FRAME, rospy.Time())
        # print "p : ", p[0]
        pos = PointStamped()
        pos.header.stamp = rospy.get_time()
        # print pos.header.stamp
        pos.point.x  = p[0]
        pos.point.y  = p[1]
        pos.point.z  = p[2]
        # print pos.point
        # print self.pos_rec[len(self.pos_rec) - 1].point
        # choose only a ball within range (x < 3m., abs(y) < 2m.) and non-repeated frame 
        if self.running_flag:
            if self.counter < 20:
                print "This must be fully filled before throwing: ", self.pos_rec
            self.counter += 1
            if pos.point.x < 3 and abs(pos.point.y) < 2 and (pos.point.x!=self.pos_rec[len(self.pos_rec) - 1].point.x) or (pos.point.y!=self.pos_rec[len(self.pos_rec) - 1].point.y):
                self.roll_mat(self.pos_rec)
                self.pos_rec[-1] = pos
                self.plot_x.append(pos.point.x)
                self.plot_y.append(pos.point.y)
                self.plot_z.append(pos.point.z)
                self.plot_t.append(pos.header.stamp)
                # print self.pos_rec[len(self.pos_rec) - 1].point
                # print "\npos old x:", self.pos_rec[0].point.x, " y: ", self.pos_rec[0].point.y, "z: ", self.pos_rec[0].point.z
                # print "\npos new x:", self.pos_rec[1].point.x, " y: ", self.pos_rec[1].point.y, "z: ", self.pos_rec[1].point.z

    def keycb(self, tdat):
        # check if there was a key pressed, and if so, check it's value and
        # toggle flag:
        if self.kb.kbhit():
            c = self.kb.getch()
            if ord(c) == 27:
                rospy.signal_shutdown("Escape pressed!")
            else:
                print c
            if c == 's':
                rospy.loginfo("You pressed 's', Program starts. Sawyer is waiting for the ball to be thrown.")
                self.running_flag = not self.running_flag
                
            elif c == 'c':
                rospy.loginfo("You pressed 'c', Start calibration\nrunning_flag = False")
                self.running_flag = False
                self.kinect_pos_calib()
            elif c == 'h':
                rospy.loginfo("You pressed 'h', Go to home position\nrunning_flag = False")
                self.running_flag = False
                self.home_pos()
            elif c == 'p':
                rospy.loginfo("You pressed 'h', Plotting")
                self.plot_flag = True
            else:
                self.print_help()
            self.kb.flush()
        return

    def kinect_pos_calib(self):
        # input x,y,z and x,y,z
        p = [X_KINECT_CALIBR, Y_KINECT_CALIBR, Z_KINECT_CALIBR]
        G = tr.euler_matrix(ROLL_KINECT_CALIBR, PITCH_KINECT_CALIBR, YAW_KINECT_CALIBR)
        for i in range(3):
            G[i][3] = p[i]
        q_seed = self.kin.random_joint_angles()
        j_names = self.kin.get_joint_names()
        q = self.kin.inverse(G, q_seed)
        while (q is None):
            q_seed = self.kin.random_joint_angles()
            q = self.kin.inverse(G, q_seed)
        j_req = dict(zip(j_names, q))
        self.limb_interface.move_to_joint_positions(j_req)

    def home_pos(self):
        self.limb_interface.move_to_joint_positions({'head_pan':0.379125, 'right_j0':-0.020025390625 , 'right_j1':0.8227529296875, 'right_j2':-2.0955126953125, 'right_j3':2.172509765625, 'right_j4':0.7021171875, 'right_j5' : -1.5003603515625, 'right_j6' : -2.204990234375})

    def print_help(self):
        help_string = \
        """
        's'   ~  Start the program
        'c'   ~  Calibration the kinect position
        'h'   ~  Move to home position
        'p'   ~  Plot the trajectory of each Cartesian coordinate
        'ESC' ~  Quit
        """
        print help_string
        return



def main():
    rospy.init_node('ik_controller_kinematics', log_level=rospy.INFO)

    try:
        controller = IKController()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()





