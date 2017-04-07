#!/usr/bin/env python
"""
Chainatee Tanakulrungson
Mar 2017

This file subscribes to the reference poses, and then 

SUBSCRIPTIONS:


PUBLISHERS:


SERVICES:


PARAMS:

"""

###############
# ROS IMPORTS #
###############
import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import cv2
import tf
import rosbag
import message_filters

##################
# PYTHON IMPORTS #
##################
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os
import time
import csv


#################
# LOCAL IMPORTS #
#################
import kbhit
import sawyer_catch_ball_calc as sawyer_calc


#GLOBAL VARIABLES
GREEN_TB_HIGHS = [179, 255, 255, 179, 255, 255, 15,15, 255, 255, 255, 255, 255, 255]
GREEN_TB_DEFAULTS = [27, 105, 110, 66, 255, 255,  2, 8, 0, 133, 0, 201, 255, 255]
WITHIN_RAN_MIN = 0
WITHIN_RAN_MAX = 3.5
OUTLIER_FILT_NUM = 0.25
AVG_PIX_RANGE = 30

class Obj3DDetector(object):


    class window_with_trackbars(object):
        def __init__(self, window_name, tb_defaults=[], tb_highs=[]):
            #initialize variables and setup
            self.window_name = window_name
            self.tb_defaults = tb_defaults
            self.tb_highs = tb_highs
            self.N = len(tb_defaults)
            #create named window 
            cv2.namedWindow(self.window_name, cv2.CV_WINDOW_AUTOSIZE)
            #add trackbars
            for i in range(self.N):
                cv2.createTrackbar('val'+str(i), self.window_name, self.tb_defaults[i], self.tb_highs[i], self.nothing)
                # cv2.createTrackbar('val'+str(i), self.window_name, self.tb_defaults[i], self.tb_highs[i])

        def nothing(self, x):
            # print "VALUE OF THE TRACKBAR IS: ", x
            pass
                
        def get_vals(self):
            vals = []
            for i in range(self.N):
                val = cv2.getTrackbarPos('val'+str(i), self.window_name)
                vals.append(val)
            return vals
        

    def __init__(self):
        rospy.loginfo("Creating Obj3DDetector class")
        
        # flags and vars
        self.ph_model = image_geometry.PinholeCameraModel()
        self.tracked_pixel = Point()
        self.use_kb = False
        #self.trackbar = self.window_with_trackbars('image_tb', GREEN_TB_DEFAULTS, GREEN_TB_HIGHS)
        self.ball_radius = 0

        # Get and set params
        self.get_and_set_params()

        # kbhit instance
        self.kb = kbhit.KBHit()
        if self.use_kb:
            self.print_help()
            rospy.on_shutdown(self.kb.set_normal_term)
            self.imwritecounter = 0

        # subscribers
        self.cam_info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.update_model_cb)
        self.msg_filt_rgb_img_sub = message_filters.Subscriber("/camera/rgb/image_color", Image)
        self.msg_filt_depth_img_sub = message_filters.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image)
        self.t_sync = message_filters.ApproximateTimeSynchronizer([self.msg_filt_rgb_img_sub, self.msg_filt_depth_img_sub], 30, slop = 0.1)
        self.t_sync.registerCallback(self.time_sync_img_cb)

        # publishers and timers
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)
        self.start_time = 0
        self.tf_br = tf.TransformBroadcaster()
        # publishers for rosbag
        # self.rgb_img_tsync_pub = rospy.Publisher('time_sync_rgb_img', Image, queue_size = 10)
        # self.depth_img_tsync_pub = rospy.Publisher('time_sync_depth_img', Image, queue_size = 10)
        # self.ball_pixel_tsync_pub = rospy.Publisher('time_sync_ball_pixel_img', Point, queue_size = 10)



    def time_sync_img_cb(self, rgb_img, depth_img):
        # rgb image process
        self.image_cb(rgb_img)
        self.depth_cb(depth_img)
        # self.rgb_img_tsync_pub.publish(self.bag_rgb_im)
        # self.depth_img_tsync_pub.publish(depth_img)



    def get_and_set_params(self):
        self.use_kb = rospy.get_param("kb",False)



    def update_model_cb(self, info):
        self.ph_model.fromCameraInfo(info)


    def depth_cb(self, depth_img):
        x = self.tracked_pixel.x
        y = self.tracked_pixel.y
        
        bridge = CvBridge()
        d_img = bridge.imgmsg_to_cv2(depth_img)
        d_img = cv2.resize(d_img, (0,0), fx=2, fy=2)


            # rec_coord = Point()
            # rec_coord.x = x
            # rec_coord.y = y
            # rec_coord.z = self.ball_radius
            # self.ball_pixel_tsync_pub.publish(rec_coord)
        depth = d_img[y][x]
        depth_rand_array = []
        try:
            for i in range(-self.ball_radius,self.ball_radius):   ##0.00113sec
                for j in range(-self.ball_radius,self.ball_radius):
                    x_rand = x + i
                    y_rand = y + j
                    depth_rand_array.append(d_img[y_rand][x_rand])
            depth_rand_array = sawyer_calc.filter_nan(depth_rand_array) ## 0.002-0.0047sec
        # time_st = rospy.get_time()
        # depth_rand_array = sawyer_calc.within_range_filter(depth_rand_array, WITHIN_RAN_MIN, WITHIN_RAN_MAX)
            ##0.006-0.01
            depth_rand_array = sawyer_calc.reject_outliers(depth_rand_array, OUTLIER_FILT_NUM) # reject outliers that seems to return very far depth
            # time_try = rospy.get_time()-time_st
            # print time_try
            depth = sawyer_calc.mean(depth_rand_array)
 
            norm_v = self.ph_model.projectPixelTo3dRay((x,y))

            scale = depth*1.0 / norm_v[2]
            pos = Point()
            pos_in_space = [z * scale for z in norm_v]
            pos.x = pos_in_space[0]
            pos.y = pos_in_space[1]
            pos.z = pos_in_space[2]
            if not np.isnan(depth):
                #print 'z: ', pos.z, ' x: ', pos.x,' y: ', pos.y
                # only send tf out if depth is not equal to 0 which is the case of null set being input into filters
                if scale != 0:
                    self.tf_br.sendTransform((pos.z,-pos.x,-pos.y), [0,0,0,1], rospy.Time.now(), "ball", "camera_link")
        except IndexError:
            pass

    def specific_color_filter(self, img_hsv, img_raw):


        # change black color to white color since green detection always detect black color too
        try:
            b = self.trackbar.get_vals()[8:14]
            v = self.trackbar.get_vals()[0:6]
            e = self.trackbar.get_vals()[6:8]
        except AttributeError:
            b = GREEN_TB_DEFAULTS[8:14]
            v = GREEN_TB_DEFAULTS[0:6]
            e = GREEN_TB_DEFAULTS[6:8]
        black_lo = np.array([b[0], b[1], b[2]])
        black_hi = np.array([b[3], b[4], b[5]])
        im_b_to_w = cv2.inRange(img_hsv, black_lo, black_hi) #th mask that cut the black portion out
        im_b_to_w = cv2.inRange(img_hsv, black_lo, black_hi) #th mask that cut the black portion out
        # GREEN_TB_DEFAULTS = [27, 105, 110, 66, 255, 255,  2, 8, 0, 133, 0, 201, 255, 255]
        # im_b_to_w = cv2.bitwise_not(im_b_to_w) # the mask that cut black portion out

        b_to_w_color_im = cv2.bitwise_and(img_raw, img_raw, mask = im_b_to_w)
        # img_blur = cv2.GaussianBlur(b_to_w_color_im, (9,9), 1)
        img_hsv = cv2.cvtColor(b_to_w_color_im, cv2.COLOR_BGR2HSV)   


        # v = self.trackbar.get_vals()[0:12]
        # v = self.trackbar.get_vals()[0:6]

            
        # v = GREEN_TB_DEFAULTS[0:12]
        # low and high band for detecting red color in hsv which spans at each end of h-band
        low_vals_lo = np.array([v[0], v[1], v[2]])
        high_vals_lo = np.array([v[3], v[4], v[5]])
        # low_vals_hi = np.array([v[6], v[7], v[8]])
        # high_vals_hi = np.array([v[9], v[10], v[11]])
        im_lo = cv2.inRange(img_hsv, low_vals_lo, high_vals_lo)
        # im_hi = cv2.inRange(img_hsv, low_vals_hi, high_vals_hi)
        # im_total = im_lo + im_hi
        im_total = im_lo
        # erode noisy details and dilate those left
        # e = self.trackbar.get_vals()[12:14]
        # e = GREEN_TB_DEFAULTS[12:14]
        # e = self.trackbar.get_vals()[12:14]
        # e = self.trackbar.get_vals()[6:8]
        # kernel_erode = np.ones((4,4),np.uint8)
        # kernel_dilate = np.ones((7,7),np.uint8)
        kernel_erode = np.ones((e[0],e[0]),np.uint8)
        kernel_dilate = np.ones((e[1],e[1]),np.uint8)
        im_total = cv2.erode(im_total, kernel_erode, iterations=2)
        im_total = cv2.dilate(im_total, kernel_dilate, iterations=2)
        imContours = cv2.findContours(im_total.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        # only proceed if at least one contour was found
        if len(imContours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(imContours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(img_raw, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(img_raw, center, 5, (0, 0, 255), -1)
                self.ball_radius = int(radius)
        # red.appendleft(center)
        # n = 2
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(n,n))
        # red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
        # red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)
        # red = cv2.bitwise_and(img_raw, img_raw)
        tracked_color_im = cv2.bitwise_and(img_raw, img_raw, mask = im_total)
        # return tracked_color_im, list(center)
        try:
            return tracked_color_im, list(center)
        except TypeError:
            # print("Warning : detect_color_full_system.py : line 159 : No object found!! ")
            pass

    def image_cb(self, ros_img):
        #call the function from CvBridge
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        # img_blur = cv2.GaussianBlur(cv_image, (9,9), 1)
        # img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)         
        # img_red, coords = self.specific_color_filter(img_hsv, cv_image)
        try:
            img_detect_color, coords = self.specific_color_filter(cv_image, cv_image)
            # self.bag_rgb_im = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            cv2.imshow('image_raw', cv_image)
            cv2.waitKey(1)
            self.tracked_pixel.x = coords[0]
            self.tracked_pixel.y = coords[1] 
        except TypeError, UnboundLocalError:
            # print("Warning : detect_color_full_system.py : line 172 : No object found!! ")
            pass

    def keycb(self, tdat):
        # check if there was a key pressed, and if so, check it's value and
        # toggle flag:
        if self.kb.kbhit() and self.use_kb:
            c = self.kb.getch()
            if ord(c) == 27:
                rospy.signal_shutdown("Escape pressed!")
            else:
                print c
            if c == 'e':
                rospy.loginfo("You pressed 'e', Stop plotting")
                self.plot_flag = True
            elif c == 'a':
                rospy.loginfo("You pressed 'a', Plotting d_img intensity plot")
                self.rec_d_img_plot_flag = True
            elif c == 'z':
                rospy.loginfo("You pressed 'z', Stop plotting d_img intensity plot")
                self.rec_d_img_plot_flag = True
            else:
                self.print_help()
            self.kb.flush()
        return

    def print_help(self):
        help_string = \
        """
        'e'   ~  Stop and plot
        ##### D_IMG plot #####
        'a'   ~  Start the d_img plot
        'z'   ~  Stop and plot d_img
        'ESC' ~  Quit
        """
        print help_string
        return


def main():
    rospy.init_node('ball_3D_detector', log_level=rospy.INFO)

    try:
        balldetect = Obj3DDetector()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()




