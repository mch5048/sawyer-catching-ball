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
import cv2.cv
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
import copy


#################
# LOCAL IMPORTS #
#################
import kbhit
import sawyer_catch_ball_calc as sawyer_calc


#GLOBAL VARIABLES
GREEN_TB_HIGHS = [179, 255, 255, 179, 255, 255, 15,15, 255, 255, 255, 255, 255, 255]
# GREEN_TB_HIGHS = [179, 255, 255, 179, 255, 255, 15,15, 255, 255, 255, 255, 255, 255,
#                   255, 255]
# GREEN_TB_DEFAULTS = [27, 105, 110, 66, 255, 255,  2, 8, 0, 133, 0, 201, 255, 255]
# GREEN_TB_DEFAULTS = [27, 105, 110, 66, 255, 255,  2, 8, 0, 100, 0, 148, 255, 255]
# GREEN_TB_DEFAULTS = [23, 105, 110, 66, 255, 255,  2, 8, 0, 100, 0, 148, 255, 255] ## Willens wing (The best one)
GREEN_TB_DEFAULTS = [25, 83, 113, 86, 255, 255,  2, 8, 0, 51, 86, 255, 255, 255] ## Willens wing (In front of RJ)
# GREEN_TB_DEFAULTS = [23, 0, 83, 64, 255, 255,  2, 8, 0, 0, 0, 255, 255, 255] ## Willens wing (3-5pm)
# GREEN_TB_DEFAULTS = [0, 0, 148, 88, 189, 255, 2, 8, 111, 0, 0, 255, 255, 255,
#                      0, 0] ## Willens wing (Cloudy) (Not yet finished)
# GREEN_TB_DEFAULTS = [0, 0, 255, 66, 0, 255, 2, 8, 111, 0, 0, 255, 255, 255,
#                      255, 255] ## Willens wing (With Orange light (around 3-5 pm))
WITHIN_RAN_MIN = 0
WITHIN_RAN_MAX = 3
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
        self.ball_radius = 0
        self.p_ball_to_cam = []
        self.bridge = CvBridge()
        self.cp = 0
        self.tb_val = self.window_with_trackbars('image_red', GREEN_TB_DEFAULTS, GREEN_TB_HIGHS)

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
        self.msg_filt_rgb_img_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        # self.msg_filt_depth_img_sub = message_filters.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image)
        self.msg_filt_depth_img_sub = message_filters.Subscriber("/camera/depth_registered/image_raw", Image)
        self.t_sync = message_filters.ApproximateTimeSynchronizer([self.msg_filt_rgb_img_sub, self.msg_filt_depth_img_sub], 1, slop = 0.05) #slop in secs
        self.t_sync.registerCallback(self.time_sync_img_cb)

        # self.last_d_img = np.zeros((240,320), dtype=np.float32)
        # self.dimg_sub = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, self.d_img_cb)

        # publishers and timers
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)
        self.start_time = 0
        self.tf_br = tf.TransformBroadcaster()

    def d_img_cb(self, d_img):
        vals = self.tb_val.get_vals()
        # b = GREEN_TB_DEFAULTS[8:14]
        # v = GREEN_TB_DEFAULTS[0:6]
        # e = GREEN_TB_DEFAULTS[6:8]
        # b = vals[8:14]
        # v = vals[0:6]
        e = vals[6:8]
        d_img = self.bridge.imgmsg_to_cv2(d_img)
        # cv2.imshow('d_img', d_img)
        d_img_sh = self.last_d_img - d_img
        # d_img_sh = cv2.absdiff(d_img, self.last_d_img)
        self.last_d_img = d_img
        # cv2.imshow('d_img_subt', d_img_sh)
        # cv2.waitKey(1)
        img_blur = cv2.GaussianBlur(d_img_sh, (5,5), 1)
        # cv2.imshow('d_img_blur', img_blur)
        # cv2.waitKey(1)
        d_img_inR = cv2.inRange(img_blur, 2.5, 5) #mask black portion out
        # cv2.imshow('d_img_inR', d_img_inR)
        # cv2.waitKey(1)
        # im_b_to_w_color = cv2.bitwise_and(, img_raw, mask = im_b_to_w)
        # # cv2.imshow('image_b_to_w_color', im_b_to_w_color)
        # # cv2.waitKey(1) 
        # img_blur = cv2.GaussianBlur(im_b_to_w_color, (9,9), 1)
        kernel_erode = np.ones((e[0],e[0]),np.uint8)
        kernel_dilate = np.ones((e[1],e[1]),np.uint8)
        d_img_er = cv2.erode(d_img_inR, kernel_erode, iterations=2)
        d_img_dl = cv2.dilate(d_img_er, kernel_dilate, iterations=2)
        cv2.imshow('d_img_dl', d_img_dl)
        cv2.waitKey(1)
    #     kernel_dilate = np.ones((e[1],e[1]),np.uint8)
    #     im_total = cv2.erode(im_total, kernel_erode, iterations=2)
    #     im_total = cv2.dilate(im_total, kernel_dilate, iterations=2)




    def time_sync_img_cb(self, rgb_img, depth_img):
        # rgb image process
        # tstart = time.time()
        # if self.color_track_3to5pm(rgb_img) is False:
        # if self.color_track(rgb_img) is False:
        if self.color_track_front_RJ(rgb_img) is False:
        # if self.color_track_edge_willen_wing(rgb_img) is False:
            print "color detection error"
            return
        # print "color_track: ", (time.time() - tstart)*1000 ,"ms"
        # tstart = time.time()
        self.depth_cb(depth_img)
        # print "depth_cb: ", (time.time() - tstart)*1000, "ms"

    def get_and_set_params(self):
        self.use_kb = rospy.get_param("kb",False)

    def update_model_cb(self, info):
        self.ph_model.fromCameraInfo(info)
        self.cam_info_sub.unregister()


    def depth_cb(self, depth_img):
        tfirst = time.time()
        x = self.tracked_pixel.x
        y = self.tracked_pixel.y        
        d_img = self.bridge.imgmsg_to_cv2(depth_img)
        # d_img = cv2.resize(d_img, (0,0), fx=2, fy=2) ###RESIZE if it's not the same size
        # print "d_img size = ", d_img.shape
        # depth = d_img[y][x]
        # print "depth: ", depth, "\r\n"
        # tstart = time.time()
        # for i in range(-self.ball_radius,self.ball_radius):   ##0.00113sec
        #     for j in range(-self.ball_radius,self.ball_radius):
        #         if ((x+i) < d_img.shape[1]) and \
        #            ((y+j) < d_img.shape[0]) and \
        #            ((x+i) > 0) and \
        #            ((y+j) > 0):
        #             x_rand = x + i
        #             y_rand = y + j
        #             depth_rand_array.append(d_img[y_rand][x_rand])
        rad = int(0.5*self.ball_radius)
        xlim = np.clip([x-rad, x+rad], 0, d_img.shape[1])
        ylim = np.clip([y-rad, y+rad], 0, d_img.shape[0])
        depth_rand_array = d_img[ylim[0]:ylim[1], xlim[0]:xlim[1]].ravel()
        # cv2.imshow('image_rgb_enclosed', rgb_img_test[ylim[0]:ylim[1], xlim[0]:xlim[1]])
        # cv2.waitKey(1)   
        # print "depth_rand_array: ", depth_rand_array ,"\r\n****************************************\r\n"
        # depth_rand_array = np.array(depth_rand_array)
        # print "=================================================="
        # print "Double for loop time = ",(time.time() - tstart)*1000, "data length = ", len(depth_rand_array)
        tstart = time.time()
        # depth_rand_array = sawyer_calc.filter_nan(depth_rand_array) ## 0.002-0.0047sec
        depth_rand_array = sawyer_calc.filter_nan_1(depth_rand_array) ## 0.002-0.0047sec
        # print "filter nan time = ",(time.time() - tstart)*1000, "data length = ", len(depth_rand_array)
        tstart = time.time()
        # depth_rand_array = sawyer_calc.within_range_filter(depth_rand_array, WITHIN_RAN_MIN, WITHIN_RAN_MAX) ##0.006-0.01
        # depth_rand_array = sawyer_calc.within_1(depth_rand_array, WITHIN_RAN_MIN, WITHIN_RAN_MAX) ##0.006-0.01
        # print "within range time = ",(time.time() - tstart)*1000, "data length = ", len(depth_rand_array)
        tstart = time.time()
        # depth_rand_array = sawyer_calc.reject_outliers(depth_rand_array, OUTLIER_FILT_NUM) # reject outliers that seems to return very far depth
        # depth_rand_array = sawyer_calc.reject_outliers_1(depth_rand_array, OUTLIER_FILT_NUM) # reject outliers that seems to return very far depth
        depth_rand_array = sawyer_calc.reject_outliers_nate(depth_rand_array) # reject outliers that seems to return very far depth
        # print "reject outliers time = ",(time.time() - tstart)*1000, "data length = ", len(depth_rand_array)
        tstart = time.time()
        # depth = sawyer_calc.mean(depth_rand_array)
        # print "depth_rand_array: ", 
        depth = np.mean(depth_rand_array)
        # print "depth: ", depth, "\r\n"
        # print "calculate mean time = ",(time.time() - tstart)*1000
        # print "TOTAL TIME = ",(time.time() - tfirst)*1000, "ms"
        # print "=================================================="
        # print "\r\n"

        # if (time.time() - tfirst)*1000 > 15: print "TOTAL TIME = ",(time.time() - tfirst)*1000, " ms\r\n"
        
        norm_v = self.ph_model.projectPixelTo3dRay((x,y))

        scale = depth*1.0 / norm_v[2] 
        pos = Point()
        pos_in_space = [z * scale for z in norm_v]
        pos.x = pos_in_space[0]
        pos.y = pos_in_space[1]
        pos.z = pos_in_space[2]

        if not np.isnan(depth):
            # only send tf out if depth is not equal to 0 which is the case of null set being input into filters
            if scale != 0:
                t_stamp = depth_img.header.stamp.secs + (depth_img.header.stamp.nsecs)*(10**(-9))
                self.p_ball_to_cam = np.array([pos.z,-pos.x,-pos.y, t_stamp])

    def color_track_3to5pm(self, ros_img):
        # For when the orange light of Willens Wing is turned on
        img_raw = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")        
        gray_image = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('gray_image', gray_image)
        # cv2.waitKey(1)

        vals = self.tb_val.get_vals()
        b = vals[8:14]
        v = vals[0:6]
        e = vals[6:8]
        gs = vals[14:16]
        im_gray_inrange = cv2.inRange(gray_image, gs[0], gs[1])
        # cv2.imshow('im_gray_inrange', im_gray_inrange)
        # cv2.waitKey(1)        

        kernel_dilate = np.ones((4,4),np.uint8)
        im_dilate = cv2.dilate(im_gray_inrange, kernel_dilate, iterations=2)  
        # cv2.imshow('gray_dilate_image', im_dilate)
        # cv2.waitKey(1)

        im_rgb = cv2.bitwise_and(img_raw, img_raw, mask = im_dilate)
        # cv2.imshow('im_gray_inrange_rgb', im_rgb)
        # cv2.waitKey(1)   

        ###### HSV RANGE
        img_hsv = cv2.cvtColor(im_rgb, cv2.COLOR_BGR2HSV)  
        low_vals_lo = np.array([v[0], v[1], v[2]])
        high_vals_lo = np.array([v[3], v[4], v[5]])
        im_hsv_ir = cv2.inRange(img_hsv, low_vals_lo, high_vals_lo)
        cv2.imshow('image_hsv_inrange', im_hsv_ir)
        cv2.waitKey(1) 
        ######

        ###### RGB RANGE 
        # img_bgr = cv2.cvtColor(im_hsv_ir, cv2.COLOR_HSV2BGR) 
        # bgr_lo = np.array([b[0], b[1], b[2]])
        # bgr_hi = np.array([b[3], b[4], b[5]])
        # im_rgb_inrange = cv2.inRange(img_bgr, bgr_lo, bgr_hi) #mask black portion out
        # cv2.imshow('image_b_inrange', im_rgb_inrange)
        # cv2.waitKey(1) 
        ######

        ###### ERODE
        kernel_erode = np.ones((e[0],e[0]),np.uint8)
        im_erode = cv2.erode(im_hsv_ir, kernel_erode, iterations=2)
        cv2.imshow('im_erode', im_erode)
        cv2.waitKey(1)
        ######


    def color_track_edge_willen_wing(self, ros_img):
        # cv_image = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        img_raw = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        # print img_raw.shape
        # img_raw = cv2.resize(img_raw, (0,0), fx=0.5, fy=0.5) ###RESIZE if it's not the same size
        # print img_raw.shape
        # cv2.rectangle(img_raw, (0,0), (40,240), (0,0,0), -1)
        # print img_raw.shape
        vals = self.tb_val.get_vals()
        b = vals[8:14]
        v = vals[0:6]
        e = vals[6:8]
        gs = vals[14:16]
        black_lo = np.array([b[0], b[1], b[2]])
        black_hi = np.array([b[3], b[4], b[5]])

        # im_b_to_w = cv2.inRange(img_raw, black_lo, black_hi) #mask black portion out
        # cv2.imshow('image_b_to_w', im_b_to_w)
        # cv2.waitKey(1) 

        img_hsv = cv2.cvtColor(img_raw, cv2.COLOR_BGR2HSV)  
        # cv2.imshow('image_hsv', img_hsv)
        # cv2.waitKey(1) 
        # low and high band for detecting red color in hsv which spans at each end of h-band
        low_vals_lo = np.array([v[0], v[1], v[2]])
        high_vals_lo = np.array([v[3], v[4], v[5]])
        im_hsv_ir = cv2.inRange(img_hsv, low_vals_lo, high_vals_lo)
        cv2.imshow('image_hsv', im_hsv_ir)
        cv2.waitKey(1) 
        # cv2.imshow('image_hsv_ir', im_hsv_ir)
        # cv2.waitKey(1)        

        ##### Gaussian Blur
        img_hsv_ir_blur = cv2.GaussianBlur(im_hsv_ir, (9,9), 1)


        ##### BGR Range
        im_hsv_filt = cv2.bitwise_and(img_raw, img_raw, mask = img_hsv_ir_blur)        
        black_lo = np.array([b[0], b[1], b[2]])
        black_hi = np.array([b[3], b[4], b[5]])  
        im_bgr_ir = cv2.inRange(im_hsv_filt, black_lo, black_hi) #mask black portion out
        cv2.imshow('image_bgr_ir', im_bgr_ir)
        cv2.waitKey(1)      
        #####

        kernel_erode = np.ones((e[0],e[0]),np.uint8)
        kernel_dilate = np.ones((e[1],e[1]),np.uint8)
        im_erode = cv2.erode(im_bgr_ir, kernel_erode, iterations=2)
        # im_erode_rgb = cv2.bitwise_and(img_raw, img_raw, mask = im_erode)
        # cv2.imshow('image_erode', im_erode_rgb)
        # cv2.waitKey(1)
        im_dilate = cv2.dilate(im_erode, kernel_dilate, iterations=2)  
        # cv2.imshow('image_dilate', im_dilate)
        # cv2.waitKey(1)   
        # img_detect_color, coords = self.specific_color_filter(cv_image)
        # imContours = cv2.findContours(im_total.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        imContours = cv2.findContours(im_dilate, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        tstart = time.time()
        # only proceed if at least one contour was found
        if len(imContours) > 0:
            # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
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
        tracked_color_im = cv2.bitwise_and(img_raw, img_raw, mask = im_dilate)
        try:
            coords = list(center)
        except TypeError:
            print "lost points: ", self.cp
            self.cp += 1
            return False
        cv2.imshow('image_final', tracked_color_im)
        cv2.waitKey(1)
        self.tracked_pixel.x = coords[0]
        self.tracked_pixel.y = coords[1]
        return True


    def color_track_front_RJ(self, ros_img):
        # cv_image = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        img_raw = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        # print img_raw.shape
        # img_raw = cv2.resize(img_raw, (0,0), fx=0.5, fy=0.5) ###RESIZE if it's not the same size
        # print img_raw.shape
        # cv2.rectangle(img_raw, (0,0), (40,240), (0,0,0), -1)
        # print img_raw.shape
        vals = self.tb_val.get_vals()
        b = vals[8:14]
        v = vals[0:6]
        e = vals[6:8]
        gs = vals[14:16]
        black_lo = np.array([b[0], b[1], b[2]])
        black_hi = np.array([b[3], b[4], b[5]])

        im_b_to_w = cv2.inRange(img_raw, black_lo, black_hi) #mask black portion out
        cv2.imshow('image_b_to_w', im_b_to_w)
        cv2.waitKey(1) 
        im_b_to_w_color = cv2.bitwise_and(img_raw, img_raw, mask = im_b_to_w)
        # cv2.imshow('image_b_to_w_color', im_b_to_w_color)
        # cv2.waitKey(1) 
        # img_blur = cv2.GaussianBlur(im_b_to_w_color, (9,9), 1)
        # cv2.imshow('image_blur', img_blur)
        # cv2.waitKey(1) 
        img_hsv = cv2.cvtColor(im_b_to_w_color, cv2.COLOR_BGR2HSV)  
        # cv2.imshow('image_hsv', img_hsv)
        # cv2.waitKey(1) 
        # low and high band for detecting red color in hsv which spans at each end of h-band
        low_vals_lo = np.array([v[0], v[1], v[2]])
        high_vals_lo = np.array([v[3], v[4], v[5]])
        im_hsv_ir = cv2.inRange(img_hsv, low_vals_lo, high_vals_lo)
        cv2.imshow('image_hsv_ir', im_hsv_ir)
        cv2.waitKey(1) 
        # cv2.imshow('image_hsv', im_hsv_ir)
        # cv2.waitKey(1) 
        # cv2.imshow('image_hsv_ir', im_hsv_ir)
        # cv2.waitKey(1)        
        kernel_erode = np.ones((e[0],e[0]),np.uint8)
        kernel_dilate = np.ones((e[1],e[1]),np.uint8)
        im_erode = cv2.erode(im_hsv_ir, kernel_erode, iterations=2)
        im_erode_rgb = cv2.bitwise_and(img_raw, img_raw, mask = im_erode)
        cv2.imshow('image_erode', im_erode_rgb)
        cv2.waitKey(1)
        im_dilate = cv2.dilate(im_erode, kernel_dilate, iterations=2)  
        # cv2.imshow('image_dilate', im_dilate)
        # cv2.waitKey(1)   
        # img_detect_color, coords = self.specific_color_filter(cv_image)
        # imContours = cv2.findContours(im_total.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        imContours = cv2.findContours(im_dilate, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        tstart = time.time()
        # only proceed if at least one contour was found
        if len(imContours) > 0:
            # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
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
        tracked_color_im = cv2.bitwise_and(img_raw, img_raw, mask = im_dilate)
        try:
            coords = list(center)
        except TypeError:
            print "lost points: ", self.cp
            self.cp += 1
            return False
        cv2.imshow('image_final', tracked_color_im)
        cv2.waitKey(1)
        self.tracked_pixel.x = coords[0]
        self.tracked_pixel.y = coords[1]
        return True

    def color_track(self, ros_img):
        # cv_image = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        img_raw = self.bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        # print img_raw.shape
        # img_raw = cv2.resize(img_raw, (0,0), fx=0.5, fy=0.5) ###RESIZE if it's not the same size
        # print img_raw.shape
        # cv2.rectangle(img_raw, (0,0), (40,240), (0,0,0), -1)
        # print img_raw.shape
        vals = self.tb_val.get_vals()
        b = vals[8:14]
        v = vals[0:6]
        e = vals[6:8]
        gs = vals[14:16]
        black_lo = np.array([b[0], b[1], b[2]])
        black_hi = np.array([b[3], b[4], b[5]])

        im_b_to_w = cv2.inRange(img_raw, black_lo, black_hi) #mask black portion out
        cv2.imshow('image_b_to_w', im_b_to_w)
        cv2.waitKey(1) 
        im_b_to_w_color = cv2.bitwise_and(img_raw, img_raw, mask = im_b_to_w)
        # cv2.imshow('image_b_to_w_color', im_b_to_w_color)
        # cv2.waitKey(1) 
        # img_blur = cv2.GaussianBlur(im_b_to_w_color, (9,9), 1)
        # cv2.imshow('image_blur', img_blur)
        # cv2.waitKey(1) 
        img_hsv = cv2.cvtColor(im_b_to_w_color, cv2.COLOR_BGR2HSV)  
        # cv2.imshow('image_hsv', img_hsv)
        # cv2.waitKey(1) 
        # low and high band for detecting red color in hsv which spans at each end of h-band
        low_vals_lo = np.array([v[0], v[1], v[2]])
        high_vals_lo = np.array([v[3], v[4], v[5]])
        im_hsv_ir = cv2.inRange(img_hsv, low_vals_lo, high_vals_lo)
        cv2.imshow('image_hsv', im_hsv_ir)
        cv2.waitKey(1) 
        # cv2.imshow('image_hsv_ir', im_hsv_ir)
        # cv2.waitKey(1)        
        kernel_erode = np.ones((e[0],e[0]),np.uint8)
        kernel_dilate = np.ones((e[1],e[1]),np.uint8)
        im_erode = cv2.erode(im_hsv_ir, kernel_erode, iterations=2)
        # im_erode_rgb = cv2.bitwise_and(img_raw, img_raw, mask = im_erode)
        # cv2.imshow('image_erode', im_erode_rgb)
        # cv2.waitKey(1)
        im_dilate = cv2.dilate(im_erode, kernel_dilate, iterations=2)  
        # cv2.imshow('image_dilate', im_dilate)
        # cv2.waitKey(1)   
        # img_detect_color, coords = self.specific_color_filter(cv_image)
        # imContours = cv2.findContours(im_total.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        imContours = cv2.findContours(im_dilate, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        tstart = time.time()
        # only proceed if at least one contour was found
        if len(imContours) > 0:
            # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
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
        tracked_color_im = cv2.bitwise_and(img_raw, img_raw, mask = im_dilate)
        try:
            coords = list(center)
        except TypeError:
            print "lost points: ", self.cp
            self.cp += 1
            return False
        cv2.imshow('image_final', tracked_color_im)
        cv2.waitKey(1)
        self.tracked_pixel.x = coords[0]
        self.tracked_pixel.y = coords[1]
        return True




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




