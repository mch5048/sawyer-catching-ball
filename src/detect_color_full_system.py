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
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import tf

##################
# PYTHON IMPORTS #
##################
import numpy as np


#################
# LOCAL IMPORTS #
#################
import kbhit
import sawyer_catch_ball_calc as sawyer_calc


#GLOBAL VARIABLES
GREEN_TB_HIGHS = [179, 255, 255, 179, 255, 255, 179, 255, 255, 179, 255, 255]
GREEN_TB_DEFAULTS = [27, 105, 110, 66, 255, 255, 0, 0, 0, 0, 0, 0]

class Obj3DDetector(object):

    def __init__(self, use_kb = False):
        rospy.loginfo("Creating Obj3DDetector class")
        
        # flags and vars
        self.start_flag = False
        self.plot_flag = False
        self.ph_model = image_geometry.PinholeCameraModel()
        self.tracked_pixel = Point()

        # kbhit instance
        self.kb = kbhit.KBHit()
        if use_kb:
            self.print_help()
            # self.kb = kbhit.KBHit()
            rospy.on_shutdown(self.kb.set_normal_term)

        # subscribers
        self.cam_info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.update_model_cb)
        self.rgb_im_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_cb)
        self.image_rect_sub = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, self.depth_cb)

        # publishers and timers
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)
        self.tf_br = tf.TransformBroadcaster()

        # self.actual_js_sub = rospy.Subscriber("robot/joint_states", JointState, self.actual_js_cb)
        # self.js_pub = rospy.Publisher("joint_states", JointState, queue_size=3)

    def update_model_cb(self, info):
        self.ph_model.fromCameraInfo(info)


    def depth_cb(self, depth_img):
        x = self.tracked_pixel.x
        y = self.tracked_pixel.y
        bridge = CvBridge()
        d_img = bridge.imgmsg_to_cv2(depth_img)
        d_img = cv2.resize(d_img, (0,0), fx=2, fy=2)
        depth = d_img[y][x]
        if np.isnan(depth) or sawyer_calc.is_within_range(depth,1,3):
            depth_rand_array = [0.]
            for i in range(-15,15):
                for j in range(-15,15):
                    x_rand = x + i
                    y_rand = y + j
                    depth_rand_array.append(d_img[y_rand][x_rand])
            # print "NaN? :", depth_rand_array
            # print "Filter nan :", filter_nan(depth_rand_array)
            depth_rand_array = sawyer_calc.filter_nan(depth_rand_array)
            depth_rand_array = sawyer_calc.within_range_filter(depth_rand_array, 1, 3)
            depth_rand_array = sawyer_calc.reject_outliers(depth_rand_array, 0.5) # reject outliers seems to return very far
            depth = sawyer_calc.mean(depth_rand_array)
        norm_v = self.ph_model.projectPixelTo3dRay((x,y))
        scale = depth*1.0 / norm_v[2]
        pos = Point()
        pos_in_space = [z * scale for z in norm_v]
        pos.x = pos_in_space[0]
        pos.y = pos_in_space[1]
        pos.z = pos_in_space[2]

        if not np.isnan(depth):
            # print "tf sending"
            self.tf_br.sendTransform((pos.z,-pos.x,-pos.y), [0,0,0,1], rospy.Time.now(), "ball", "camera_link")        

    def specific_color_filter(self, img_hsv, img_raw):
        v = GREEN_TB_DEFAULTS
        # low and high band for detecting red color in hsv which spans at each end of h-band
        low_vals_lo = np.array([v[0], v[1], v[2]])
        high_vals_lo = np.array([v[3], v[4], v[5]])
        low_vals_hi = np.array([v[6], v[7], v[8]])
        high_vals_hi = np.array([v[9], v[10], v[11]])
        im_lo = cv2.inRange(img_hsv, low_vals_lo, high_vals_lo)
        im_hi = cv2.inRange(img_hsv, low_vals_hi, high_vals_hi)
        im_total = im_lo + im_hi
        # erode noisy details and dilate those left
        kernel_erode = np.ones((4,4),np.uint8)
        kernel_dilate = np.ones((7,7),np.uint8)
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
            print("detect_color_full_system.py : line 159 : No object found!! ")


    def image_cb(self, ros_img):
        #call the function from CvBridge
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
        img_blur = cv2.GaussianBlur(cv_image, (9,9), 1)
        img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)        
        # img_red, coords = self.specific_color_filter(img_hsv, cv_image)
        try:
            img_red, coords = self.specific_color_filter(img_hsv, cv_image)
            cv2.imshow('image', img_red)
            cv2.waitKey(1)
            self.tracked_pixel.x = coords[0]
            self.tracked_pixel.y = coords[1] 
        except TypeError, UnboundLocalError:
            print("detect_color_full_system.py : line 172 : No object found!! ")
   

    def keycb(self, tdat):
        # check if there was a key pressed, and if so, check it's value and
        # toggle flag:
        if self.kb.kbhit() and self.use_kb:
            c = self.kb.getch()
            if ord(c) == 27:
                rospy.signal_shutdown("Escape pressed!")
            else:
                print c
            if c == 's':
                rospy.loginfo("You pressed 's', Program starts. Sawyer is waiting for the ball to be thrown.")
                self.start_plot = True
            elif c == 'p':
                rospy.loginfo("You pressed 'h', Plotting")
                self.plot_flag = True
            else:
                self.print_help()
            self.kb.flush()
        return


    def print_help(self):
        help_string = \
        """
        's'   ~  Start the plot
        'p'   ~  Stop and plot
        'ESC' ~  Quit
        """
        print help_string
        return


def main():
    rospy.init_node('ball_3D_detector', log_level=rospy.INFO)

    try:
        use_kb = False
        balldetect = Obj3DDetector(use_kb)
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
