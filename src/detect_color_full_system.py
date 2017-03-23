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
# GREEN_TB_HIGHS = [179, 255, 255, 179, 255, 255, 179, 255, 255, 179, 255, 255,15,15, 255, 255, 255, 255, 255, 255]
# # GREEN_TB_DEFAULTS = [27, 105, 110, 66, 255, 255, 0, 0, 0, 0, 0, 0, 4, 8]
# GREEN_TB_DEFAULTS = [27, 105, 110, 66, 255, 255, 0, 0, 0, 0, 0, 0, 2, 8, 0, 0, 0, 0, 0, 0] # the setting facing the door of D110
GREEN_TB_HIGHS = [179, 255, 255, 179, 255, 255, 15,15, 255, 255, 255, 255, 255, 255]
GREEN_TB_DEFAULTS = [27, 105, 110, 66, 255, 255,  2, 8, 0, 133, 0, 201, 255, 255]
WITHIN_RAN_MIN = 0
WITHIN_RAN_MAX = 3.2
OUTLIER_FILT_NUM = 0.25

class Obj3DDetector(object):


    class window_with_trackbars(object):
        def __init__(self, window_name, tb_defaults=[], tb_highs=[]):
            #initialie variables and setup
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
        self.start_flag = False
        self.ph_model = image_geometry.PinholeCameraModel()
        self.tracked_pixel = Point()
        self.use_kb = False
        self.trackbar = self.window_with_trackbars('image_tb', GREEN_TB_DEFAULTS, GREEN_TB_HIGHS)
        self.within_ran_min = WITHIN_RAN_MIN
        self.within_ran_max = WITHIN_RAN_MAX

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
        self.rgb_im_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_cb)
        self.image_rect_sub = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, self.depth_cb)

        # publishers and timers
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)
        self.start_time = 0
        self.tf_br = tf.TransformBroadcaster()

        # plotting assistant
        self.plotter = rospy.Timer(rospy.Duration(0.01), self.plotter_cb)
        self.plot_flag = False
        self.rec_plot_flag = False
        self.plot_input_pixel_x = []
        self.plot_input_pixel_y = []
        self.plot_x = []
        self.plot_y = []
        self.plot_z = []
        self.plot_t = []
        self.plot_isnan = []
        self.plot_isnan_t = []
        self.pres_x = 0
        self.pres_y = 0
        self.pres_z = 0
        self.pres_isnan = 0
        # plot inputted pixel
        self.pres_input_pix_y = 0
        self.pres_input_pix_x = 0
        # plot returned depth
        self.plot_depth = []
        self.pres_depth = 0
        # plot returned norm vector
        self.plot_norm_x = []
        self.plot_norm_y = []
        self.plot_norm_z = []
        self.pres_norm_x = 0
        self.pres_norm_y = 0
        self.pres_norm_z = 0
        self.dirname = ''


        # self.actual_js_sub = rospy.Subscriber("robot/joint_states", JointState, self.actual_js_cb)
        # self.js_pub = rospy.Publisher("joint_states", JointState, queue_size=3)

    def plotter_cb(self, tdat):
        if self.plot_flag:
            self.rec_plot_flag = False
            self.plot_flag = False
            # print "plot_x ", self.plot_x 
            # print "plot_y ", self.plot_y 
            # print "plot_z ", self.plot_z 
            # print "plot_t ", self.plot_t 
            # x_patch = mpatches.Patch(color='red', hatch='o', label='x(depth)')
            # y_patch = mpatches.Patch(color='g', hatch='.', label='y')
            # z_patch = mpatches.Patch(color='b', hatch='o', label='z')
            # nan_patch = mpatches.Patch(color='b', hatch='x', label='NaN')
            plt.plot(self.plot_t, self.plot_x, 'ro', label='x(depth)')
            plt.plot(self.plot_t, self.plot_y, 'g.', label='y')
            plt.plot(self.plot_t, self.plot_z, 'bo', label='z')
            plt.plot(self.plot_t, self.plot_depth, 'yo', label='raw_depth')
            plt.plot(self.plot_isnan_t, self.plot_isnan, 'x', label='NaN')
            # plt.legend(handles=[x_patch])
            # plt.legend(handles=[y_patch])
            # plt.legend(handles=[z_patch])
            # plt.legend(handles=[nan_patch])
            # plt.legend(handles=[leg_x, leg_y, leg_z, leg_nan])
            plt.show()
            plt.plot(self.plot_t, self.plot_input_pixel_x, 'rx', label='NaN')
            plt.plot(self.plot_t, self.plot_input_pixel_y, 'gx', label='NaN')
            plt.show()
            plt.plot(self.plot_t, self.plot_norm_x, 'r.', label='x(depth)')
            plt.plot(self.plot_t, self.plot_norm_y, 'g.', label='y')
            plt.plot(self.plot_t, self.plot_norm_z, 'b.', label='z')
            plt.show()
            plt.plot(self.plot_t, self.plot_x, 'ro', label='x(depth)')
            plt.plot(self.plot_t, self.plot_y, 'g.', label='y')
            plt.plot(self.plot_t, self.plot_z, 'bo', label='z')
            # plt.plot(self.plot_t, self.plot_depth, 'r.', label='raw_depth')
            plt.plot(self.plot_isnan_t, self.plot_isnan, 'x', label='NaN')
            filename_traj_plot = "pos3D_olierFilt-%.3f_wtRanFilt-%.2f-%.2f.jpg" % (OUTLIER_FILT_NUM, WITHIN_RAN_MIN, WITHIN_RAN_MAX)
            # plt.savefig(os.path.join(self.dirname, 'traj_plot.jpg'))
            plt.plot(self.plot_t, self.plot_depth, 'yo', label='raw_depth')
            plt.savefig(os.path.join(self.dirname, filename_traj_plot))
            plt.close()
            # plt.plot(self.plot_t, self.plot_depth, 'yo', label='raw_depth')
            # plt.savefig(os.path.join(self.dirname, 'raw_depth.jpg'))
            # plt.close()
            plt.plot(self.plot_t, self.plot_input_pixel_x, 'rx', label='NaN')
            plt.plot(self.plot_t, self.plot_input_pixel_y, 'gx', label='NaN')
            time_name = time.strftime("%y_%m_%d_%Hh-%Mm-%Ss") 
            filename_pix_plot = "%s_%s.jpg" % ("tracked_pixel",time_name)
            # plt.savefig(os.path.join(self.dirname, 'tracked_pixel_plot.jpg'))
            plt.savefig(os.path.join(self.dirname, filename_pix_plot))
            plt.close()
            plt.plot(self.plot_t, self.plot_norm_x, 'r.', label='x(depth)')
            plt.plot(self.plot_t, self.plot_norm_y, 'g.', label='y')
            plt.plot(self.plot_t, self.plot_norm_z, 'b.', label='z')
            filename_raw_norm = "%s_%s.jpg" % ("raw_3dRay",time_name)
            plt.savefig(os.path.join(self.dirname, filename_raw_norm))
            # write to csv file
            with open(os.path.join(self.dirname, 'csv'), 'wb') as csvfile:
                spamwriter = csv.writer(csvfile, delimiter=' ', quotechar=' ', quoting=csv.QUOTE_MINIMAL)
                spamwriter.writerow(['u', 'v', 'repeated?',' | ', 'v_x', 'v_y', 'v_z'])
                rgb_repeat = '0'
                norm_repeat = '0'
                both_repeat = 'False'
                for s in range(len(self.plot_t)):
                    for t in range(3):
                        if s != 0 and (self.plot_input_pixel_x[s-1] == self.plot_input_pixel_x[s]) and (self.plot_input_pixel_y[s-1] == self.plot_input_pixel_y[s]):
                            rgb_repeat = '1'
                        if s != 0 and (self.plot_norm_x[s-1] == self.plot_norm_x[s]) and  (self.plot_norm_y[s-1] == self.plot_norm_y[s]) and  (self.plot_norm_z[s-1] == self.plot_norm_z[s]):
                            norm_repeat = '1'
                        if rgb_repeat == '1' and norm_repeat == '1':
                            both_repeat = 'True'
                    time_write = "%.4f" % (self.plot_t[s])
                    spamwriter.writerow([time_write, self.plot_input_pixel_x[s], self.plot_input_pixel_y[s] ,' | ', self.plot_norm_x[s], self.plot_norm_y[s], self.plot_norm_z[s], ' | ', rgb_repeat, norm_repeat, ' | ', both_repeat])
                    rgb_repeat = '0'
                    norm_repeat = '0'
                    both_repeat = 'False'
            # self.plot_x = []
            # self.plot_y = []
            # self.plot_z = []
            # self.plot_t = []
            # self.plot_isnan = []
            # self.plot_isnan_t = []
            self.pres_x = 0
            self.pres_y = 0
            self.pres_z = 0
            self.pres_isnan = 0
            self.pres_input_pix_y = 0
            self.pres_input_pix_x = 0
            self.pres_depth = 0
            del self.plot_x[:]
            del self.plot_y[:]
            del self.plot_z[:]
            del self.plot_t[:]
            del self.plot_isnan[:]
            del self.plot_isnan_t[:]
            del self.plot_depth[:]


    def get_and_set_params(self):
        self.use_kb = rospy.get_param("kb", True)



    def update_model_cb(self, info):
        self.ph_model.fromCameraInfo(info)


    def depth_cb(self, depth_img):
        try :
            x = self.tracked_pixel.x
            y = self.tracked_pixel.y
            bridge = CvBridge()
            d_img = bridge.imgmsg_to_cv2(depth_img)
            d_img = cv2.resize(d_img, (0,0), fx=2, fy=2)
            self.pres_input_pix_y = -self.tracked_pixel.y
            self.pres_input_pix_x = -self.tracked_pixel.x
            depth = d_img[y][x]
            is_nan = 0
            # if np.isnan(depth) or not sawyer_calc.is_within_range(depth,1,3):
            #     depth_rand_array = [0.]
            #     for i in range(-15,15):
            #         for j in range(-15,15):
            #             x_rand = x + i
            #             y_rand = y + j
            #             depth_rand_array.append(d_img[y_rand][x_rand])
            #             # print "NaN? :", depth_rand_array
            #             # print "Filter nan :", filter_nan(depth_rand_array)
            #     depth_rand_array = sawyer_calc.filter_nan(depth_rand_array)
            #     depth_rand_array = sawyer_calc.within_range_filter(depth_rand_array, WITHIN_RAN_MIN, WITHIN_RAN_MAX)
            #     # depth_rand_array = sawyer_calc.within_range_filter(depth_rand_array, self.within_ran_min, self.within_ran_max)
            #     depth_rand_array = sawyer_calc.reject_outliers(depth_rand_array, OUTLIER_FILT_NUM) # reject outliers that seems to return very far depth
            #     depth = sawyer_calc.mean(depth_rand_array)
            #     self.within_ran_max = depth + 0.2
            #     self.within_ran_min = 0
            #     is_nan = 0.1
            depth_rand_array = [0.]
            if np.isnan(depth):
                is_nan = 0.1
            for i in range(-15,15):
                for j in range(-15,15):
                    x_rand = x + i
                    y_rand = y + j
                    depth_rand_array.append(d_img[y_rand][x_rand])
                    # print "NaN? :", depth_rand_array
                    # print "Filter nan :", filter_nan(depth_rand_array)
            depth_rand_array = sawyer_calc.filter_nan(depth_rand_array)
            depth_rand_array = sawyer_calc.within_range_filter(depth_rand_array, WITHIN_RAN_MIN, WITHIN_RAN_MAX)
            # depth_rand_array = sawyer_calc.within_range_filter(depth_rand_array, self.within_ran_min, self.within_ran_max)
            depth_rand_array = sawyer_calc.reject_outliers(depth_rand_array, OUTLIER_FILT_NUM) # reject outliers that seems to return very far depth
            depth = sawyer_calc.mean(depth_rand_array)
            

            
            norm_v = self.ph_model.projectPixelTo3dRay((x,y))

            # check if the function return the same vector every time
            old_norm = norm_v
            for k in range(5):
                norm_v = self.ph_model.projectPixelTo3dRay((x,y))
                if cmp(old_norm,norm_v) != 0:
                    print "not equal in five times"
                old_norm = norm_v
                # print old_norm

            scale = depth*1.0 / norm_v[2]
            pos = Point()
            pos_in_space = [z * scale for z in norm_v]
            pos.x = pos_in_space[0]
            pos.y = pos_in_space[1]
            pos.z = pos_in_space[2]

            if not np.isnan(depth):
                # print "tf sending"
                self.tf_br.sendTransform((pos.z,-pos.x,-pos.y), [0,0,0,1], rospy.Time.now(), "ball", "camera_link")
                self.pres_x = pos.z
                self.pres_y = -pos.x
                self.pres_z = -pos.y
                self.pres_isnan = is_nan
                self.pres_depth = depth
                # plot returned norm vector
                plot_norm = [z for z in norm_v]
                self.pres_norm_x = plot_norm[2]
                self.pres_norm_y = -plot_norm[0]
                self.pres_norm_z = -plot_norm[1]
                # if self.rec_plot_flag:
                #     self.plot_x.append(pos.z)
                #     self.plot_y.append(-pos.x)
                #     self.plot_z.append(-pos.y)
                #     self.plot_t.append(rospy.get_time())
        except IndexError:
            pass

    def specific_color_filter(self, img_hsv, img_raw):


        # change black color to white color since green detection always detect black color too
        b = self.trackbar.get_vals()[8:14]
        black_lo = np.array([b[0], b[1], b[2]])
        black_hi = np.array([b[3], b[4], b[5]])
        im_b_to_w = cv2.inRange(img_hsv, black_lo, black_hi) #th mask that cut the black portion out
        # im_b_to_w = cv2.bitwise_not(im_b_to_w) # the mask that cut black portion out

        b_to_w_color_im = cv2.bitwise_and(img_raw, img_raw, mask = im_b_to_w)
        # img_blur = cv2.GaussianBlur(b_to_w_color_im, (9,9), 1)
        img_hsv = cv2.cvtColor(b_to_w_color_im, cv2.COLOR_BGR2HSV)   
        # cv2.imshow('b to w', img_hsv)
        # im_white = np.zeros(img_hsv.shape, dtype=np.uint8)
        # x_range = img_hsv.shape[0]
        # y_range = img_hsv.shape[1]
        # im_white.fill(255)
        # im_white = cv2.cvtColor(im_white, cv2.COLOR_BGR2HSV)
        # print "img_raw : ", img_raw.shape, ", " ,img_raw.size, ", ",img_raw.dtype
        # print "im_white : ", im_white.shape, ", " ,im_white.size, ", ",im_white.dtype
        # for i in range(x_range):
        #     for j in range(y_range):
        #         im_white[i][j][2] = 255
        # b_to_w_color_im = cv2.bitwise_and(img_raw, im_white, mask = im_b_to_w)
        # b_to_w_color_im = cv2.bitwise_not(img_raw, mask = im_b_to_w)


        # v = self.trackbar.get_vals()[0:12]
        v = self.trackbar.get_vals()[0:6]
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
        e = self.trackbar.get_vals()[6:8]
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
            # img_detect_color, coords = self.specific_color_filter(img_hsv, cv_image)
            img_detect_color, coords = self.specific_color_filter(cv_image, cv_image)
            # cv2.imshow('image_detect', img_detect_color)
            cv2.imshow('image_raw', cv_image)
            if self.rec_plot_flag:
                time = rospy.get_time() - self.start_time
                filename_det = "%s_%d_%f.jpg" % ("im_detect", self.imwritecounter,time)
                filename_raw = "%s_%d_%f.jpg" % ("im_raw", self.imwritecounter,time)
                self.imwritecounter += 1
                cv2.imwrite(os.path.join(self.dirname, filename_det), img_detect_color)
                cv2.imwrite(os.path.join(self.dirname, filename_raw), cv_image)
                self.plot_input_pixel_x.append(self.pres_input_pix_x)
                self.plot_input_pixel_y.append(self.pres_input_pix_y)
                self.plot_x.append(self.pres_x)
                self.plot_y.append(self.pres_y)
                self.plot_z.append(self.pres_z)
                self.plot_t.append(time) 
                self.plot_isnan.append(self.pres_isnan)
                self.plot_isnan_t.append(time)
                self.plot_norm_x.append(self.pres_norm_x)
                self.plot_norm_y.append(self.pres_norm_y)
                self.plot_norm_z.append(self.pres_norm_z)
                self.plot_depth.append(self.pres_depth)
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
            if c == 'p':
                rospy.loginfo("You pressed 'p', Plotting")
                self.rec_plot_flag = True
                self.dirname = time.strftime("%y_%m_%d_%Hh-%Mm-%Ss")  
                os.mkdir(self.dirname)
                self.start_time = rospy.get_time()
            elif c == 'e':
                rospy.loginfo("You pressed 'e', Stop plotting")
                self.plot_flag = True
            else:
                self.print_help()
            self.kb.flush()
        return


    def print_help(self):
        help_string = \
        """
        'p'   ~  Start the plot
        'e'   ~  Stop and plot
        'ESC' ~  Quit
        """
        print help_string
        return


def main():
    rospy.init_node('ball_3D_detector', log_level=rospy.INFO)

    try:
        # use_kb = True
        balldetect = Obj3DDetector()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()




