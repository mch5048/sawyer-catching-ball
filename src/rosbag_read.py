#!/usr/bin/env python


###############
# ROS IMPORTS #
###############
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class RosbagRead(object):
    
    def __init__(self):
        rospy.loginfo("rosbag read")
        self.bridge = CvBridge()
        self.pix_x = 0
        self.pix_y = 0
        self.radius = 0

        self.pixel_sub = rospy.Subscriber("/pixel_coord", Point, self.get_pixel_cb)
        self.d_img_sub = rospy.Subscriber("/depth_img", Image, self.depth_im_cb)
        self.rgb_img_sub = rospy.Subscriber("/rgb_img", Image, self.rgb_im_cb)

    def rgb_im_cb(self, rgb_img):
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_img)
        cv2.imshow('rgb_img', rgb_img)
        cv2.waitKey(1)
        return

    def get_pixel_cb(self, pixel):
        self.pix_x = pixel.x
        self.pix_y = pixel.y
        self.radius = pixel.z
        return
        
    def depth_im_cb(self, depth_img):
        d_img = self.bridge.imgmsg_to_cv2(depth_img)
        d_img = cv2.resize(d_img, (0,0), fx=2, fy=2)
        # cv2.imshow('d_img', d_img)
        print d_img[self.pix_y][self.pix_x]
        d_rgb_im = cv2.cvtColor(d_img,cv2.COLOR_GRAY2RGB)

        # draw the circle around the ball according to x, y and radius
        cv2.circle(d_rgb_im, (int(self.pix_x), int(self.pix_y)), int(self.radius) ,(0, 255, 255), 2)
        # d_rgb_im[self.pix_y][self.pix_x] = [255, 0, 0] 
        cv2.imshow('d_rgb_im', d_rgb_im)
        cv2.waitKey(1)
        return


def main():
    rospy.init_node('rosbag_read', log_level=rospy.INFO)

    try:
        rosbagread = RosbagRead()
    except rospy.ROSInterruptException: pass
    
    rospy.spin()

if __name__=='__main__':
    main()
