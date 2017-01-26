#!/usr/bin/env python
import rospy
import cv_bridge
import cv2
from cv_bridge import (
    CvBridge
)
from sensor_msgs.msg import (
    Image
)


def image_cb(ros_img):
    #convert ros_img to openCV
    
    #call the function from CvBridge
    bridge = CvBridge()
    
    # convert ros_ing to cv_image in bgr order
    cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
    
    #flip image around the x-axis
    img_raw = cv2.flip(cv_image, 1)
    # img_width, img_height, img_depth = img_raw.shape (row, column, channel)

    cv2.imshow('image',img_raw)
    #Show picture for 10 ms
    cv2.waitKey(10)

    

if __name__ == '__main__':
    try:
        #initial node
        rospy.init_node('object_detect')
        #subscribe to image topic
        rospy.Subscriber("/camera/rgb/image_color", Image, image_cb)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
