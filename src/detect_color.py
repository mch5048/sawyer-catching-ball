#!/usr/bin/env python
import rospy
import cv_bridge
import cv2
import numpy as np
from cv_bridge import (
    CvBridge
)
from sensor_msgs.msg import (
    Image ,
    PointCloud2
)

class Output(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

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
        
    def nothing(self, x):
        print "VALUE OF THE TRACKBAR IS: ", x

    def get_vals(self):
        vals = []
        for i in range(self.N):
            val = cv2.getTrackbarPos('val'+str(i), self.window_name)
            vals.append(val)
        return vals
        

def filter_red(img_hsv, img_raw):

    v = redwin.get_vals()
    low_vals = np.array([v[0], v[1], v[2]])
    high_vals = np.array([v[3], v[4], v[5]])
    red_lo = cv2.inRange(img_hsv, low_vals, high_vals)
    red_hi = cv2.inRange(img_hsv, np.array([v[6], v[7], v[8]]), np.array([v[9], v[10], v[11]]))
    red = red_lo + red_hi

    n = 2
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(n,n))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)

    red = cv2.bitwise_and(img_raw, img_raw, mask = red)
    return red
    
# def object_position(img):
#     greyscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     contours = cv2.findContours(greyscale,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]
#     return contours



def object_position(img):

    greyscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    contours = cv2.findContours(greyscale,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[0]

    c = max(contours,key=cv2.contourArea)
    # for i, c in enumerate(contours):
    # print i
    M = cv2.moments(c)
    if M["m00"] > 50:
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        except ZeroDivisionError:
            # continue
            coords = [0, 0, 0]
            return coords
        # print center
        # print center[0], center[1]
        epsilon = 0.003 * cv2.arcLength(c, True)
        c = cv2.approxPolyDP(c, epsilon, True)
        cv2.drawContours(img, [c], -1, (0, 255, 0), thickness=2, maxLevel=0)
        # cv2.drawContours(image, contours, contourIdx, color[, thickness[, lineType[, hierarchy[, maxLevel[, offset]]]]])
        cv2.circle(img, center, 5, (0, 255, 0), -1)
        # cv2.putText(img, str(i), (center[0] - 20, center[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    img_height, img_width, img_depth = img.shape

    try:
        # x = center[0] - img_width / 2
        # y = center[1] - img_height / 2
        # if abs(x) < 10:
        #     x = 0.
        # if abs(y) < 10:
        #     y = 0.
        # coords = [x, y, 0]
        
        #the coordinate of the center of a ball in an array
        x = center[0]
        y = center[1]
        # coords = [x, y, 0]
        coords = [x, y]
    except UnboundLocalError:
        # coords = [0, 0, 0]
        coords = [0, 0]
    return img, coords




def image_cb(ros_img, output):
    #convert ros_img to openCV
    
    #call the function from CvBridge
    bridge = CvBridge()
    
    # convert ros_ing to cv_image in bgr order
    cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
    # print "original size : ",cv_image.shape 
    #flip image around the x-axis : don't flip since we'll get a depth image
    # img_raw = cv2.flip(cv_image, 1)

    # img_width, img_height, img_depth = img_raw.shape (row, column, channel)

    img_blur = cv2.GaussianBlur(cv_image, (9,9), 1)
    img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
    #img_canny = cv2.Canny(img_hsv, cannywin.get_vals()[0], cannywin.get_vals()[1])
    #img_canny = cv2.Canny(img_hsv, 100, 100)
    img_red = filter_red(img_hsv, cv_image)
    tracked_red, coords = object_position(img_red)

    cv2.imshow('image',tracked_red)
    #Show picture for 10 ms
    cv2.waitKey(10)

    output.x = coords[0]
    output.y = coords[1]

# def get_pos(pc, output):
    
#     # Function to convert 2D pixel point to 3D point by extracting point
#     # from PointCloud2 corresponding to input pixel coordinate. This function
#     # can be used to get the X,Y,Z coordinates of a feature using an 
#     # RGBD camera, e.g., Kinect. resource : http://answers.ros.org/question/191265/pointcloud2-access-data/
    
#     # get width and height of 2D point cloud data
#     width = pc.width
#     height = pc.height

#     # Convert from u (column / width), v (row/height) to position in array where X,Y,Z data starts
#     u = output.y
#     v = output.x
#     arrayPosition = v*pc.row_step + u*pc.point_step
#     # print arrayPosition

#     # print "check : ", pc.fields[0].offset + 1
#     # print "check : ", arrayPosition - 200000
#     # compute position in array where x,y,z data start
#     arrayPosX = int(arrayPosition) + pc.fields[0].offset # X has an offset of 0
#     arrayPosY = int(arrayPosition) + int(pc.fields[1].offset) # Y has an offset of 4
#     arrayPosZ = int(arrayPosition) + int(pc.fields[2].offset) # Z has an offset of 8

#     print len(pc.data)
#     print arrayPosY
#     output.x = pc.data[arrayPosX]
#     output.y = pc.data[arrayPosY]
#     output.z = pc.data[arrayPosZ]
    
#     print "x : ", output.x, "\n"
#     print "Y : ", output.y, "\n"
#     print "Z : ", output.z, "\n" 
    

def callback_depth(depth_img, output):
    # check if coordinate is not zero for both x and y
    x = output.x
    y = output.y
    # access the depth map array to get the depth of the ball
    # depth = depth_img.data[x][y]

    # convert image to depth_image
    # print x,' , ' ,y

    bridge = CvBridge()
    d_img = bridge.imgmsg_to_cv2(depth_img)
    d_img = cv2.resize(d_img, (0,0), fx=2, fy=2) 

    # print d_img.size
    # d_img = np.array(d_img, dtype=np.float32)
    # print 'shape = ', d_img.shape
    print d_img[y][x]

if __name__ == '__main__':
    try:
        #initial node
        rospy.init_node('object_detect')
        #subscribe to image topic

        red_tb_highs = [179, 255, 255, 179, 255, 255, 179, 255, 255, 179, 255, 255]
        #From Baxter Barista
        # red_tb_defaults = [0, 130, 70, 10, 230, 255, 170, 0, 0, 179, 255, 255]
        #tune for camera at our desk in D11o
        # red_tb_defaults = [0, 255, 73, 0, 230, 255, 132, 177, 162, 179, 255, 255]     
        # red_tb_defaults = [0, 255, 73, 0, 230, 255, 53, 180, 162, 179, 255, 255] 
        red_tb_defaults = [0, 255, 75, 0, 230, 255, 46, 183, 169, 179, 255, 255] 
        output = Output()
        redwin = window_with_trackbars('image_red', red_tb_defaults, red_tb_highs)

        rospy.Subscriber("/camera/rgb/image_color", Image, image_cb, output)

        #map x and y coord to u and v of the point cloud
        # rospy.Subscriber("/camera/depth_registered/points", PointCloud2, get_pos, output)

        #access depth map
        rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, callback_depth, output)
        # rospy.Subscriber("/camera/depth_registered/image_rect", Image, callback_depth, output)


        rospy.spin()

    except rospy.ROSInterruptException:
        pass
