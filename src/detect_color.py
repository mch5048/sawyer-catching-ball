#!/usr/bin/env python
import rospy
import cv_bridge
import cv2
import numpy as np
import kinect_error_plot as kepl
from cv_bridge import (
    CvBridge
)
import image_geometry  
from geometry_msgs.msg import Point
from sensor_msgs.msg import (
    Image ,
    PointCloud2,
    CameraInfo
)
from std_msgs.msg import Float64
import tf
import random
import matplotlib.pyplot as plt

class Plot(object):
    def __init__(self):
        self.x = []
        self.y = []
        self.depth = []

    def plot(self, x, y, depth, t):
        plt.plot(x, t, '-', label='label here')
        plt.plot(y, t, ':', label='label here')
        plt.plot(depth, t, '--', label='label here')
        plt.show()

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
    # h: 0 - 25, s: 70 - 100 v :  
    low_vals_red_lo = np.array([v[0], v[1], v[2]])
    high_vals_red_lo = np.array([v[3], v[4], v[5]])
    low_vals_red_hi = np.array([v[6], v[7], v[8]])
    high_vals_red_hi = np.array([v[9], v[10], v[11]])
    red_lo = cv2.inRange(img_hsv, low_vals_red_lo, high_vals_red_lo)
    # red_hi = cv2.inRange(img_hsv, np.array([v[6], v[7], v[8]]), np.array([v[9], v[10], v[11]]))
    red_hi = cv2.inRange(img_hsv, low_vals_red_hi, high_vals_red_hi)
    red = red_lo + red_hi
    kernel_erode = np.ones((4,4),np.uint8)
    kernel_dilate = np.ones((7,7),np.uint8)
    red = cv2.erode(red, kernel_erode, iterations=2)
    red = cv2.dilate(red, kernel_dilate, iterations=2)
    redContours = cv2.findContours(red.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

    center = None
 
    # only proceed if at least one contour was found
    if len(redContours) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(redContours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(img_raw, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(img_raw, center, 5, (0, 0, 255), -1)
    # red.appendleft(center)
    # n = 2
    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(n,n))
    # red = cv2.morphologyEx(red, cv2.MORPH_OPEN, kernel)
    # red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)

    # red = cv2.bitwise_and(img_raw, img_raw)
    red = cv2.bitwise_and(img_raw, img_raw, mask = red)
    # print "center = ", center
    # print "list(center)", list(center)
    return red, list(center)
    
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

    # img_height, img_width, img_depth = img.shape

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
    img_red, coords = filter_red(img_hsv, cv_image)
    # tracked_red, coords = object_position(img_red)
    cv2.imshow('image', img_red)

    # cv2.imshow('image',tracked_red)
    #Show picture for 10 ms
    cv2.waitKey(1)

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
    
def update_model_cb(info):
    ph_model.fromCameraInfo(info)

# def reject_outliers(data, m = 2.):
#     d = np.abs(data - np.median(data))
#     mdev = np.median(d)
#     s = d/mdev if mdev else 0.
#     dat_ret = []
#     return data[s<m]

def reject_outliers(data, m = 2.):
    data = sorted(data)
    data_raw = list(data)
    dat_ret = []
    med = data[int(len(data)/2)]
    for i in range(len(data)):
        data[i] = abs(data[i] - med)
    data = sorted(data)
    mdev = data[int(len(data)/2)]
    for i in range(len(data)):
        data[i] = data[i]/mdev if mdev else 0
    for i in range(len(data)):
        if data[i] < m:
            dat_ret.append(data_raw[i])
    return dat_ret

def mean(data):
    return  1.0*sum(data)/len(data)

def within_range_filter(data, min_range, max_range):
    dat_ret = []
    for i in range(len(data)):
        if data[i] < max_range and data[i] > min_range:
            dat_ret.append(data[i])
    return dat_ret

def filter_nan(data):
    # print "data in filternan:", data
    data_len = len(data)
    dat_ret = []
    for i in range(data_len):
        if not np.isnan(data[i]):
        #     np.append(dat_ret, data[i])
            dat_ret.append(data[i])
    return dat_ret
         
def is_within_range(num, low, high):
    if num < high and num > low:
        return True
    else:
        return False


# def reject_outliers(data, m = 2.):
#     d = np.abs(data - np.median(data))
#     mdev = np.median(d)
#     s = d/mdev if mdev else 0.
#     return data[s<m]


# def callback_depth(depth_img, output, last_depth):
def callback_depth(depth_img, output):
    # check if coordinate is not zero for both x and y
    x = output.x 
    y = output.y
    # print "dat", dat
    # x = dat[0].x 
    # y = dat[1].y
    

    bridge = CvBridge()
    d_img = bridge.imgmsg_to_cv2(depth_img)
    d_img = cv2.resize(d_img, (0,0), fx=2, fy=2)


        #if not NaN or Outlier (beyond ball radius), increment depth 

        
    depth = d_img[y][x]

    # if np.isnan(depth):
    #     # print "depth nan:", depth
    #     # print "output.x : ", x, " output.y : ", y
    #     # counter = 0
    #     depth_rand_array = np.zeros(5)
    #     for i in range(5):
    #         #random x and y
    #         not_nan = False
    #         while not not_nan:
    #             x_rand = x + random.randint(-15,15)
    #             y_rand = y + random.randint(-15,15)
    #             #get depth 
    #             depth_rand = d_img[y_rand][x_rand]
    #             print "Nan ? :", depth_rand
    #             if not np.isnan(depth_rand):
    #                 depth_rand_array[i] = depth_rand
    #                 not_nan = True
    #         # take all outliers out
    #     # depth_rand_array = reject_outliers(depth_rand_array)
    #     depth = np.mean(depth_rand_array)

    # if np.isnan(depth):
    #     depth_rand_array = [0]*5
    #     for i in range(5):
    #         #random x and y
    #         not_nan = False
    #         while not not_nan:
    #             x_rand = x + random.randint(-7,7)
    #             y_rand = y + random.randint(-7,7)
    #             #get depth 
    #             depth_rand = d_img[y_rand][x_rand]
    #             depth_rand_array[i] = depth_rand
    #             if not np.isnan(depth_rand):
    #                 depth_rand_array[i] = depth_rand
    #                 not_nan = True
    #         # take all outliers out
    #     # depth_rand_array = reject_outliers(depth_rand_array)
    #     # depth = mean(depth_rand_array)
    #     depth = min(depth_rand_array
    # print "depth:",depth
    if np.isnan(depth) or not is_within_range(depth,1,3):
        depth_rand_array = [0]*5
        # depth_rand_array = np.zeros(1)
        #random x and y
        for i in range(-15,15):
            for j in range(-15,15):
                x_rand = x + i
                y_rand = y + j
                #get depth 
                # np.append(depth_rand_array,np.array([d_img[y_rand][x_rand]]))
                depth_rand_array.append(d_img[y_rand][x_rand])
        # print "NaN? :", depth_rand_array
        # print "Filter nan :", filter_nan(depth_rand_array)
        depth_rand_array = filter_nan(depth_rand_array)
        depth_rand_array = within_range_filter(depth_rand_array, 1, 3)
        depth_rand_array = reject_outliers(depth_rand_array, 0.5) # reject outliers seems to return very far
        depth = mean(depth_rand_array)
        print "mean depth : ", depth

    # if np.isnan(depth):
    #         #random x and y
    #     x_rand = x + random.randint(-7,7)
    #     y_rand = y + random.randint(-7,7)
    #     #get depth 
    #     depth = d_img[y_rand][x_rand]
       

    norm_v = ph_model.projectPixelTo3dRay((x,y))
    scale = depth*1.0 / norm_v[2]


    pos = Point()
    pos_in_space = [z * scale for z in norm_v]
    pos.x = pos_in_space[0]
    pos.y = pos_in_space[1]
    pos.z = pos_in_space[2]
    # print "x ,y, z : ", pos.x, ", ", pos.y, ", ", pos.z

    if not np.isnan(depth):
        # print "tf sending"
        tf_br.sendTransform((pos.z,-pos.x,-pos.y), [0,0,0,1], rospy.Time.now(), "ball", "camera_link")        
        pos_pub.publish(pos)
        

    # plt.plot(<X AXIS VALUES HERE>, <Y AXIS VALUES HERE>, 'line type', label='label here')
    # plt.plot(<X AXIS VALUES HERE>, <Y AXIS VALUES HERE>, 'line type', label='label here')
    # plt.show()



# def callback_depth_test(depth_img, output):
#     # check if coordinate is not zero for both x and y
#     x = output.x
#     y = output.y
#     # access the depth map array to get the depth of the ball
#     # depth = depth_img.data[x][y]

#     # convert image to depth_image
#     # print x,' , ' ,y

#     bridge = CvBridge()
#     d_img = bridge.imgmsg_to_cv2(depth_img)
#     d_img = cv2.resize(d_img, (0,0), fx=2, fy=2) 

#     # print d_img.size
#     # d_img = np.array(d_img, dtype=np.float32)
#     # print 'shape = ', d_img.shape
#     depth = d_img[y][x]
#     # kinect_error_plot.depth_update(depth)
#     # depth_pub.publish(Float64(depth))
#     print "depth_img_raw_test", depth

if __name__ == '__main__':
    try:
        rospy.init_node('object_detect')
        #subscribe to image topic

        red_tb_highs = [179, 255, 255, 179, 255, 255, 179, 255, 255, 179, 255, 255]
        #From Baxter Barista

        # red_tb_defaults = [0, 130, 70, 10, 230, 255, 170, 0, 0, 179, 255, 255]
        #tune for camera at our desk in D11o
        # red_tb_defaults = [0, 255, 73, 0, 230, 255, 132, 177, 162, 179, 255, 255]     
        # red_tb_defaults = [0, 255, 73, 0, 230, 255, 53, 180, 162, 179, 255, 255] 

        # red_tb_defaults = [0, 255, 75, 0, 230, 255, 46, 183, 169, 179, 255, 255] 
        # red_tb_defaults = [152, 0, 0, 179, 255, 255, 46, 183, 169, 179, 255, 255] 
        # red_tb_defaults = [164, 51, 55, 179, 255, 255, 0, 107, 84, 11, 255, 255] 
        red_tb_defaults = [164, 198, 157, 179, 255, 255, 0, 195, 105, 11, 255, 255] #Best
        red_tb_defaults = [164, 198, 203, 179, 255, 255, 0, 223, 127, 11, 255, 255] #Best
        green_tb_highs = [179, 255, 255, 179, 255, 255, 179, 255, 255, 179, 255, 255]
        green_tb_defaults = [27, 105, 110, 66, 255, 255, 0, 0, 0, 0, 0, 0]
        pos = Output()
        output = Output()
        redwin = window_with_trackbars('image_red', green_tb_defaults, green_tb_highs)

        ph_model = image_geometry.PinholeCameraModel()
        rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, update_model_cb)
        # kinect_error_plot
        rospy.Subscriber("/camera/rgb/image_color", Image, image_cb, output)

        #map x and y coord to u and v of the point cloud
        # rospy.Subscriber("/camera/depth_registered/points", PointCloud2, get_pos, output)
        tf_br = tf.TransformBroadcaster()
        # depth_pub = rospy.Publisher("red_ball/depth", Float64, queue_size = 10)
        pos_pub = rospy.Publisher("tracked_obj/position", Point, queue_size = 10)
        #access depth map
        rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, callback_depth, output)
        


        # rospy.Subscriber("/camera/depth_registered/image_rect", Image, callback_depth, output)


        rospy.spin()

    except rospy.ROSInterruptException:
        pass
