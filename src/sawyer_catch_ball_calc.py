import numpy as np
from math import cos, sin, radians, sqrt, atan2, atan, tan
from geometry_msgs.msg import Point, PointStamped


g = 9.81

s10 = sin(radians(10)) 
c10 = cos(radians(10))

Blist = np.array([[s10, -c10, 0., -1.0155*c10, -1.0155*s10, -0.1603],
                  [-c10, -s10, 0., -0.9345*s10, 0.9345*c10, 0.],
                  [0. , 0., 1., -0.0322*s10, 0.0322*c10, 0.],
                  [-c10, -s10, 0., -0.5345*s10, 0.5345*c10, 0.],
                  [0., 0., 1., 0.1363*s10, -0.1363*c10, 0.],
                  [-c10, -s10, 0., -0.1345*s10, 0.1345*c10, 0.],
                  [0., 0., 1., 0., 0., 0.]])
Blist = Blist.T


M = np.array([[0., 0., 1., 1.0155],
              [-c10, -s10, 0., 0.1603],
              [s10, -c10, 0., 0.317],
              [0., 0., 0., 1.]])

#######################
# PHYSICS CALCULATION #
#######################

def projectile_calc(pointstamped_0, pointstamped_1, z_ref):
    p0 = pointstamped_0.point
    p1 = pointstamped_1.point
    t0 = pointstamped_0.header.stamp.secs + pointstamped_0.header.stamp.nsecs*(10**(-9))
    t1 = pointstamped_1.header.stamp.secs + pointstamped_1.header.stamp.nsecs*(10**(-9))

    dt = t1 - t0

    x0 = p0.x 
    x1 = p1.x
    y0 = p0.y
    y1 = p1.y
    z0 = p0.z 
    z1 = p1.z

    g = 9.81

    alpha = atan2(y1-y0, x1-x0)

    # find dropping time
    a = g/2
    b = -(z1-z0)/dt
    c = z_ref - z1
    try:
        tDropList = [(-b + sqrt(b**2 - 4*a*c))/(2*a), (-b - sqrt(b**2 - 4*a*c))/(2*a)]   
    except ValueError:
        print "Quadratic error: "
        print "a: ", a
        print "b: ", b
        print "c: ", c
        print "z1: ", z1
        print "z0: ", z0
        print "z_ref: ", z_ref
    tDrop = max(tDropList)

    #find final position
    xFin = x1 + ((x1-x0)/dt)*tDrop
    yFin = y1 + ((x1-x0)/dt)*tan(alpha)*tDrop
    zFin = z1 + (((z1-z0)/dt)*tDrop) - (0.5*g*(tDrop**2))

    point_ret = Point()
    point_ret.x = xFin
    point_ret.y = yFin
    point_ret.z = zFin

    return point_ret

###################
# ROS MSG FILTERS #
###################

def point_msg_reject_outliers_xAxis(point_arr, m = 2.):
# take an array of ros points and reject outlier points in specified axis
    # create a list collecting [Point, data]
    if point_arr != []:
        p_arr = [[p.x, p] for p in point_arr]
        p_arr = sorted(p_arr, key=lambda x: x[0])
        p_arr_raw = [p_arr[j][1] for j in range(len(p_arr))]
        p_arr_ret = []
        med = (p_arr[int(len(p_arr)/2)])[0]
        p_arr = [[abs(p_arr[i][0] - med), p_arr[i][1]] for i in range(len(p_arr))]
        p_arr = sorted(p_arr, key=lambda x: x[0])
        mdev = p_arr[int(len(p_arr)/2)][0]
        for i in range(len(p_arr)):
            p_arr[i][0] = p_arr[i][0]/mdev if mdev else 0
        for i in range(len(p_arr)):
            if p_arr[i][0] < m:
                p_arr_ret.append(p_arr_raw[i])
        return p_arr_ret
    else :
        print "Error in point_msg_reject_outliers_1axis function : Input is null set, [] will be returned"
        
def point_msg_avg(point_arr):
    count = len(point_arr)
    x_total = sum([ p.x for p in point_arr])
    y_total = sum([ p.y for p in point_arr])
    z_total = sum([ p.z for p in point_arr])
    return Point(x_total/count, y_total/count, z_total/count)

######################
# POINTCLOUD FILTERS #
######################

def reject_outliers(data, m = 2.):
# take matrix and return the one with no outliers
    if data != []:
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
    else :
        print "Error in reject_outliers function : Input is null set, [] will be returned"
        return []



def mean(data):
# return mean of matrix. (For Python 3.6.1 and upper, use statistics libs instead)
# check version of Python with "python -V"
    if data != []:
        return  1.0*sum(data)/len(data)
    else:
        print "Error in mean function : Input is null set, 0 will be returned"
        return 0




def within_range_filter(data, min_range, max_range):
# return matrix with data within specified range
    dat_ret = []
    # print 'before\n', data
    for i in range(len(data)):
        if data[i] < max_range and data[i] >= min_range:
            dat_ret.append(data[i])
    # print 'after\n', dat_ret
    return dat_ret




def filter_nan(data):
# filter NaN data in matrix
    # print "data in filternan:", data
    data_len = len(data)
    dat_ret = []
    for i in range(data_len):
        if not np.isnan(data[i]):
        #     np.append(dat_ret, data[i])
            dat_ret.append(data[i])
    return dat_ret
         



def is_within_range(num, low, high):
# check if the number is within specified range or not
    if num < high and num > low:
        return True
    else:
        return False
