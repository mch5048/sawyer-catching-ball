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
    tDropList = [(-b + sqrt(b**2 - 4*a*c))/(2*a), (-b - sqrt(b**2 - 4*a*c))/(2*a)]   
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


def projectile_calc_ver3(pointstamped_0, pointstamped_1, z_ref):
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
    
    x_delta = x1 - x0
    y_delta = y1 - y0
    z_delta = z1 - z0

    # alpha = atan2(-(y_delta), abs(x_delta))
    alpha = atan2(y_delta, x_delta)

    # project to 2d position
    ## find initial velocity
    uVert = abs(x_delta/cos(alpha))/dt
    uHor = abs(z_delta)/dt
    ## find dropping time 
    g = 9.8
    sVert = z_ref - z1
    a = 0.5*g
    b = -uVert
    c = sVert
    print "a: ", a
    print "b: ", b
    print "c: ", c
    tList = [(-b + sqrt(b**2 - 4*a*c))/(2*a), (-b - sqrt(b**2 - 4*a*c))/(2*a)]   
    tDrop = max(tList)
    ## plugin tDrop to get vertical motion
    x1_2D = abs(x_delta)*cos(alpha)
    z1_2D = z1
    ## calculate in 2D projection
    sHorFinal = x1 - uHor * tDrop

    point_ret = Point()
    point_ret.x = sHorFinal*cos(alpha)
    point_ret.y = sHorFinal*sin(alpha)
    point_ret.z = z_ref

    return point_ret

    


def projectile_calc_ver2(pointstamped_0, pointstamped_1, z_ref):
# calculate the final position of the projectile motion based on two given points
# 0 and 1 represent first and second position of points in time
# Cartesian frame based on Sawyer's /base frame
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

    x_delta = x1 - x0
    y_delta = y1 - y0
    z_delta = z1 - z0

    theta = atan(abs(y_delta/x_delta))

    # d_vert = z_ref - z1
    d_vert = z1 - z_ref
    u_vert = z_delta/dt
    d_hor = sqrt(x_delta**2 + y_delta**2)
    u_hor = -1.0*d_hor/dt

    # if u_hor == 0:
    #     u_hor = 0.00001
    a = g/(2*(u_hor**2))
    b = -u_vert/u_hor
    c = d_vert

    s = [(-b + sqrt(b**2 - 4*a*c))/(2*a), (-b - sqrt(b**2 - 4*a*c))/(2*a)]        
    s_max = max(s)

    # Point() of the ball dropping position
    point_ret = Point()
    
    point_ret.x = x1 - s_max*cos(theta)
    if y_delta > 0:
        point_ret.y = y1 + s_max*sin(theta)
    elif y_delta < 0:
        point_ret.y = y1 - s_max*sin(theta)
    point_ret.z = z_ref
    return point_ret

def projectile_calc_ver1(pos, z_ref):
    
    # dt =1.0/hz
    # print pos[0]
    # print pos[1]
    dt = (pos[1].header.stamp) - (pos[0].header.stamp)
    array_size = len(pos)
    max_s_list = [Point() for i in range(array_size - 1)]
    # ret_pos = Point()
    ret_pos = PointStamped()
    
    print "\n\nlog in sawyer_calc.py"
    print "Position old in pos : ", pos[0].point
    print "Position new in pos : ", pos[1].point
    

    for i in range(array_size - 1):
        x_b = (pos[i+1]).point.x 
        x_a = (pos[i]).point.x
        y_b = (pos[i+1]).point.y
        y_a = (pos[i]).point.y
        z_b = (pos[i+1]).point.z 
        z_a = (pos[i]).point.z

        x_delta = x_b - x_a
        y_delta = y_b - y_a
        z_delta = z_b - z_a    

        # theta1 = atan2(y_a,x_a)
        # theta2 = atan2(y_b,x_b)
        # theta = (theta1 + theta2)/2.0
        theta = atan(abs(y_delta/x_delta))

        d_vert = z_ref - z_b
        # d_vert = z_ref - z_a
        u_vert = z_delta/dt
        d_hor = sqrt(x_delta**2 + y_delta**2)
        u_hor = -1.0*d_hor/dt
        if u_hor == 0:
            u_hor = 0.00001
        print "x_delta:", x_delta, " y_delta:", y_delta, " z_delta:", z_delta
        print " theta:", theta
        print "dt:", dt
        print "d_hor:",d_hor, 
        print "u_hor:", u_hor
        a = g/(2*(u_hor**2))
        b = -u_vert/u_hor
        c = d_vert
        print "a:",a, " b:",b, " c:",c
        s = [(-b + sqrt(b**2 - 4*a*c))/(2*a), (-b - sqrt(b**2 - 4*a*c))/(2*a)]        
        s_max = max(s)
        print "s_max:", s_max

        pos_ret = Point()

        pos_ret.x = x_b - s_max*cos(theta)
        if y_delta > 0:
            pos_ret.y = y_b + s_max*sin(theta)
        elif y_delta < 0:
            pos_ret.y = y_b - s_max*sin(theta)
        pos_ret.z = z_ref

        # pos_ret.x = s_max*cos(theta)
        # pos_ret.y = s_max*sin(theta)
        # pos_ret.z = z_ref
        
        max_s_list[i] = pos_ret
        print "(This loop) fin_x:", pos_ret.x, "fin_y:", pos_ret.y
    

    sum_val_x = 0
    sum_val_y = 0
    for i in range(len(max_s_list)):
        sum_val_x =+ max_s_list[i].x
        sum_val_y =+ max_s_list[i].y
    ret_pos.point.x = sum_val_x/len(max_s_list)
    ret_pos.point.y = sum_val_y/len(max_s_list)
    ret_pos.point.z = z_ref
    return ret_pos
    


####################
# AVERAGING RADIUS #
####################

# def ball_pointcloud_filter(data, radius) : 
# # return a matrix with those valuse close to the ball


######################
# POINTCLOUD FILTERS #
######################

def reject_outliers(data, m = 2.):
# take matrix and return the one with no outliers
    # print 'in reject outlier'
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
        print "Error in mean function : Input is null set, [] will be returned"
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
