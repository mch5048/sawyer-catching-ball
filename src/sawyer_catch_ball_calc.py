import numpy as np
from math import cos, sin, radians, sqrt, atan2, atan, tan
from geometry_msgs.msg import Point, PointStamped
from scipy.optimize import leastsq, minimize
import time



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
    t0 = pointstamped_0.header.stamp
    t1 = pointstamped_1.header.stamp


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


def f_proj_2(lis, t):
# array[Xo, Yo, Zo, Vhor, Vvert, alpha]
    ap = lis[5]
    Vhor = lis[3]
    return np.array([lis[0] + Vhor*cos(ap)*t \
                     ,lis[1] + Vhor*sin(ap)*t \
                     ,lis[2] + lis[4]*t - 9.81/2*(t**2)])

def f_proj(lis, t):
# tpl(Xo, Yo, Zo, Vo, alpha,  theta)
    Vo = lis[3]
    alpha = lis[4]
    c_theta = cos(lis[5])
    return np.array([lis[0] + Vo*cos(alpha)*c_theta*t \
                     ,lis[1] + Vo*sin(alpha)*c_theta*t \
                    ,lis[2] + Vo*sin(lis[5])*t - 9.81/2*(t**2)])

def f_proj_1(t,lis):
    # tpl(Xo, Yo, Zo, Vo, alpha,  theta)
    Vo = lis[3]
    alpha = lis[4]
    c_theta = cos(lis[5])
    return np.array([lis[0] + Vo*cos(alpha)*c_theta*t \
                     ,lis[1] + Vo*sin(alpha)*c_theta*t \
                    ,lis[2] + Vo*sin(lis[5])*t - 9.81/2*(t**2)])

def opt_min_proj_calc_1(ps_xyzt, init_guess, z_ref):
### Optimize
    # list_init = np.zeros(6)
    def cost_fnc_proj(ls):
        # a = time.time()
        b = sum(np.linalg.norm(f_proj_1(ps_xyzt[:,3],ls).T - ps_xyzt[:,:3], axis = 1))
        # print "time in opt: ", time.time() - a, "\r\n"
        return b
    # g = time.time()
    result = minimize(cost_fnc_proj, init_guess)
    # print "time min: ",time.time() - g
    coeff = np.array([-9.81/2, result.x[3]*sin(result.x[5]), result.x[2] - z_ref])
    tFin = np.amax(np.roots(coeff))
    Fin = f_proj_1(tFin, result.x)
    point_ret = Point()
    point_ret.x = Fin[0]
    point_ret.y = Fin[1]
    point_ret.z = Fin[2]
    return point_ret

def opt_min_get_init_guess(ps_xyzt):
### Optimize
    list_init = np.zeros(6)
    def cost_fnc_proj(ls):
        # a = time.time()
        b = sum(np.linalg.norm(f_proj_1(ps_xyzt[:,3],ls).T - ps_xyzt[:,:3], axis = 1))
        # print "time in opt: ", time.time() - a, "\r\n"
        return b
    # g = time.time()
    result = minimize(cost_fnc_proj, list_init)
    return result.x

def f_proj_2D(t,lis):
    # tpl(Lo, Zo, Vo, theta)
    Vo = lis[2]
    theta = lis[3]
    return np.array([lis[0] + Vo*cos(theta)*t \
                     ,lis[1] + Vo*sin(theta)*t - 9.81/2*(t**2)])


def opt_min_2Dproj_get_init_guess(ps_xyzt):
    # find alpha, the angle for projecting point to plane
    ps_x = ps_xyzt[:,0]
    ps_y = ps_xyzt[:,1]
    ps_z = ps_xyzt[:,2]
    s = np.array([atan2((ps_y[i+1] - ps_y[i]),(ps_x[i+1] - ps_x[i])) for i in range(ps_xyzt.shape[0] - 2)])
    for i in range(s.shape[0]):
        # print "i: ", s[i]
        if s[i] < 0:
            # print "i < than zero"
            s[i] = s[i] + (2*np.pi)
    alpha = np.mean(s) 
    # project into 2D plane
    list_init = np.zeros(4)
    def cost_fnc_proj(ls):
        ps_lz = np.array([ps_x/cos(alpha), ps_z]).T        
        b = sum(np.linalg.norm(f_proj_2D(ps_xyzt[:,3],ls).T - ps_lz, axis = 1))       
        return b
    result = minimize(cost_fnc_proj, list_init)
    return result.x

def opt_min_2Dproj_calc_2(ps_xyzt, init_guess, z_ref):
    # find alpha, the angle for projecting point to plane
    ps_x = ps_xyzt[:,0]
    ps_y = ps_xyzt[:,1]
    ps_z = ps_xyzt[:,2]
    s = np.array([atan((ps_y[i+1] - ps_y[i])/(ps_x[i+1] - ps_x[i])) for i in range(ps_xyzt.shape[0] - 2)])
    print "atan2 array: ", s
    alpha = np.mean(s) 
    print "alpha: ", alpha
    # project into 2D plane
    # list_init = np.zeros(4)
    def cost_fnc_proj(ls):
        ps_lz = np.array([ps_x/cos(alpha), ps_z]).T        
        b = sum(np.linalg.norm(f_proj_2D(ps_xyzt[:,3],ls).T - ps_lz, axis = 1))       
        return b
    result = minimize(cost_fnc_proj, init_guess)
    coeff = np.array([-9.81/2, result.x[2]*sin(result.x[3]), result.x[1] - z_ref])
    tFin = np.amax(np.roots(coeff))
    Fin = f_proj_2D(tFin, result.x)
    #find final position
    point_ret = Point()
    point_ret.x = Fin[0]*cos(alpha)
    point_ret.y = Fin[0]*sin(alpha)
    point_ret.z = Fin[1]
    return point_ret


def opt_min_2Dproj_calc(ps_xyzt, init_guess, z_ref):
    # find alpha, the angle for projecting point to plane
    ps_x = ps_xyzt[:,0]
    ps_y = ps_xyzt[:,1]
    ps_z = ps_xyzt[:,2]
    s = np.array([atan2((ps_y[i+1] - ps_y[i]),(ps_x[i+1] - ps_x[i])) for i in range(ps_xyzt.shape[0] - 2)])
    print "atan2 array B4: ", s
    for i in range(s.shape[0]):
        # print "i: ", s[i]
        if s[i] < 0:
            # print "i < than zero"
            s[i] = s[i] + (2*np.pi)
    print "atan2 array AT: ", s
    alpha = np.mean(s)
    print "alpha B4: ", alpha
    if alpha > np.pi:
        alpha -= (2*np.pi)
    print "alpha AT: ", alpha
    # project into 2D plane
    # list_init = np.zeros(4)
    def cost_fnc_proj(ls):
        ps_lz = np.array([ps_x/cos(alpha), ps_z]).T        
        b = sum(np.linalg.norm(f_proj_2D(ps_xyzt[:,3],ls).T - ps_lz, axis = 1))       
        return b
    result = minimize(cost_fnc_proj, init_guess)
    coeff = np.array([-9.81/2, result.x[2]*sin(result.x[3]), result.x[1] - z_ref])
    tFin = np.amax(np.roots(coeff))
    Fin = f_proj_2D(tFin, result.x)
    print "Fin: ", Fin
    #find final position
    point_ret = Point()
    point_ret.x = Fin[0]*cos(alpha)
    point_ret.y = Fin[0]*sin(alpha)
    print "Fin[0]*cos(alpha): ", Fin[0]*cos(alpha)
    print "Fin[0]*sin(alpha): ", Fin[0]*sin(alpha), "\r\n#################\r\n\r\n"
    point_ret.z = Fin[1]
    return point_ret


def opt_min_proj_calc(ps_list, z_ref):
    list_init = np.zeros(6)
    tmin = ps_list[0].header.stamp
    def cost_fnc_proj(ls):
        tmin = ps_list[0].header.stamp
        psxyzt = np.array([[ps.point.x, ps.point.y, ps.point.z, ps.header.stamp - tmin] for ps in ps_list])
        return np.sum(np.linalg.norm(f_proj_1(psxyzt[:,3],ls).T - psxyzt[:,:3]))
    result = minimize(cost_fnc_proj, list_init)
    coeff = np.array([-9.81/2, result.x[3]*sin(result.x[5]), result.x[2] - z_ref])
    tFin = np.amax(np.roots(coeff))
    Fin = f_proj(result.x, tFin)
    point_ret = Point()
    point_ret.x = Fin[0]
    point_ret.y = Fin[1]
    point_ret.z = Fin[2]
    return point_ret



def opt_min_proj_calc_old(ps_list, z_ref):
    list_init = np.zeros(6)
    tmin = ps_list[0].header.stamp
    cost_func=lambda ls: sum([np.linalg.norm(f_proj(ls, (ps.header.stamp-tmin))-[ps.point.x, ps.point.y, ps.point.z]) for ps in ps_list])
    result = minimize(cost_func, list_init)
    coeff = np.array([-9.81/2, result.x[3]*sin(result.x[5]), result.x[2] - z_ref])
    tFin = np.amax(np.roots(coeff))
    Fin = f_proj(result.x, tFin)
    point_ret = Point()
    point_ret.x = Fin[0]
    point_ret.y = Fin[1]
    point_ret.z = Fin[2]
    return point_ret

def least_square_proj_calc(psList, z_ref):
    
    print "####Check ps list####"
    print "psList: ", psList
    print "#######"

    xList = np.array([a.point.x for a in psList])
    yList = np.array([a.point.y for a in psList])
    zList = np.array([a.point.z for a in psList])    

    # here, create lambda functions for Line, Quadratic fit
    # tpl is a tuple that contains the parameters of the fit
    funcQuad_xz=lambda tpl,x : tpl[0]*x**2+tpl[1]*x+tpl[2]
    ErrorFunc_xz=lambda tpl,x,z: funcQuad_xz(tpl,x)-z
    tplInitialxz=(1.0,2.0,3.0)
    tplFinalxz,success=leastsq(ErrorFunc_xz,tplInitialxz[:],args=(xList,zList))
    
    funcQuad_yz=lambda tpl,y : tpl[0]*y**2+tpl[1]*y+tpl[2]
    ErrorFunc_yz=lambda tpl,y,z: funcQuad_yz(tpl,y)-z
    tplInitialyz=(1.0,2.0,3.0)
    tplFinalyz,success=leastsq(ErrorFunc_yz,tplInitialyz[:],args=(yList,zList))
    
    xx1=np.linspace(xList.min(),xList.max(),50)
    yy1=np.linspace(yList.min(),yList.max(),50)
    zz1=funcQuad_xz(tplFinalxz,xx1)
    zz2=funcQuad_yz(tplFinalyz,yy1)
    #------------------------------------------------
    # now the quadratic fit
    #-------------------------------------------------
    a = tplFinalxz[0]
    b = tplFinalxz[1]
    c = tplFinalxz[2] - z_ref
   
    xList = [(-b + sqrt(b**2 - 4*a*c))/(2*a), (-b - sqrt(b**2 - 4*a*c))/(2*a)]  

    a = tplFinalyz[0]
    b = tplFinalyz[1]
    c = tplFinalyz[2] - z_ref

    yList = [(-b + sqrt(b**2 - 4*a*c))/(2*a), (-b - sqrt(b**2 - 4*a*c))/(2*a)]  

    xFin = min(xList)
    yFin = min(yList)

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

def reject_outliers(data, m = 1.):
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

def reject_outliers_1(data, m=2.):
    mu = np.mean(data)
    sigma = np.std(data)
    f = 0.5
    return within_1(data, mu-sigma*f, mu+sigma*f)


def reject_outliers_nate(data):
   # take matrix and return the one with no outliers
   if data != []:
       rball = 0.03 # 3 cm in meters (estimate - Chainatee measure this)
       m = min(data)
       return [z for z in data if abs(z-m) < rball]
   else:
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


def within_1(data, min_range, max_range):
    return [x for x in data if x < max_range and x >= min_range]

    
def filter_nan(data):
# filter NaN data in matrix
    # print "data in filternan:", data
    data_len = len(data)
    dat_ret = []
    for i in range(data_len):
        if not np.isnan(data[i]):
            # np.append(dat_ret, data[i])
            dat_ret.append(data[i])            
    return dat_ret

def filter_nan_1(data):
    return data[np.logical_not(np.isnan(data))]

def is_within_range(num, low, high):
# check if the number is within specified range or not
    if num < high and num > low:
        return True
    else:
        return False
