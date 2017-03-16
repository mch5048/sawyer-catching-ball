import numpy as np
from math import cos, sin, radians, sqrt, atan2
from geometry_msgs.msg import Point


g = -9.81

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


# z_ref = final z that the ball will land on. 
def projectile_calc(pos, z_ref, hz):
    
    dt =1.0/hz
    array_size = len(pos)
    max_s_list = [Point() for i in range(array_size - 1)]
    ret_pos = Point()
    
    print pos
    

    for i in range(array_size - 1):
        x_b = (pos[i+1])[0] 
        x_a = (pos[i])[0]
        y_b = (pos[i+1])[1]
        y_a = (pos[i])[1]
        z_b = (pos[i+1])[2] 
        z_a = (pos[i])[2]


        theta1 = atan2(y_a,x_a)
        theta2 = atan2(y_b,x_b)
        theta = (theta1 + theta2)/2.0

        x_delta = x_b - x_a
        y_delta = y_b - y_a
        z_delta = z_b - z_a    

        d_vert = z_ref - z_a
        u_vert = z_delta/dt
        u_hor = 1.0*x_delta/dt
        if u_hor == 0:
            u_hor = 0.00001
        print "x_a : ", x_a
        print "x_b : ", x_b
        print "dt : ", dt
        print "x_delta : ", x_delta
        print "u_hor : ", u_hor
        a = g/(2*(u_hor**2))
        b = -u_vert/u_hor
        c = 1
        s = [(-b + sqrt(b**2 - 4*a*c))/(2*a), (-b - sqrt(b**2 - 4*a*c))/(2*a)]        
        s_max = max(s)

        pos_ret = Point()
        pos_ret.x = s_max*cos(theta)
        pos_ret.y = s_max*sin(theta)
        pos_ret.z = z_ref
        max_s_list[i] = pos_ret
    

    sum_val_x = 0
    sum_val_y = 0
    for i in range(len(max_s_list)):
        sum_val_x =+ max_s_list[i].x
        sum_val_y =+ max_s_list[i].y
    ret_pos.x = sum_val_x/len(max_s_list)
    ret_pos.y = sum_val_y/len(max_s_list)
    ret_pos.z = z_ref
    return ret_pos
    
