import numpy as np
from math import cos, sin, radians, sqrt, atan2, atan
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


# z_ref = final z that the ball will land on. 
# def projectile_calc(pos, z_ref, hz): 
def projectile_calc(pos, z_ref):
    
    # dt =1.0/hz
    dt = pos[1].header.stamp - pos[0].header.stamp
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
    
