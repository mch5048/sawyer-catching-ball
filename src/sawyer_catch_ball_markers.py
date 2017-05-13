import rospy
import time
from visualization_msgs.msg import Marker 
# more information in https://mirror.umd.edu/roswiki/doc/diamondback/api/visualization_msgs/html/msg/Marker.html

class MarkerDrawer(object):
    def __init__(self, frame_in, namespace, max_id_num=500):
        rospy.loginfo("Creating MarkerDrawer Class")
        self.frame = frame_in # eg. "/map" or "/base"
        self.ls_ns = namespace + '_ls_' # Namespace to place this object in... used in conjunction with id to create a unique name for the object
        self.sph_ns = namespace + '_sph_'
        self.numtxt_ns = namespace + '_numtxt_'
        self.max_id_num = max_id_num
        self.ls_id_num = 1
        self.sph_id_num = 1
        self.numtxt_id_num = 0

        # Marker instances
        self.line_strip = Marker()
        self.sphere = Marker()
        self.numtxt = Marker()

        # Publishers
        self.mk_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)


    def delete_all_mks(self):
        del_mk = Marker()
        # self.line_strip.action = 3 #delete all
        del_mk.action = 3 #delete all
        self.mk_pub.publish(del_mk)
        return


    def draw_line_strips(self, rgba_arr, scale_arr, p0, p1, lifetime=0):
        # draw line strip based on p0 to p1 
        # t1 = time.time()
        self.line_strip.header.frame_id = self.frame
        self.line_strip.header.stamp = rospy.Time.now()
        self.line_strip.action = Marker.ADD
        if (self.ls_id_num < self.max_id_num):
            self.ls_id_num += 1 
        else:
            self.ls_id_num = 1
        self.line_strip.ns = self.ls_ns + str(self.ls_id_num)
        self.line_strip.type = Marker.LINE_STRIP
        self.line_strip.scale.x = scale_arr[0]
        self.line_strip.scale.y = scale_arr[1]
        self.line_strip.scale.z = scale_arr[2]
        # print "t2_dls: ", time.time() - t1, " ms"
        # t2 = time.time()
        self.line_strip.color.r = rgba_arr[0]
        self.line_strip.color.g = rgba_arr[1]
        self.line_strip.color.b = rgba_arr[2]
        self.line_strip.color.a = rgba_arr[3]
        self.line_strip.points = [p0,p1]
        self.line_strip.lifetime = rospy.Duration.from_sec(lifetime)
        self.mk_pub.publish(self.line_strip)
        # print "t2_dsp: ", time.time() - t2, " ms"
        # tSl = time.time()
        # rospy.sleep(0.01)
        # print "t2Sl_dsp: ", time.time() - tSl, " ms \r\n"
        return


    def draw_spheres(self, rgba_arr, scale_arr, p, lifetime=0):
        # t1 = time.time()
        self.sphere.header.frame_id = self.frame
        self.sphere.header.stamp = rospy.Time.now()
        self.sphere.action = Marker.ADD
        if (self.sph_id_num < self.max_id_num):
            self.sph_id_num += 1 
        else:
            self.sph_id_num = 1
        self.sphere.ns = self.sph_ns + str(self.sph_id_num)
        self.sphere.type = Marker.SPHERE
        self.sphere.scale.x = scale_arr[0]
        self.sphere.scale.y = scale_arr[1]
        self.sphere.scale.z = scale_arr[2]
        # print "t1_dsp: ", time.time() - t1, " ms"
        # t2 = time.time()
        self.sphere.color.r = rgba_arr[0]
        self.sphere.color.g = rgba_arr[1]
        self.sphere.color.b = rgba_arr[2]
        self.sphere.color.a = rgba_arr[3]
        # self.sphere.points = [p]
        self.sphere.pose.position = p
        self.sphere.lifetime = rospy.Duration.from_sec(lifetime)
        self.mk_pub.publish(self.sphere)
        # print "t2_dsp: ", time.time() - t2, " ms"
        # tSl = time.time()
        # rospy.sleep(0.01)
        # print "t1Sl_dsp: ", time.time() - tSl, " ms \r\n"
        return

        
    def draw_numtxts(self, rgba_arr, size, p, floating_height=0, lifetime=0):
        self.numtxt.header.frame_id = self.frame
        self.numtxt.header.stamp = rospy.Time.now()
        self.numtxt.action = Marker.ADD
        if (self.numtxt_id_num < self.max_id_num):
            self.numtxt_id_num += 1 
        else:
            self.numtxt_id_num = 1
        self.numtxt.ns = self.numtxt_ns + str(self.numtxt_id_num)
        self.numtxt.type = Marker.TEXT_VIEW_FACING
        self.numtxt.scale.z = size # size of Z equal ot the height of the uppercase 'A'
        self.numtxt.color.r = rgba_arr[0]
        self.numtxt.color.g = rgba_arr[1]
        self.numtxt.color.b = rgba_arr[2]
        self.numtxt.color.a = rgba_arr[3]
        p.z = p.z + floating_height
        self.numtxt.pose.position = p 
        self.numtxt.text = str(self.numtxt_id_num)
        self.numtxt.lifetime = rospy.Duration.from_sec(lifetime)
        self.mk_pub.publish(self.numtxt)
        # rospy.sleep(0.01)
        return            
