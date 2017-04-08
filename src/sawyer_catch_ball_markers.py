import rospy
from visualization_msgs.msg import Marker 
# more information in https://mirror.umd.edu/roswiki/doc/diamondback/api/visualization_msgs/html/msg/Marker.html

class MarkerDrawer(object):
    def __init__(self, frame_in, namespace, max_id_num=500):
        rospy.loginfo("Creating MarkerDrawer Class")
        self.frame = frame_in # eg. "/map" or "/base"
        self.ns = namespace # Namespace to place this object in... used in conjunction with id to create a unique name for the object
        self.max_id_num = max_id_num
        self.id_num = 1

        # Marker instances
        self.line_strip = Marker()
        self.sphere = Marker()

        # Publishers
        self.mk_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    def delete_line_strips(self):
        self.line_strip.action = 3 #delete all
        self.mk_pub.publish(self.line_strip)
        return

    def draw_line_strips(self, strip_rgba_arr, strip_scale_arr, p0, p1, lifetime=0):
        # draw line strip based on p0 to p1 
        self.line_strip.header.frame_id = self.frame
        self.line_strip.header.stamp = rospy.Time.now()
        self.line_strip.action = Marker.ADD
        if (self.id_num < self.max_id_num):
            self.id_num += 1 
        else:
            self.id_num = 1
        self.line_strip.ns = self.ns + str(self.id_num)
        self.line_strip.type = Marker.LINE_STRIP
        self.line_strip.scale.x = strip_scale_arr[0]
        self.line_strip.scale.y = strip_scale_arr[1]
        self.line_strip.scale.z = strip_scale_arr[2]
        # print "scale: ", self.line_strip.scale
        self.line_strip.color.r = strip_rgba_arr[0]
        self.line_strip.color.g = strip_rgba_arr[1]
        self.line_strip.color.b = strip_rgba_arr[2]
        self.line_strip.color.a = strip_rgba_arr[3]
        # print "scale: ", self.line_strip.color
        self.line_strip.points = [p0,p1]
        # print "points: ", self.line_strip.points
        self.line_strip.lifetime = rospy.Duration.from_sec(lifetime)
        self.mk_pub.publish(self.line_strip)
        rospy.sleep(0.02)
        return
