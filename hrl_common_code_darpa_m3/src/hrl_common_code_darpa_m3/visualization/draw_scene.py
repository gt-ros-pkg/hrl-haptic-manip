
#
# common visualization helper code for software simulation.
#

import roslib
import math
roslib.load_manifest('hrl_common_code_darpa_m3')
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

class SceneDraw:
    def __init__(self, topic='sim/viz/bodies', frame='/world'):
        self.pub = rospy.Publisher(topic, Marker)
        self.frame = frame
        self.Marker = Marker

    def pub_body(self, pos, quat, scale, color, num, shape, text = ''):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time.now()

        marker.ns = "basic_shapes"
        marker.id = num

        marker.type = shape
        marker.action = Marker.ADD

        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration()
        marker.text = text
        self.pub.publish(marker)

    def pub_arrow(self, pos1, pos2, color, mag_force):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time.now()

        marker.ns = "basic_shapes"
        marker.id = 99999

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        pt1 = Point()
        pt2 = Point()
        pt1.x = pos1[0,0]
        pt1.y = pos1[1,0]
        pt1.z = pos1[2,0]
        pt2.x = pos2[0,0]
        pt2.y = pos2[1,0]
        pt2.z = pos2[2,0]
        marker.points.append(pt1)
        marker.points.append(pt2)

        marker.scale.x = 0.05
        marker.scale.y = 0.1
        #marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration()
        marker.text = mag_force
        self.pub.publish(marker)


    def get_rot_mat(self, rot):
#        rot_mat = np.matrix([[rot[0], rot[3], rot[6]],[rot[1], rot[4], rot[7]], [rot[2], rot[5], rot[8]]])
        rot_mat = np.matrix([[rot[0], rot[4], rot[8]],[rot[1], rot[5], rot[9]], [rot[2], rot[6], rot[10]]])
        return rot_mat

    def quat_from_axis_angle(self, axis, angle=math.pi/2.0):
        np_axis = np.array(axis)
        unit_np_ax = np_axis / np.linalg.norm(np_axis)
        if angle != 0:
            quat = [math.cos(angle/2.0), math.sin(angle/2.0)*unit_np_ax[0], math.sin(angle/2.0)*unit_np_ax[1],
                    math.sin(angle/2.0)*unit_np_ax[2]]
        else:
            print "angle cannot be zero in quat_from_axis_angle, it isn't defined"
        return quat


