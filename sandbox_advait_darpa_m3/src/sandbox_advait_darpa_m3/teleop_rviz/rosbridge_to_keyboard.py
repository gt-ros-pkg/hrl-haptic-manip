#!/usr/bin/python

# node to map ROS messages from Phil's interface (Rosbridge) to Marc's
# keyboard teleop interface. The idea is that anything that has the
# interface of the keyboard teleop can also be used directly with
# cart_skin_control

import math

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')
from geometry_msgs.msg import Vector3, TwistStamped

import rospy

def rosbridge_cb(msg):
    dx = msg.twist.linear.x
    dy = msg.twist.linear.y
    dz = msg.twist.linear.z

    # -ve because we feel that phil is sending things in a different
    # way.
    da = msg.twist.angular.x
    db = -msg.twist.angular.y
    dc = -msg.twist.angular.z

    if dx == 0. and dy == 0. and dz == 0.:
        # send angular command
        out_msg = Vector3(da, db, dc)
        rot_pub.publish(out_msg)

    if da == 0. and db == 0. and dc == 0.:
        # send linear command
        out_msg = Vector3(dx, dy, dz)
        pos_pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('rosbridge_to_keyboard')
    pos_pub = rospy.Publisher('/cartesian_skin_control/position_command', Vector3)
    rot_pub = rospy.Publisher('/cartesian_skin_control/rotation_command', Vector3)

    rospy.Subscriber('r_cart/web_commands', TwistStamped, rosbridge_cb)
    rospy.loginfo('rosbridging_to_keyboard')
    rospy.spin()


