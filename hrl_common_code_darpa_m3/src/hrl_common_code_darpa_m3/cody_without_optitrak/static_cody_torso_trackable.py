#!/usr/bin/python

#
# Use this node to publish TransformedStamped object to the
# cody_torso_trackable/pose topic.
#
# This is a proxy for the optitrak node.
#
# Along with a static transform publisher, this node allows us to run
# Cody without requiring any mocap or visual odometry.
# (see setup_for_control_using_skin.launch in hrl_meka_skin_sensor_darpa_m3)
#
#

import numpy as np, math

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import rospy

from geometry_msgs.msg import TransformStamped


if __name__ == '__main__':
    rospy.init_node('static_cody_torso_trackable_publisher')

    torso_pose_topic = '/cody_torso_trackable/pose'
    torso_pose_pub = rospy.Publisher(torso_pose_topic, TransformStamped)

    rospy.loginfo('Started static_cody_torso_trackable_publisher')

    ts = TransformStamped()
    ts.header.frame_id = '/world'
    ts.child_frame_id = '/torso_lift_link'

    ts.transform.translation.x = 0.
    ts.transform.translation.y = 0.
    ts.transform.translation.z = 0.

    ts.transform.rotation.x = 0.
    ts.transform.rotation.y = 0.
    ts.transform.rotation.z = 0.
    ts.transform.rotation.w = 1.

    rt = rospy.Rate(50)
    while not rospy.is_shutdown():
        ts.header.stamp = rospy.Time.now()
        torso_pose_pub.publish(ts)
        rt.sleep()







