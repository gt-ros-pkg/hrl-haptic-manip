#!/usr/bin/env python

import math, numpy as np
import sys

import interactive_marker_util as imu

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy

import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerFeedback, InteractiveMarkerControl
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Bool

# stop current controller and then allow a new controller to execute.
def stop_start_epc():
    # stop current controller
    stop_pub.publish(Bool(True))
    rospy.sleep(0.3)
    # allow controller to start.
    stop_pub.publish(Bool(False))
    rospy.sleep(0.1)

def wp_feedback_rviz_cb(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        pp = feedback.pose.position
        ps = PointStamped()
        ps.header.frame_id = feedback.header.frame_id
        ps.point.x = pp.x
        ps.point.y = pp.y
        ps.point.z = pp.z
        wp_pos_pub.publish(ps)
    server.applyChanges()

def wp_feedback_menu_handler(feedback):
    stop_start_epc()
    ros_pub.publish('reach_to_way_point')

def goal_feedback_rviz_cb(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        pp = feedback.pose.position
        ps = PointStamped()
        ps.header.frame_id = feedback.header.frame_id
        ps.point.x = pp.x
        ps.point.y = pp.y
        ps.point.z = pp.z
        goal_pos_pub.publish(ps)
    server.applyChanges()

def goal_feedback_menu_handler(feedback):
    if feedback.menu_entry_id == 1:
        ros_pub.publish('set_goal')
    elif feedback.menu_entry_id == 2:
        stop_start_epc()
        ros_pub.publish('reach_to_goal')
    elif feedback.menu_entry_id == 3:
        pause_pub.publish(Bool(True))
    elif feedback.menu_entry_id == 4:
        pause_pub.publish(Bool(False))



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--cody', action='store_true', dest='cody',
                 help='task monitoring for cody')
    p.add_option('--sim', action='store_true', dest='sim',
                 help='software simulation')

    opt, args = p.parse_args()

    rospy.init_node('teleop_rviz')
    goal_pos_pub = rospy.Publisher('/teleop_rviz/command/goal_position', PointStamped)
    wp_pos_pub = rospy.Publisher('/teleop_rviz/command/way_point_position', PointStamped)
    ros_pub = rospy.Publisher('/epc_skin/command/behavior', String)
    pause_pub = rospy.Publisher('/epc/pause', Bool)
    stop_pub = rospy.Publisher('/epc/stop', Bool)

    server = ims.InteractiveMarkerServer('teleop_rviz_server')

    pos = np.matrix([0.,0.,0.]).T
    ps = PointStamped()
    ps.header.frame_id = '/torso_lift_link'

    #--- interactive marker for way point ---
    ps.point.x = 0.4
    ps.point.y = -0.1

    if opt.cody:
        ps.point.z = -0.15
        wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif opt.sim:
        ps.point.z = 0.15
        wp_im = imu.make_marker_position_xy(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    else:
        rospy.logerr('Please specify either --cody or --sim')
        sys.exit()

    wp_im.name = 'way_point'
    wp_im.description = 'Way Point'
    server.insert(wp_im, wp_feedback_rviz_cb)
    server.applyChanges()

    wp_menu_handler = mh.MenuHandler()
    wp_menu_handler.insert('Go', callback = wp_feedback_menu_handler)
    imu.add_menu_handler(wp_im, wp_menu_handler, server)

    #--- interactive marker for goal ---
    ps.point.x = 0.5
    ps.point.y = -0.2

    if opt.cody:
        ps.point.z = -0.15
        goal_im = imu.make_3dof_marker_position(ps, 0.15, (0., 1., 1.,0.4), 'sphere')
    else:
        ps.point.z = 0.15
        goal_im = imu.make_marker_position_xy(ps, 0.15, (0., 1., 1.,0.4), 'sphere')

    goal_im.name = 'goal'
    goal_im.description = 'Goal'
    server.insert(goal_im, goal_feedback_rviz_cb)
    server.applyChanges()

    goal_menu_handler = mh.MenuHandler()
    goal_menu_handler.insert('Set', callback = goal_feedback_menu_handler)
    goal_menu_handler.insert('Go', callback = goal_feedback_menu_handler)
    goal_menu_handler.insert('Pause', callback = goal_feedback_menu_handler)
    goal_menu_handler.insert('Resume', callback = goal_feedback_menu_handler)
    imu.add_menu_handler(goal_im, goal_menu_handler, server)


    rospy.loginfo('Interactive marker server started')
    rospy.spin()






