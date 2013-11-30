#!/usr/bin/env python

import math, numpy as np
import sys

import interactive_marker_util as imu

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy

import hrl_lib.transforms as tr

import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerFeedback, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Bool, Empty

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
        ps = PoseStamped()
        ps.header.frame_id = feedback.header.frame_id

        pp = feedback.pose.position
        qq = feedback.pose.orientation

        quat = [qq.x, qq.y, qq.z, qq.w]
        r = tr.quaternion_to_matrix(quat)
        offset = np.matrix([0.0, 0., 0.]).T
        o_torso = r * offset

        ps.pose.position.x = pp.x + o_torso[0,0]
        ps.pose.position.y = pp.y + o_torso[1,0]
        ps.pose.position.z = pp.z + o_torso[2,0]

        ps.pose.orientation.x = qq.x
        ps.pose.orientation.y = qq.y
        ps.pose.orientation.z = qq.z
        ps.pose.orientation.w = qq.w

        wp_pose_pub.publish(ps)
    server.applyChanges()

def wp_feedback_go_handler(feedback):
    stop_start_epc()
    ros_pub.publish('go_to_way_point')

def wp_feedback_reduce_forces(feedback):
    stop_start_epc()
    ros_pub.publish('reduce_force')

def wp_feedback_start_darpa_trial(feedback):
    ros_pub.publish('go_darpa_goals')

def wp_feedback_orient_handler(feedback):
    stop_start_epc()
    ros_pub.publish('orient_to_way_point')

def wp_feedback_stop_handler(feedback):
    stop_start_epc()

def wp_feedback_open_handler(feedback):
    open_pub.publish(Empty())

def wp_feedback_close_handler(feedback):
    close_pub.publish(Empty())

def wp_feedback_disable_handler(feedback):
    disable_pub.publish(Empty())

def wp_feedback_enable_handler(feedback):
    enable_pub.publish(Empty())

def wp_feedback_zero_handler(feedback):
    zero_gripper_pub.publish(Empty())
    zero_gripper_right_link_pub.publish(Empty())
    zero_gripper_left_link_pub.publish(Empty())
    zero_gripper_palm_pub.publish(Empty())
    zero_forearm_pub.publish(Empty())
    zero_upperarm_pub.publish(Empty())
    zero_pps_pub.publish(Empty())

def goal_feedback_rviz_cb(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        ps = PoseStamped()
        ps.header.frame_id = feedback.header.frame_id

        pp = feedback.pose.position
        ps.pose.position.x = pp.x
        ps.pose.position.y = pp.y
        ps.pose.position.z = pp.z

        qq = feedback.pose.orientation
        ps.pose.orientation.x = qq.x
        ps.pose.orientation.y = qq.y
        ps.pose.orientation.z = qq.z
        ps.pose.orientation.w = qq.w

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
                 help='interactive markers for cody')
    p.add_option('--pr2', action='store_true', dest='pr2',
                 help='interactive markers for the PR2')
    p.add_option('--sim', action='store_true', dest='sim',
                 help='software simulation')
    p.add_option('--orientation', action='store_true', dest='orientation',
                 help='command orientation as well')

    opt, args = p.parse_args()

    rospy.init_node('teleop_rviz')
    goal_pos_pub = rospy.Publisher('/teleop_rviz/command/goal_position', PoseStamped)
    wp_pose_pub = rospy.Publisher('/teleop_rviz/command/way_point_pose', PoseStamped)
    ros_pub = rospy.Publisher('/epc_skin/command/behavior', String)
    pause_pub = rospy.Publisher('/epc/pause', Bool)
    stop_pub = rospy.Publisher('/epc/stop', Bool)

    open_pub = rospy.Publisher('open_gripper', Empty)
    close_pub = rospy.Publisher('close_gripper', Empty)

    disable_pub = rospy.Publisher('/pr2_fabric_gripper_sensor/disable_sensor', Empty)
    enable_pub = rospy.Publisher('/pr2_fabric_gripper_sensor/enable_sensor', Empty)

    zero_gripper_pub = rospy.Publisher('/pr2_fabric_gripper_sensor/zero_sensor', Empty)
    zero_gripper_left_link_pub = rospy.Publisher('/pr2_fabric_gripper_left_link_sensor/zero_sensor', Empty)
    zero_gripper_right_link_pub = rospy.Publisher('/pr2_fabric_gripper_right_link_sensor/zero_sensor', Empty)
    zero_gripper_palm_pub = rospy.Publisher('/pr2_fabric_gripper_palm_sensor/zero_sensor', Empty)
    zero_forearm_pub = rospy.Publisher('/pr2_fabric_forearm_sensor/zero_sensor', Empty)
    zero_upperarm_pub = rospy.Publisher('/pr2_fabric_upperarm_sensor/zero_sensor', Empty)
    zero_pps_pub = rospy.Publisher('/pr2_pps_sensor/zero_sensor', Empty)

    server = ims.InteractiveMarkerServer('teleop_rviz_server')

    pos = np.matrix([0.,0.,0.]).T
    ps = PointStamped()
    ps.header.frame_id = '/torso_lift_link'

    #--- interactive marker for way point ---
    if opt.cody:
        ps.point.x = 0.4
        ps.point.y = -0.1
        ps.point.z = -0.15
        if opt.orientation:
            wp_im = imu.make_6dof_gripper(False, ps, 0.28, (1., 1., 0.,0.4), "cody")
            #wp_im = imu.make_6dof_marker(False, ps, 0.15, (1., 1., 0.,0.4), 'sphere')
        else:
            wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif opt.pr2:
        ps.point.x = 0.6
        ps.point.y = -0.1
        ps.point.z = -0.15
        if opt.orientation:
            #wp_im = imu.make_6dof_marker(False, ps, 0.15, (1., 1., 0.,0.4), 'sphere')
            wp_im = imu.make_6dof_gripper(False, ps, 0.28, (1., 1., 0.,0.4))
        else:
            wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif opt.sim:
        ps.point.x = 0.4
        ps.point.y = -0.1
        ps.point.z = 0.15
        wp_im = imu.make_marker_position_xy(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    else:
        rospy.logerr('Please specify a testbed')
        sys.exit()

    wp_im.name = 'way_point'
    wp_im.description = 'Way Point'
    server.insert(wp_im, wp_feedback_rviz_cb)
    server.applyChanges()

    wp_menu_handler = mh.MenuHandler()
#    wp_menu_handler.insert('', callback = wp_feedback_go_handler)
    wp_menu_handler.insert('Go', callback = wp_feedback_go_handler)
#    wp_menu_handler.insert('_____________', callback = wp_feedback_go_handler)
#    wp_menu_handler.insert('', callback = wp_feedback_orient_handler)
    wp_menu_handler.insert('Orient', callback = wp_feedback_orient_handler)
#    wp_menu_handler.insert('_____________', callback = wp_feedback_orient_handler)
#    wp_menu_handler.insert('', callback = wp_feedback_stop_handler)
    wp_menu_handler.insert('Stop', callback = wp_feedback_stop_handler)
#    wp_menu_handler.insert('_____________', callback = wp_feedback_stop_handler)
#    wp_menu_handler.insert('', callback = wp_feedback_open_handler)
    wp_menu_handler.insert('Open Gripper', callback = wp_feedback_open_handler)
#    wp_menu_handler.insert('_____________', callback = wp_feedback_open_handler)
#    wp_menu_handler.insert('', callback = wp_feedback_close_handler)
    wp_menu_handler.insert('Close Gripper', callback = wp_feedback_close_handler)
#    wp_menu_handler.insert('_____________', callback = wp_feedback_close_handler)
#    wp_menu_handler.insert('Disable Gripper Sensor', callback = wp_feedback_disable_handler)
#    wp_menu_handler.insert('Enable Gripper Sensor', callback = wp_feedback_enable_handler)
#    wp_menu_handler.insert('', callback = wp_feedback_zero_handler)
    wp_menu_handler.insert('Zero Skin', callback = wp_feedback_zero_handler)
#    wp_menu_handler.insert('', callback = wp_feedback_zero_handler)
    wp_menu_handler.insert('Reduce Forces', callback = wp_feedback_reduce_forces)
#    wp_menu_handler.insert('Run Darpa Trials', callback = wp_feedback_start_darpa_trial)
    imu.add_menu_handler(wp_im, wp_menu_handler, server)

#    #--- interactive marker for goal ---
#    if opt.cody:
#        ps.point.x = 0.5
#        ps.point.y = -0.2
#        ps.point.z = -0.15
#        if opt.orientation:
#            goal_im = imu.make_6dof_marker(False, ps, 0.15, (0., 1., 1.,0.4), 'sphere')
#        else:
#            goal_im = imu.make_3dof_marker_position(ps, 0.15, (0., 1., 1.,0.4), 'sphere')
#    elif opt.pr2:
#        ps.point.x = 0.7
#        ps.point.y = -0.2
#        ps.point.z = -0.15
#        if opt.orientation:
#            goal_im = imu.make_6dof_marker(False, ps, 0.15, (0., 1., 1.,0.4), 'sphere')
#        else:
#            goal_im = imu.make_3dof_marker_position(ps, 0.15, (0., 1., 1.,0.4), 'sphere')
#    elif opt.sim:
#        ps.point.x = 0.5
#        ps.point.y = -0.2
#        ps.point.z = 0.15
#        goal_im = imu.make_marker_position_xy(ps, 0.15, (0., 1., 1.,0.4), 'sphere')
#
#    goal_im.name = 'goal'
#    goal_im.description = 'Goal'
#    server.insert(goal_im, goal_feedback_rviz_cb)
#    server.applyChanges()
#
#    goal_menu_handler = mh.MenuHandler()
#    goal_menu_handler.insert('Set', callback = goal_feedback_menu_handler)
#    goal_menu_handler.insert('Go', callback = goal_feedback_menu_handler)
#    goal_menu_handler.insert('Pause', callback = goal_feedback_menu_handler)
#    goal_menu_handler.insert('Resume', callback = goal_feedback_menu_handler)
#    imu.add_menu_handler(goal_im, goal_menu_handler, server)

    rospy.loginfo('Interactive marker server started')
    rospy.spin()



