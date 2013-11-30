#!/usr/bin/env python

import math, numpy as np
import sys

import interactive_marker_util as imu

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy

import interactive_markers.interactive_marker_server as ims
roslib.load_manifest('hrl_lib')
from hrl_lib import transforms as tr

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerFeedback, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, PointStamped, Vector3, Quaternion
from std_msgs.msg import String, Bool


class CartesianSkinMotion():
    def __init__(self):
        self.cntrl_orientation = False
        self.person_frame = None
        self.server = ims.InteractiveMarkerServer('setup_cartesian_body_frame_server')
        rospy.Subscriber('/cartesian_skin_control/set_orient_cntrl', 
                         Bool,
                         self.toggle_orient_cntrl)
        rospy.Subscriber('/cartesian_skin_control/position_command', 
                         Vector3, 
                         self.move_with_respect_to_person)

        self.rotate_wrt_gripper = False
        if self.rotate_wrt_gripper:
            rospy.Subscriber('/cartesian_skin_control/rotation_command', 
                             Vector3, 
                             self.rotate_with_respect_to_gripper)
        else:
            rospy.Subscriber('/cartesian_skin_control/rotation_command', 
                             Vector3, 
                             self.rotate_with_respect_to_person)
                        
        self.stop_pub = rospy.Publisher('/epc/stop', Bool)
        self.goal_pos_pub = rospy.Publisher('/teleop_rviz/command/delta_goal_position', Vector3)
        self.goal_rot_pub = rospy.Publisher('/teleop_rviz/command/delta_goal_orientation', Quaternion)
        self.ros_pub = rospy.Publisher('/epc_skin/command/behavior', String)

        #initialize the cylinder 
        pos = np.matrix([0.,0.,0.]).T
        ps = PointStamped()
        ps.header.frame_id = '/torso_lift_link'

        #--- interactive marker for body_frame definition
        ps.point.x = 0.8
        ps.point.y = -0.2
        ps.point.z = 0.0

        wp_im = imu.make_6dof_marker(False, ps, 0.15, (1., 1., 0.,0.4), 'cylinder')
        #wp_im = imu.make_3dof_marker_position(ps, 0.24, (1., 1., 0.,0.4), 'sphere')
        wp_im.name = 'body_frame'
        wp_im.description = 'body frame for person'
        self.server.insert(wp_im, self.body_frame_feedback_rviz_cb)
        self.server.applyChanges()

    def stop_start_epc(self):
        # stop current controller
        self.stop_pub.publish(Bool(True))
        rospy.sleep(0.3)
        # allow controller to start.
        self.stop_pub.publish(Bool(False))
        rospy.sleep(0.1)

    def toggle_orient_cntrl(self, msg):
        self.cntrl_orientation = msg.data
        rospy.loginfo('Switching cntrl_orientation to :'+str(msg.data))

    def rotate_with_respect_to_person(self, msg):
        quat_0 = [self.person_frame.orientation.x, 
                  self.person_frame.orientation.y, 
                  self.person_frame.orientation.z, 
                  self.person_frame.orientation.w] 

        if abs(math.degrees(msg.y)) > 85.0 and abs(math.degrees(msg.y)) < 95.0:
            if abs(math.degrees(msg.y)) <= 90.0:
                msg.y = math.radians(85)*np.sign(msg.y)
            else:
                msg.y = math.radians(95)*np.sign(msg.y)

        #can replace these three lines with a single quaternion inverse and multiply #########
        #however, i haven't had time to test so I'm being careful ############################

        rot_0 = tr.quaternion_to_matrix(quat_0)
        rot_delt_p_frame = tr.getRotSubMat(tr.tft.euler_matrix(msg.x, msg.y, msg.z))
        rot_delt_t_frame = rot_0*rot_delt_p_frame*rot_0.T
        #rot_delta = rot_0*tr.getRotSubMat(tr.tft.euler_matrix(msg.x, msg.y, msg.z))
        
        quat_delta = tr.matrix_to_quaternion(rot_delt_t_frame)

        self.goal_rot_pub.publish(Quaternion(quat_delta[0], 
                                             quat_delta[1],
                                             quat_delta[2], 
                                             quat_delta[3]))
        self.stop_start_epc()
        self.ros_pub.publish('orient_to_way_point')

    def rotate_with_respect_to_gripper(self, msg):
        rot_delta = tr.getRotSubMat(tr.tft.euler_matrix(msg.x, msg.y, msg.z))
        quat_delta = tr.matrix_to_quaternion(rot_delta)
        self.goal_rot_pub.publish(Quaternion(quat_delta[0], 
                                             quat_delta[1],
                                             quat_delta[2], 
                                             quat_delta[3]))
        self.stop_start_epc()
        self.ros_pub.publish('orient_to_way_point')

    def move_with_respect_to_person(self, msg):
        quat_0 = [self.person_frame.orientation.x, 
                  self.person_frame.orientation.y, 
                  self.person_frame.orientation.z, 
                  self.person_frame.orientation.w] 
        rot_0 = tr.quaternion_to_matrix(quat_0)
        pos_1 = np.matrix([msg.x, msg.y, msg.z]).reshape(3,1)
        
        delta_pos = (rot_0*pos_1).A1.tolist()
        
        self.goal_pos_pub.publish(Vector3(delta_pos[0], 
                                          delta_pos[1], 
                                          delta_pos[2]))

        self.stop_start_epc()
        if self.cntrl_orientation == False:
            self.ros_pub.publish('go_to_way_point')
        else:
            self.ros_pub.publish('orient_to_way_point')


    def body_frame_feedback_rviz_cb(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.person_frame = feedback.pose
        self.server.applyChanges()

if __name__ == '__main__':

    rospy.init_node('cartesian_control_with_skin')

    cart_skin_motion = CartesianSkinMotion()

    rospy.loginfo('Cartesian skin control started')

    rospy.spin()



