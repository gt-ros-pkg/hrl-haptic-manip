#!/usr/bin/env python

# Software License Agreement (New BSD License)
#
# Copyright (c) 2014, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of Georgia Tech nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE GEORGIA TECH RESEARCH CORPORATION BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Kevin Chow, Jeff Hawke
# Healthcare Robotics Laboratory

import math, numpy as np
import sys, optparse

import copy
import interactive_marker_util as imu

import roslib; roslib.load_manifest('LMPC')
import rospy

import hrl_lib.transforms as tr
import haptic_mpc_util
import LMPC_msgs.msg as haptic_msgs

import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerFeedback, InteractiveMarkerControl, MarkerArray
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray, Point, Quaternion, Pose
from std_msgs.msg import String, Bool, Empty, ColorRGBA
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2GripperCommand, JointTrajectoryControllerState as JTCS
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionIKRequest
from kinematics_msgs.msg import PositionIKRequest
from sttr_msgs.msg import GoalPose, CronaState, RagdollObjectArray, WrenchArray

from sttr_behaviors.crona2_behaviors import crona2_lower_down
from sttr_behaviors.crona2_behaviors import crona2_move_towards_object
from sttr_behaviors.crona2_behaviors import crona2_raise_arm
from sttr_behaviors.crona2_behaviors import crona2_move_and_scoop
from sttr_behaviors.crona2_behaviors import crona2_return_home
from sttr_behaviors.crona2_behaviors import crona2_release_object

## RViz teleop interface to the controller. 
#
# Publishes goal poses on the appropriate topics and allows a user to teleop the arm controller.
class MPCTeleopInteractiveMarkers():
  def __init__(self):
    base_path = 'haptic_mpc'
    control_path = '/control_params'
    #self.orient_weight = rospy.get_param(base_path + control_path + '/orientation_weight')
    #self.pos_weight = rospy.get_param(base_path + control_path + '/position_weight')
    #self.posture_weight = 1.0#rospy.get_param(base_path + control_path + '/posture_weight')    
    self.init_poses_received = False
    self.continuous_update = False
    self.gp = GoalPose()
    self.lower_down = crona2_lower_down.cRoNA2LowerDown()
    self.move_towards_object = crona2_move_towards_object.cRoNA2MoveTowardsObject()
    self.raise_arm = crona2_raise_arm.cRoNA2RaiseArm()
    self.move_and_scoop = crona2_move_and_scoop.cRoNA2MoveAndScoop()
    self.return_home = crona2_return_home.cRoNA2ReturnHome()
    self.release_object = crona2_release_object.cRoNA2ReleaseObject()
    self.ragdoll_m1_position_marked = 0
    self.ragdoll_m2_position_marked = 0
    self.ragdoll_m3_position_marked = 0
    self.lforearm_pose_marked = 0
    self.rforearm_pose_marked = 0
    self.platform_1_position_marked = 0
    self.platform_2_position_marked = 0
    self.platform_3_position_marked = 0
    self.platform_4_position_marked = 0
    self.larm_force = [0,0,0]
    self.rarm_force = [0,0,0]

    self.ragdoll_random_wrench_frame = 'lower_body'
    self.ragdoll_random_wrench_force_x = 0.1
    self.ragdoll_random_wrench_force_y = 0.1
    self.ragdoll_random_wrench_force_z = 0.1

    self.ragdoll_m1_pose = Pose()
    self.ragdoll_m2_pose = Pose()
    self.ragdoll_m3_pose = Pose()
    self.ragdoll_lower_body_pose = Pose()
    self.ragdoll_l_thigh_pose = Pose()
    self.ragdoll_l_shin_pose = Pose()
    self.ragdoll_l_foot_pose = Pose()
    self.ragdoll_r_thigh_pose = Pose()
    self.ragdoll_r_shin_pose = Pose()
    self.ragdoll_r_foot_pose = Pose()
    self.ragdoll_middle_body_pose = Pose()
    self.ragdoll_upper_body_pose = Pose()
    self.ragdoll_l_arm_pose = Pose()
    self.ragdoll_l_wrist_pose = Pose()
    self.ragdoll_l_hand_pose = Pose()
    self.ragdoll_neck_pose = Pose()
    self.ragdoll_head_pose = Pose()
    self.ragdoll_r_arm_pose = Pose()
    self.ragdoll_r_wrist_pose = Pose()
    self.ragdoll_r_hand_pose = Pose()

    self.ragdoll_random_wrench_marker = Marker()
    self.ragdoll_random_wrench_text_marker = Marker()
    self.ragdoll_random_wrench_text_marker2 = Marker()

    self.larm_force_markers = MarkerArray()
    larm_force_marker_x = Marker()
    larm_force_marker_y = Marker()
    larm_force_marker_z = Marker()
    larm_force_marker_text_x = Marker()
    larm_force_marker_text_y = Marker()
    larm_force_marker_text_z = Marker()
    self.rarm_force_markers = MarkerArray()
    rarm_force_marker_x = Marker()
    rarm_force_marker_y = Marker()
    rarm_force_marker_z = Marker()
    rarm_force_marker_text_x = Marker()
    rarm_force_marker_text_y = Marker()
    rarm_force_marker_text_z = Marker()
    self.platform_marker = Marker()
    self.ragdoll_markers = MarkerArray()
    ragdoll_m1_marker = Marker()
    ragdoll_m2_marker = Marker()
    ragdoll_m3_marker = Marker()
    ragdoll_lower_body_marker = Marker()
    ragdoll_l_thigh_marker = Marker()
    ragdoll_l_shin_marker = Marker()
    ragdoll_l_foot_marker = Marker()
    ragdoll_r_thigh_marker = Marker()
    ragdoll_r_shin_marker = Marker()
    ragdoll_r_foot_marker = Marker()
    ragdoll_middle_body_marker = Marker()
    ragdoll_upper_body_marker = Marker()
    ragdoll_l_arm_marker = Marker()
    ragdoll_l_wrist_marker = Marker()
    ragdoll_l_hand_marker = Marker()
    ragdoll_neck_marker = Marker()
    ragdoll_head_marker = Marker()
    ragdoll_r_arm_marker = Marker()
    ragdoll_r_wrist_marker = Marker()
    ragdoll_r_hand_marker = Marker()

    self.ragdoll_random_wrench_marker.header.frame_id = 'world_link'
    self.ragdoll_random_wrench_marker.type = 2
    self.ragdoll_random_wrench_marker.action = 0
    self.ragdoll_random_wrench_marker.color.r = 1
    self.ragdoll_random_wrench_marker.color.b = 1
    self.ragdoll_random_wrench_marker.color.a = 0.5
    self.ragdoll_random_wrench_marker.pose.orientation.x = 0
    self.ragdoll_random_wrench_marker.pose.orientation.y = 0
    self.ragdoll_random_wrench_marker.pose.orientation.z = 0
    self.ragdoll_random_wrench_marker.pose.orientation.w = 1
    self.ragdoll_random_wrench_marker.scale.x = 0.2 
    self.ragdoll_random_wrench_marker.scale.y = 0.2 
    self.ragdoll_random_wrench_marker.scale.z = 0.2 
    self.ragdoll_random_wrench_text_marker.header.frame_id = 'world_link'
    self.ragdoll_random_wrench_text_marker.type = 9
    self.ragdoll_random_wrench_text_marker.action = 0
    self.ragdoll_random_wrench_text_marker.color.r = 1
    self.ragdoll_random_wrench_text_marker.color.b = 1
    self.ragdoll_random_wrench_text_marker.color.a = 0.5
    self.ragdoll_random_wrench_text_marker.pose.orientation.x = 0
    self.ragdoll_random_wrench_text_marker.pose.orientation.y = 0
    self.ragdoll_random_wrench_text_marker.pose.orientation.z = 0.707
    self.ragdoll_random_wrench_text_marker.pose.orientation.w = 0.707
    self.ragdoll_random_wrench_text_marker2.header.frame_id = 'world_link'
    self.ragdoll_random_wrench_text_marker2.type = 9
    self.ragdoll_random_wrench_text_marker2.action = 0
    self.ragdoll_random_wrench_text_marker2.color.r = 1
    self.ragdoll_random_wrench_text_marker2.color.b = 1
    self.ragdoll_random_wrench_text_marker2.color.a = 0.5
    self.ragdoll_random_wrench_text_marker2.pose.orientation.x = 0
    self.ragdoll_random_wrench_text_marker2.pose.orientation.y = 0
    self.ragdoll_random_wrench_text_marker2.pose.orientation.z = 0.707
    self.ragdoll_random_wrench_text_marker2.pose.orientation.w = 0.707

    self.platform_marker.header.frame_id = 'world_link'
    self.platform_marker.type = 1
    self.platform_marker.action = 0
    self.platform_marker.color.r = 1
    self.platform_marker.color.a = 1
    self.platform_marker.pose.position.x = 0
    self.platform_marker.pose.position.y = 2.
    self.platform_marker.pose.position.z = 0.1
    self.platform_marker.pose.orientation.x = 0.
    self.platform_marker.pose.orientation.y = 0.
    self.platform_marker.pose.orientation.z = 0.707
    self.platform_marker.pose.orientation.w = 0.707
    self.platform_marker.scale.x = 1.0
    self.platform_marker.scale.y = 2.5
    self.platform_marker.scale.z = 0.1

    ragdoll_m1_marker.header.frame_id = 'world_link'
    ragdoll_m1_marker.type = 2
    ragdoll_m1_marker.ns = 'ragdoll_m1'
    ragdoll_m1_marker.action = 0
    ragdoll_m1_marker.color.r = 1
    ragdoll_m1_marker.color.a = 0.5
    ragdoll_m1_marker.pose.position.x = 0.
    ragdoll_m1_marker.pose.position.y = -1. 
    ragdoll_m1_marker.pose.position.z = 0.1
    ragdoll_m1_marker.pose.orientation.x = 0.
    ragdoll_m1_marker.pose.orientation.y = 0.
    ragdoll_m1_marker.pose.orientation.z = 0.
    ragdoll_m1_marker.pose.orientation.w = 1.
    ragdoll_m1_marker.scale.x = 0.2
    ragdoll_m1_marker.scale.y = 0.2
    ragdoll_m1_marker.scale.z = 0.2

    ragdoll_m2_marker.header.frame_id = 'world_link'
    ragdoll_m2_marker.type = 2
    ragdoll_m2_marker.ns = 'ragdoll_m2'
    ragdoll_m2_marker.action = 0
    ragdoll_m2_marker.action = 0
    ragdoll_m2_marker.color.g = 1
    ragdoll_m2_marker.color.a = 0.5
    ragdoll_m2_marker.pose.position.x = 0
    ragdoll_m2_marker.pose.position.y = 0.
    ragdoll_m2_marker.pose.position.z = 0.1
    ragdoll_m2_marker.pose.orientation.x = 0.
    ragdoll_m2_marker.pose.orientation.y = 0.
    ragdoll_m2_marker.pose.orientation.z = 0.
    ragdoll_m2_marker.pose.orientation.w = 1.
    ragdoll_m2_marker.scale.x = 0.2
    ragdoll_m2_marker.scale.y = 0.2
    ragdoll_m2_marker.scale.z = 0.2

    ragdoll_m3_marker.header.frame_id = 'world_link'
    ragdoll_m3_marker.type = 2
    ragdoll_m3_marker.ns = 'ragdoll_m3'
    ragdoll_m3_marker.action = 0
    ragdoll_m3_marker.color.b = 1
    ragdoll_m3_marker.color.a = 0.5
    ragdoll_m3_marker.pose.position.x = 0
    ragdoll_m3_marker.pose.position.y = 2.
    ragdoll_m3_marker.pose.position.z = 0.1
    ragdoll_m3_marker.pose.orientation.x = 0.
    ragdoll_m3_marker.pose.orientation.y = 0.
    ragdoll_m3_marker.pose.orientation.z = 0.
    ragdoll_m3_marker.pose.orientation.w = 1.
    ragdoll_m3_marker.scale.x = 0.2
    ragdoll_m3_marker.scale.y = 0.2
    ragdoll_m3_marker.scale.z = 0.2

    ragdoll_lower_body_marker.header.frame_id = 'world_link'
    ragdoll_lower_body_marker.type = 10
    ragdoll_lower_body_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Lower_Body.STL'
    ragdoll_lower_body_marker.ns = 'ragdoll_lower_body'
    ragdoll_lower_body_marker.action = 0
    ragdoll_lower_body_marker.color.r = 1.
    ragdoll_lower_body_marker.color.g = 1.
    ragdoll_lower_body_marker.color.b = 1.
    ragdoll_lower_body_marker.color.a = 0.5
    ragdoll_lower_body_marker.pose.position.x = 0
    ragdoll_lower_body_marker.pose.position.y = 2.
    ragdoll_lower_body_marker.pose.position.z = 0.1
    ragdoll_lower_body_marker.pose.orientation.x = 0.
    ragdoll_lower_body_marker.pose.orientation.y = 0.
    ragdoll_lower_body_marker.pose.orientation.z = 0.
    ragdoll_lower_body_marker.pose.orientation.w = 1.
    ragdoll_lower_body_marker.scale.x = 0.03
    ragdoll_lower_body_marker.scale.y = 0.03
    ragdoll_lower_body_marker.scale.z = 0.03

    ragdoll_l_thigh_marker.header.frame_id = 'world_link'
    ragdoll_l_thigh_marker.type = 10
    ragdoll_l_thigh_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Thigh.STL'
    ragdoll_l_thigh_marker.ns = 'ragdoll_l_thigh'
    ragdoll_l_thigh_marker.action = 0
    ragdoll_l_thigh_marker.color.r = 1.
    ragdoll_l_thigh_marker.color.g = 1.
    ragdoll_l_thigh_marker.color.b = 1.
    ragdoll_l_thigh_marker.color.a = 0.5
    ragdoll_l_thigh_marker.pose.position.x = 0
    ragdoll_l_thigh_marker.pose.position.y = 2.
    ragdoll_l_thigh_marker.pose.position.z = 0.1
    ragdoll_l_thigh_marker.pose.orientation.x = 0.
    ragdoll_l_thigh_marker.pose.orientation.y = 0.
    ragdoll_l_thigh_marker.pose.orientation.z = 0.
    ragdoll_l_thigh_marker.pose.orientation.w = 1.
    ragdoll_l_thigh_marker.scale.x = 0.03
    ragdoll_l_thigh_marker.scale.y = 0.03
    ragdoll_l_thigh_marker.scale.z = 0.03

    ragdoll_l_shin_marker.header.frame_id = 'world_link'
    ragdoll_l_shin_marker.type = 10
    ragdoll_l_shin_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Shin.STL'
    ragdoll_l_shin_marker.ns = 'ragdoll_l_shin'
    ragdoll_l_shin_marker.action = 0
    ragdoll_l_shin_marker.color.r = 1.
    ragdoll_l_shin_marker.color.g = 1.
    ragdoll_l_shin_marker.color.b = 1.
    ragdoll_l_shin_marker.color.a = 0.5
    ragdoll_l_shin_marker.pose.position.x = 0
    ragdoll_l_shin_marker.pose.position.y = 2.
    ragdoll_l_shin_marker.pose.position.z = 0.1
    ragdoll_l_shin_marker.pose.orientation.x = 0.
    ragdoll_l_shin_marker.pose.orientation.y = 0.
    ragdoll_l_shin_marker.pose.orientation.z = 0.
    ragdoll_l_shin_marker.pose.orientation.w = 1.
    ragdoll_l_shin_marker.scale.x = 0.03
    ragdoll_l_shin_marker.scale.y = 0.03
    ragdoll_l_shin_marker.scale.z = 0.03

    ragdoll_l_foot_marker.header.frame_id = 'world_link'
    ragdoll_l_foot_marker.type = 10
    ragdoll_l_foot_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Foot.STL'
    ragdoll_l_foot_marker.ns = 'ragdoll_l_foot'
    ragdoll_l_foot_marker.action = 0
    ragdoll_l_foot_marker.color.r = 1.
    ragdoll_l_foot_marker.color.g = 1.
    ragdoll_l_foot_marker.color.b = 1.
    ragdoll_l_foot_marker.color.a = 0.5
    ragdoll_l_foot_marker.pose.position.x = 0
    ragdoll_l_foot_marker.pose.position.y = 2.
    ragdoll_l_foot_marker.pose.position.z = 0.1
    ragdoll_l_foot_marker.pose.orientation.x = 0.
    ragdoll_l_foot_marker.pose.orientation.y = 0.
    ragdoll_l_foot_marker.pose.orientation.z = 0.
    ragdoll_l_foot_marker.pose.orientation.w = 1.
    ragdoll_l_foot_marker.scale.x = 0.03
    ragdoll_l_foot_marker.scale.y = 0.03
    ragdoll_l_foot_marker.scale.z = 0.03

    ragdoll_r_thigh_marker.header.frame_id = 'world_link'
    ragdoll_r_thigh_marker.type = 10
    ragdoll_r_thigh_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Thigh.STL'
    ragdoll_r_thigh_marker.ns = 'ragdoll_r_thigh'
    ragdoll_r_thigh_marker.action = 0
    ragdoll_r_thigh_marker.color.r = 1.
    ragdoll_r_thigh_marker.color.g = 1.
    ragdoll_r_thigh_marker.color.b = 1.
    ragdoll_r_thigh_marker.color.a = 0.5
    ragdoll_r_thigh_marker.pose.position.x = 0
    ragdoll_r_thigh_marker.pose.position.y = 2.
    ragdoll_r_thigh_marker.pose.position.z = 0.1
    ragdoll_r_thigh_marker.pose.orientation.x = 0.
    ragdoll_r_thigh_marker.pose.orientation.y = 0.
    ragdoll_r_thigh_marker.pose.orientation.z = 0.
    ragdoll_r_thigh_marker.pose.orientation.w = 1.
    ragdoll_r_thigh_marker.scale.x = 0.03
    ragdoll_r_thigh_marker.scale.y = 0.03
    ragdoll_r_thigh_marker.scale.z = 0.03

    ragdoll_r_shin_marker.header.frame_id = 'world_link'
    ragdoll_r_shin_marker.type = 10
    ragdoll_r_shin_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Shin.STL'
    ragdoll_r_shin_marker.ns = 'ragdoll_r_shin'
    ragdoll_r_shin_marker.action = 0
    ragdoll_r_shin_marker.color.r = 1.
    ragdoll_r_shin_marker.color.g = 1.
    ragdoll_r_shin_marker.color.b = 1.
    ragdoll_r_shin_marker.color.a = 0.5
    ragdoll_r_shin_marker.pose.position.x = 0
    ragdoll_r_shin_marker.pose.position.y = 2.
    ragdoll_r_shin_marker.pose.position.z = 0.1
    ragdoll_r_shin_marker.pose.orientation.x = 0.
    ragdoll_r_shin_marker.pose.orientation.y = 0.
    ragdoll_r_shin_marker.pose.orientation.z = 0.
    ragdoll_r_shin_marker.pose.orientation.w = 1.
    ragdoll_r_shin_marker.scale.x = 0.03
    ragdoll_r_shin_marker.scale.y = 0.03
    ragdoll_r_shin_marker.scale.z = 0.03

    ragdoll_r_foot_marker.header.frame_id = 'world_link'
    ragdoll_r_foot_marker.type = 10
    ragdoll_r_foot_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Foot.STL'
    ragdoll_r_foot_marker.ns = 'ragdoll_r_foot'
    ragdoll_r_foot_marker.action = 0
    ragdoll_r_foot_marker.color.r = 1.
    ragdoll_r_foot_marker.color.g = 1.
    ragdoll_r_foot_marker.color.b = 1.
    ragdoll_r_foot_marker.color.a = 0.5
    ragdoll_r_foot_marker.pose.position.x = 0
    ragdoll_r_foot_marker.pose.position.y = 2.
    ragdoll_r_foot_marker.pose.position.z = 0.1
    ragdoll_r_foot_marker.pose.orientation.x = 0.
    ragdoll_r_foot_marker.pose.orientation.y = 0.
    ragdoll_r_foot_marker.pose.orientation.z = 0.
    ragdoll_r_foot_marker.pose.orientation.w = 1.
    ragdoll_r_foot_marker.scale.x = 0.03
    ragdoll_r_foot_marker.scale.y = 0.03
    ragdoll_r_foot_marker.scale.z = 0.03

    ragdoll_middle_body_marker.header.frame_id = 'world_link'
    ragdoll_middle_body_marker.type = 10
    ragdoll_middle_body_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Middle_Body.STL'
    ragdoll_middle_body_marker.ns = 'ragdoll_middle_body'
    ragdoll_middle_body_marker.action = 0
    ragdoll_middle_body_marker.color.r = 1.
    ragdoll_middle_body_marker.color.g = 1.
    ragdoll_middle_body_marker.color.b = 1.
    ragdoll_middle_body_marker.color.a = 0.5
    ragdoll_middle_body_marker.pose.position.x = 0
    ragdoll_middle_body_marker.pose.position.y = 2.
    ragdoll_middle_body_marker.pose.position.z = 0.1
    ragdoll_middle_body_marker.pose.orientation.x = 0.
    ragdoll_middle_body_marker.pose.orientation.y = 0.
    ragdoll_middle_body_marker.pose.orientation.z = 0.
    ragdoll_middle_body_marker.pose.orientation.w = 1.
    ragdoll_middle_body_marker.scale.x = 0.03
    ragdoll_middle_body_marker.scale.y = 0.03
    ragdoll_middle_body_marker.scale.z = 0.03

    ragdoll_upper_body_marker.header.frame_id = 'world_link'
    ragdoll_upper_body_marker.type = 10
    ragdoll_upper_body_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Upper_Body.STL'
    ragdoll_upper_body_marker.ns = 'ragdoll_upper_body'
    ragdoll_upper_body_marker.action = 0
    ragdoll_upper_body_marker.color.r = 1.
    ragdoll_upper_body_marker.color.g = 1.
    ragdoll_upper_body_marker.color.b = 1.
    ragdoll_upper_body_marker.color.a = 0.5
    ragdoll_upper_body_marker.pose.position.x = 0
    ragdoll_upper_body_marker.pose.position.y = 2.
    ragdoll_upper_body_marker.pose.position.z = 0.1
    ragdoll_upper_body_marker.pose.orientation.x = 0.
    ragdoll_upper_body_marker.pose.orientation.y = 0.
    ragdoll_upper_body_marker.pose.orientation.z = 0.
    ragdoll_upper_body_marker.pose.orientation.w = 1.
    ragdoll_upper_body_marker.scale.x = 0.03
    ragdoll_upper_body_marker.scale.y = 0.03
    ragdoll_upper_body_marker.scale.z = 0.03

    ragdoll_l_arm_marker.header.frame_id = 'world_link'
    ragdoll_l_arm_marker.type = 10
    ragdoll_l_arm_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Arm.STL'
    ragdoll_l_arm_marker.ns = 'ragdoll_l_arm'
    ragdoll_l_arm_marker.action = 0
    ragdoll_l_arm_marker.color.r = 1.
    ragdoll_l_arm_marker.color.g = 1.
    ragdoll_l_arm_marker.color.b = 1.
    ragdoll_l_arm_marker.color.a = 0.5
    ragdoll_l_arm_marker.pose.position.x = 0
    ragdoll_l_arm_marker.pose.position.y = 2.
    ragdoll_l_arm_marker.pose.position.z = 0.1
    ragdoll_l_arm_marker.pose.orientation.x = 0.
    ragdoll_l_arm_marker.pose.orientation.y = 0.
    ragdoll_l_arm_marker.pose.orientation.z = 0.
    ragdoll_l_arm_marker.pose.orientation.w = 1.
    ragdoll_l_arm_marker.scale.x = 0.03
    ragdoll_l_arm_marker.scale.y = 0.03
    ragdoll_l_arm_marker.scale.z = 0.03

    ragdoll_l_wrist_marker.header.frame_id = 'world_link'
    ragdoll_l_wrist_marker.type = 10 
    ragdoll_l_wrist_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Wrist.STL'
    ragdoll_l_wrist_marker.ns = 'ragdoll_l_wrist'
    ragdoll_l_wrist_marker.action = 0
    ragdoll_l_wrist_marker.color.r = 1. 
    ragdoll_l_wrist_marker.color.g = 1. 
    ragdoll_l_wrist_marker.color.b = 1. 
    ragdoll_l_wrist_marker.color.a = 0.5
    ragdoll_l_wrist_marker.pose.position.x = 0
    ragdoll_l_wrist_marker.pose.position.y = 2. 
    ragdoll_l_wrist_marker.pose.position.z = 0.1
    ragdoll_l_wrist_marker.pose.orientation.x = 0. 
    ragdoll_l_wrist_marker.pose.orientation.y = 0. 
    ragdoll_l_wrist_marker.pose.orientation.z = 0. 
    ragdoll_l_wrist_marker.pose.orientation.w = 1. 
    ragdoll_l_wrist_marker.scale.x = 0.03 
    ragdoll_l_wrist_marker.scale.y = 0.03 
    ragdoll_l_wrist_marker.scale.z = 0.03

    ragdoll_l_hand_marker.header.frame_id = 'world_link'
    ragdoll_l_hand_marker.type = 10 
    ragdoll_l_hand_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Hand.STL'
    ragdoll_l_hand_marker.ns = 'ragdoll_l_hand'
    ragdoll_l_hand_marker.action = 0
    ragdoll_l_hand_marker.color.r = 1. 
    ragdoll_l_hand_marker.color.g = 1. 
    ragdoll_l_hand_marker.color.b = 1. 
    ragdoll_l_hand_marker.color.a = 0.5
    ragdoll_l_hand_marker.pose.position.x = 0
    ragdoll_l_hand_marker.pose.position.y = 2. 
    ragdoll_l_hand_marker.pose.position.z = 0.1
    ragdoll_l_hand_marker.pose.orientation.x = 0. 
    ragdoll_l_hand_marker.pose.orientation.y = 0. 
    ragdoll_l_hand_marker.pose.orientation.z = 0. 
    ragdoll_l_hand_marker.pose.orientation.w = 1. 
    ragdoll_l_hand_marker.scale.x = 0.03 
    ragdoll_l_hand_marker.scale.y = 0.03 
    ragdoll_l_hand_marker.scale.z = 0.03

    ragdoll_neck_marker.header.frame_id = 'world_link'
    ragdoll_neck_marker.type = 10 
    ragdoll_neck_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Neck.STL'
    ragdoll_neck_marker.ns = 'ragdoll_neck'
    ragdoll_neck_marker.action = 0
    ragdoll_neck_marker.color.r = 1. 
    ragdoll_neck_marker.color.g = 1. 
    ragdoll_neck_marker.color.b = 1. 
    ragdoll_neck_marker.color.a = 0.5
    ragdoll_neck_marker.pose.position.x = 0
    ragdoll_neck_marker.pose.position.y = 2. 
    ragdoll_neck_marker.pose.position.z = 0.1
    ragdoll_neck_marker.pose.orientation.x = 0. 
    ragdoll_neck_marker.pose.orientation.y = 0. 
    ragdoll_neck_marker.pose.orientation.z = 0. 
    ragdoll_neck_marker.pose.orientation.w = 1. 
    ragdoll_neck_marker.scale.x = 0.03 
    ragdoll_neck_marker.scale.y = 0.03 
    ragdoll_neck_marker.scale.z = 0.03

    ragdoll_head_marker.header.frame_id = 'world_link'
    ragdoll_head_marker.type = 10 
    ragdoll_head_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Head.STL'
    ragdoll_head_marker.ns = 'ragdoll_head'
    ragdoll_head_marker.action = 0
    ragdoll_head_marker.color.r = 1. 
    ragdoll_head_marker.color.g = 1. 
    ragdoll_head_marker.color.b = 1. 
    ragdoll_head_marker.color.a = 0.5
    ragdoll_head_marker.pose.position.x = 0
    ragdoll_head_marker.pose.position.y = 2. 
    ragdoll_head_marker.pose.position.z = 0.1
    ragdoll_head_marker.pose.orientation.x = 0. 
    ragdoll_head_marker.pose.orientation.y = 0. 
    ragdoll_head_marker.pose.orientation.z = 0. 
    ragdoll_head_marker.pose.orientation.w = 1. 
    ragdoll_head_marker.scale.x = 0.03 
    ragdoll_head_marker.scale.y = 0.03 
    ragdoll_head_marker.scale.z = 0.03
    
    ragdoll_r_arm_marker.header.frame_id = 'world_link'
    ragdoll_r_arm_marker.type = 10
    ragdoll_r_arm_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Arm.STL'
    ragdoll_r_arm_marker.ns = 'ragdoll_r_arm'
    ragdoll_r_arm_marker.action = 0
    ragdoll_r_arm_marker.color.r = 1.
    ragdoll_r_arm_marker.color.g = 1.
    ragdoll_r_arm_marker.color.b = 1.
    ragdoll_r_arm_marker.color.a = 0.5
    ragdoll_r_arm_marker.pose.position.x = 0
    ragdoll_r_arm_marker.pose.position.y = 2.
    ragdoll_r_arm_marker.pose.position.z = 0.1
    ragdoll_r_arm_marker.pose.orientation.x = 0.
    ragdoll_r_arm_marker.pose.orientation.y = 0.
    ragdoll_r_arm_marker.pose.orientation.z = 0.
    ragdoll_r_arm_marker.pose.orientation.w = 1.
    ragdoll_r_arm_marker.scale.x = 0.03
    ragdoll_r_arm_marker.scale.y = 0.03
    ragdoll_r_arm_marker.scale.z = 0.03

    ragdoll_r_wrist_marker.header.frame_id = 'world_link'
    ragdoll_r_wrist_marker.type = 10
    ragdoll_r_wrist_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Wrist.STL'
    ragdoll_r_wrist_marker.ns = 'ragdoll_r_wrist'
    ragdoll_r_wrist_marker.action = 0
    ragdoll_r_wrist_marker.color.r = 1.
    ragdoll_r_wrist_marker.color.g = 1.
    ragdoll_r_wrist_marker.color.b = 1.
    ragdoll_r_wrist_marker.color.a = 0.5
    ragdoll_r_wrist_marker.pose.position.x = 0
    ragdoll_r_wrist_marker.pose.position.y = 2.
    ragdoll_r_wrist_marker.pose.position.z = 0.1
    ragdoll_r_wrist_marker.pose.orientation.x = 0.
    ragdoll_r_wrist_marker.pose.orientation.y = 0.
    ragdoll_r_wrist_marker.pose.orientation.z = 0.
    ragdoll_r_wrist_marker.pose.orientation.w = 1.
    ragdoll_r_wrist_marker.scale.x = 0.03
    ragdoll_r_wrist_marker.scale.y = 0.03
    ragdoll_r_wrist_marker.scale.z = 0.03

    ragdoll_r_hand_marker.header.frame_id = 'world_link'
    ragdoll_r_hand_marker.type = 10
    ragdoll_r_hand_marker.mesh_resource = 'package://crona_description/stand_alone_gazebo/models/full_ragdoll/meshes/Test_Hand.STL'
    ragdoll_r_hand_marker.ns = 'ragdoll_r_hand'
    ragdoll_r_hand_marker.action = 0
    ragdoll_r_hand_marker.color.r = 1.
    ragdoll_r_hand_marker.color.g = 1.
    ragdoll_r_hand_marker.color.b = 1.
    ragdoll_r_hand_marker.color.a = 0.5
    ragdoll_r_hand_marker.pose.position.x = 0
    ragdoll_r_hand_marker.pose.position.y = 2.
    ragdoll_r_hand_marker.pose.position.z = 0.1
    ragdoll_r_hand_marker.pose.orientation.x = 0.
    ragdoll_r_hand_marker.pose.orientation.y = 0.
    ragdoll_r_hand_marker.pose.orientation.z = 0.
    ragdoll_r_hand_marker.pose.orientation.w = 1.
    ragdoll_r_hand_marker.scale.x = 0.03
    ragdoll_r_hand_marker.scale.y = 0.03
    ragdoll_r_hand_marker.scale.z = 0.03
   
    self.ragdoll_markers.markers.append(ragdoll_m1_marker)
    self.ragdoll_markers.markers.append(ragdoll_m2_marker)
    self.ragdoll_markers.markers.append(ragdoll_m3_marker)
    self.ragdoll_markers.markers.append(ragdoll_lower_body_marker)
    self.ragdoll_markers.markers.append(ragdoll_l_thigh_marker)
    self.ragdoll_markers.markers.append(ragdoll_l_shin_marker)
    self.ragdoll_markers.markers.append(ragdoll_l_foot_marker)
    self.ragdoll_markers.markers.append(ragdoll_r_thigh_marker)
    self.ragdoll_markers.markers.append(ragdoll_r_shin_marker)
    self.ragdoll_markers.markers.append(ragdoll_r_foot_marker)
    self.ragdoll_markers.markers.append(ragdoll_middle_body_marker)
    self.ragdoll_markers.markers.append(ragdoll_upper_body_marker)
    self.ragdoll_markers.markers.append(ragdoll_l_arm_marker)
    self.ragdoll_markers.markers.append(ragdoll_l_wrist_marker)
    self.ragdoll_markers.markers.append(ragdoll_l_hand_marker)
    self.ragdoll_markers.markers.append(ragdoll_neck_marker)
    self.ragdoll_markers.markers.append(ragdoll_head_marker)
    self.ragdoll_markers.markers.append(ragdoll_r_arm_marker)
    self.ragdoll_markers.markers.append(ragdoll_r_wrist_marker)
    self.ragdoll_markers.markers.append(ragdoll_r_hand_marker)

    larm_force_marker_x.header.frame_id = 'base_link'
    larm_force_marker_x.type = 0
    larm_force_marker_x.action = 0
    larm_force_marker_x.color.r = 1
    larm_force_marker_x.color.a = 0.5
    larm_force_marker_x.pose.orientation.x = 0
    larm_force_marker_x.pose.orientation.y = 0
    larm_force_marker_x.pose.orientation.z = 0
    larm_force_marker_x.pose.orientation.w = 1
    larm_force_marker_text_x.header.frame_id = 'base_link'
    larm_force_marker_text_x.type = 9
    larm_force_marker_text_x.action = 0
    larm_force_marker_text_x.color.r = 1
    larm_force_marker_text_x.color.a = 0.5
    larm_force_marker_text_x.pose.orientation.x = 0
    larm_force_marker_text_x.pose.orientation.y = 0
    larm_force_marker_text_x.pose.orientation.z = 0
    larm_force_marker_text_x.pose.orientation.w = 1    

    larm_force_marker_y.header.frame_id = 'base_link'
    larm_force_marker_y.type = 0
    larm_force_marker_y.action = 0
    larm_force_marker_y.color.g = 1
    larm_force_marker_y.color.a = 0.5
    larm_force_marker_y.pose.orientation.x = 0
    larm_force_marker_y.pose.orientation.y = 0
    larm_force_marker_y.pose.orientation.z = 0.707
    larm_force_marker_y.pose.orientation.w = 0.707
    larm_force_marker_text_y.header.frame_id = 'base_link'
    larm_force_marker_text_y.type = 9
    larm_force_marker_text_y.action = 0
    larm_force_marker_text_y.color.g = 1
    larm_force_marker_text_y.color.a = 0.5
    larm_force_marker_text_y.pose.orientation.x = 0
    larm_force_marker_text_y.pose.orientation.y = 0
    larm_force_marker_text_y.pose.orientation.z = 0.707
    larm_force_marker_text_y.pose.orientation.w = 0.707

    larm_force_marker_z.header.frame_id = 'base_link'
    larm_force_marker_z.type = 0
    larm_force_marker_z.action = 0
    larm_force_marker_z.color.b = 1
    larm_force_marker_z.color.a = 0.5
    larm_force_marker_z.pose.orientation.x = 0
    larm_force_marker_z.pose.orientation.y = -0.707
    larm_force_marker_z.pose.orientation.z = 0
    larm_force_marker_z.pose.orientation.w = 0.707
    larm_force_marker_text_z.header.frame_id = 'base_link'
    larm_force_marker_text_z.type = 9
    larm_force_marker_text_z.action = 0
    larm_force_marker_text_z.color.b = 1
    larm_force_marker_text_z.color.a = 0.5
    larm_force_marker_text_z.pose.orientation.x = 0
    larm_force_marker_text_z.pose.orientation.y = -0.707
    larm_force_marker_text_z.pose.orientation.z = 0.
    larm_force_marker_text_z.pose.orientation.w = 0.707

    rarm_force_marker_x.header.frame_id = 'base_link'
    rarm_force_marker_x.type = 0
    rarm_force_marker_x.action = 0
    rarm_force_marker_x.color.r = 1
    rarm_force_marker_x.color.a = 0.5
    rarm_force_marker_x.pose.orientation.x = 0
    rarm_force_marker_x.pose.orientation.y = 0
    rarm_force_marker_x.pose.orientation.z = 0
    rarm_force_marker_x.pose.orientation.w = 1
    rarm_force_marker_text_x.header.frame_id = 'base_link'
    rarm_force_marker_text_x.type = 9
    rarm_force_marker_text_x.action = 0
    rarm_force_marker_text_x.color.r = 1
    rarm_force_marker_text_x.color.a = 0.5
    rarm_force_marker_text_x.pose.orientation.x = 0
    rarm_force_marker_text_x.pose.orientation.y = 0
    rarm_force_marker_text_x.pose.orientation.z = 0
    rarm_force_marker_text_x.pose.orientation.w = 1    

    rarm_force_marker_y.header.frame_id = 'base_link'
    rarm_force_marker_y.type = 0
    rarm_force_marker_y.action = 0
    rarm_force_marker_y.color.g = 1
    rarm_force_marker_y.color.a = 0.5
    rarm_force_marker_y.pose.orientation.x = 0
    rarm_force_marker_y.pose.orientation.y = 0
    rarm_force_marker_y.pose.orientation.z = 0.707
    rarm_force_marker_y.pose.orientation.w = 0.707
    rarm_force_marker_text_y.header.frame_id = 'base_link'
    rarm_force_marker_text_y.type = 9
    rarm_force_marker_text_y.action = 0
    rarm_force_marker_text_y.color.g = 1
    rarm_force_marker_text_y.color.a = 0.5
    rarm_force_marker_text_y.pose.orientation.x = 0
    rarm_force_marker_text_y.pose.orientation.y = 0
    rarm_force_marker_text_y.pose.orientation.z = -0.707
    rarm_force_marker_text_y.pose.orientation.w = 0.707   

    rarm_force_marker_z.header.frame_id = 'base_link'
    rarm_force_marker_z.type = 0
    rarm_force_marker_z.action = 0
    rarm_force_marker_z.color.b = 1
    rarm_force_marker_z.color.a = 0.5
    rarm_force_marker_z.pose.orientation.x = 0
    rarm_force_marker_z.pose.orientation.y = -0.707
    rarm_force_marker_z.pose.orientation.z = 0
    rarm_force_marker_z.pose.orientation.w = 0.707
    rarm_force_marker_text_z.header.frame_id = 'base_link'
    rarm_force_marker_text_z.type = 9
    rarm_force_marker_text_z.action = 0
    rarm_force_marker_text_z.color.b = 1
    rarm_force_marker_text_z.color.a = 0.5
    rarm_force_marker_text_z.pose.orientation.x = 0
    rarm_force_marker_text_z.pose.orientation.y = -0.707
    rarm_force_marker_text_z.pose.orientation.z = 0.
    rarm_force_marker_text_z.pose.orientation.w = 0.707

    self.larm_force_markers.markers.append(larm_force_marker_x)
    self.larm_force_markers.markers.append(larm_force_marker_y)
    self.larm_force_markers.markers.append(larm_force_marker_z)
    self.larm_force_markers.markers.append(larm_force_marker_text_x)
    self.larm_force_markers.markers.append(larm_force_marker_text_y)
    self.larm_force_markers.markers.append(larm_force_marker_text_z)
    self.rarm_force_markers.markers.append(rarm_force_marker_x)
    self.rarm_force_markers.markers.append(rarm_force_marker_y)
    self.rarm_force_markers.markers.append(rarm_force_marker_z)
    self.rarm_force_markers.markers.append(rarm_force_marker_text_x)
    self.rarm_force_markers.markers.append(rarm_force_marker_text_y)
    self.rarm_force_markers.markers.append(rarm_force_marker_text_z)

  def _updateIKFeasibility(self, ps):
    p = PoseStamped()
    p.pose.orientation = ps.pose.orientation
    p.header.frame_id = '/base_link'
    offset = -0.216 #distance from tool frame to wrist_roll_link
    q = ps.pose.orientation
    rot = tr.tft.quaternion_matrix((q.x, q.y, q.z, q.w))
    offset_vec = rot * np.matrix([[offset, 0., 0., 1.]]).T
    p.pose.position.x = ps.pose.position.x + offset_vec[0]
    p.pose.position.y = ps.pose.position.y + offset_vec[1]
    p.pose.position.z = ps.pose.position.z + offset_vec[2]
    self.test_pos_pub.publish(p)

    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(5.0)
    ik_req = PositionIKRequest()
    ik_req.ik_link_name = self.ik_solver_info.link_names[-1]
    ik_req.ik_seed_state.joint_state.name = self.ik_solver_info.joint_names
    ik_req.ik_seed_state.joint_state.position = self.joint_state
    ik_req.pose_stamped = ps
    req.ik_request = ik_req
    ik_sol = self.pr2_ik_client.call(req)
    if ik_sol.error_code.val < 0:
      print "IK Failed with error code: %s" %ik_sol.error_code.val
      for marker in self.wp_im.controls[0].markers:
        marker.color = ColorRGBA(1,0,0,0.7)
    else:
      for marker in self.wp_im.controls[0].markers:
        marker.color = ColorRGBA(0,1,0,0.7)

  ## Callback for the interactive marker location. 
  #
  # Receives and stores the updated pose of the marker in space as the user moves it around.
  def lforearm_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_lforearm_goal_pose = ps

  def rforearm_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_rforearm_goal_pose = ps

  def ragdoll_m1_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_ragdoll_m1_goal_pose = ps

  def ragdoll_m2_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_ragdoll_m2_goal_pose = ps

  def ragdoll_m3_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_ragdoll_m3_goal_pose = ps

  def object_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      #gp = GoalPose()
      #gp.base_pose.x = feedback.pose.position.x
      #gp.base_pose.y = feedback.pose.position.y
      self.current_object_goal_pose = ps

  def platform_1_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_platform_1_goal_pose = ps

  def platform_2_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_platform_2_goal_pose = ps

  def platform_3_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_platform_3_goal_pose = ps

  def platform_4_im_LocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_platform_4_goal_pose = ps

  def updateMarkerPositions(self):
    #self.server.setPose(self.lforearm_im, self.current_lforearm_goal_pose)
    #self.server.setPose(self.rforearm_im, self.current_rforearm_goal_pose)
    #self.server.setPose(self.ragdoll_m1_im, self.current_ragdoll_m1_goal_pose)
    #self.server.setPose(self.ragdoll_m2_im, self.current_ragdoll_m2_goal_pose)
    #self.server.setPose(self.ragdoll_m3_im, self.current_ragdoll_m3_goal_pose)
    self.server.setPose(self.object_im, self.current_object_goal_pose)
    self.server.applyChanges()

  ## Publishes the current pose of the interactive marker as the goal pose for the MPC.
  # Also sets the orientation weight for the controller to 0.0 (ie, position only).
  def lforearm_goalHandler(self, feedback):
    rospy.loginfo("MPC Teleop: Publishing new goal. Position only.")
    self.lforearm_pose_marked = 1
    self.gp.lforearm_pose = self.current_lforearm_goal_pose.pose
    if self.lforearm_pose_marked == 1 and self.rforearm_pose_marked == 1:
	self.gp.base_pose.x = self.base_ja[0]
	self.gp.base_pose.y = self.base_ja[1]
	self.gp.base_pose.z = self.base_ja[2]
	print "gp: ",self.gp
    	for i in range(5):
    	    self.goal_pos_pub.publish(self.gp)
	self.lforearm_pose_marked = 0
	self.rforearm_pose_marked = 0

  def rforearm_goalHandler(self, feedback):
    rospy.loginfo("MPC Teleop: Publishing new goal. Position only.")
    self.rforearm_pose_marked = 1
    self.gp.rforearm_pose = self.current_rforearm_goal_pose.pose
    if self.lforearm_pose_marked == 1 and self.rforearm_pose_marked == 1:
	self.gp.base_pose.x = self.base_ja[0]
        self.gp.base_pose.y = self.base_ja[1]
        self.gp.base_pose.z = self.base_ja[2]
	print "gp: ",self.gp
    	for i in range(5):
            self.goal_pos_pub.publish(self.gp)  
	self.lforearm_pose_marked = 0
        self.rforearm_pose_marked = 0

  def crona2_lift_set_height(self, feedback):
    self.gp.base_pose.x = self.base_ja[0]
    self.gp.base_pose.y = self.base_ja[1]
    self.gp.base_pose.z = self.base_ja[2]
    self.gp.lforearm_pose.position.x = 0.79
    #self.gp.lforearm_pose.position.y = self.init_lforearm_pose.pose.position.y
    self.gp.lforearm_pose.position.y = 0.265
    self.gp.lforearm_pose.position.z = 0.607
    self.gp.lforearm_pose.orientation.x = 0.
    self.gp.lforearm_pose.orientation.y = -0.707
    self.gp.lforearm_pose.orientation.z = 0.
    self.gp.lforearm_pose.orientation.w = 0.707
    self.gp.rforearm_pose.position.x = 0.79
    #self.gp.rforearm_pose.position.y = self.init_rforearm_pose.pose.position.y
    self.gp.rforearm_pose.position.y = -0.265
    self.gp.rforearm_pose.position.z = 0.607
    self.gp.rforearm_pose.orientation.x = 0.
    self.gp.rforearm_pose.orientation.y = -0.707
    self.gp.rforearm_pose.orientation.z = 0.
    self.gp.rforearm_pose.orientation.w = 0.707
    self.gp.object_pose.position.x = 0.79
    self.gp.object_pose.position.y = 0.
    self.gp.object_pose.position.z = 0.607
    self.gp.object_pose.orientation.x = 0.
    self.gp.object_pose.orientation.y = 0.
    self.gp.object_pose.orientation.z = 0.
    self.gp.object_pose.orientation.w = 1.
    for i in range(10):
        self.goal_pos_pub.publish(self.gp)

  def ragdoll_m1_position(self, feedback):
    self.ragdoll_m1_position_marked = 1
    self.ragdoll_m1_marked_position = self.current_ragdoll_m1_goal_pose.pose
    if self.ragdoll_m1_position_marked == 1 and self.ragdoll_m2_position_marked == 1 and self.ragdoll_m3_position_marked == 1:
	print "ragdoll_m1 position: ",self.ragdoll_m1_marked_position
	print "ragdoll_m2 position: ",self.ragdoll_m2_marked_position
	print "ragdoll_m3 position: ",self.ragdoll_m3_marked_position
	self.crona2_pick_up_ragdoll(self.ragdoll_m1_marked_position, self.ragdoll_m2_marked_position, self.ragdoll_m3_marked_position)
	self.ragdoll_m1_position_marked = 0
	self.ragdoll_m2_position_marked = 0
	self.ragdoll_m3_position_marked = 0

  def ragdoll_m2_position(self, feedback):
    self.ragdoll_m2_position_marked = 1
    self.ragdoll_m2_marked_position = self.current_ragdoll_m2_goal_pose.pose
    if self.ragdoll_m1_position_marked == 1 and self.ragdoll_m2_position_marked == 1 and self.ragdoll_m3_position_marked == 1:
	print "ragdoll_m1 position: ",self.ragdoll_m1_marked_position
        print "ragdoll_m2 position: ",self.ragdoll_m2_marked_position
        print "ragdoll_m3 position: ",self.ragdoll_m3_marked_position
	self.crona2_pick_up_ragdoll(self.ragdoll_m1_marked_position, self.ragdoll_m2_marked_position, self.ragdoll_m3_marked_position)
	self.ragdoll_m1_position_marked = 0
        self.ragdoll_m2_position_marked = 0
        self.ragdoll_m3_position_marked = 0

  def ragdoll_m3_position(self, feedback):
    self.ragdoll_m3_position_marked = 1
    self.ragdoll_m3_marked_position = self.current_ragdoll_m3_goal_pose.pose
    if self.ragdoll_m1_position_marked == 1 and self.ragdoll_m2_position_marked == 1 and self.ragdoll_m3_position_marked == 1:
	print "ragdoll_m1 position: ",self.ragdoll_m1_marked_position
        print "ragdoll_m2 position: ",self.ragdoll_m2_marked_position
        print "ragdoll_m3 position: ",self.ragdoll_m3_marked_position
	self.crona2_pick_up_ragdoll(self.ragdoll_m1_marked_position, self.ragdoll_m2_marked_position, self.ragdoll_m3_marked_position)
	self.ragdoll_m1_position_marked = 0
        self.ragdoll_m2_position_marked = 0
        self.ragdoll_m3_position_marked = 0

  def platform_1_position(self, feedback):
    self.platform_1_position_marked = 1
    self.platform_1_marked_position = self.current_platform_1_goal_pose.pose

  def platform_2_position(self, feedback):
    self.platform_2_position_marked = 1
    self.platform_2_marked_position = self.current_platform_2_goal_pose.pose

  def platform_3_position(self, feedback):
    self.platform_3_position_marked = 1
    self.platform_3_marked_position = self.current_platform_3_goal_pose.pose

  def platform_4_position(self, feedback):
    self.platform_4_position_marked = 1
    self.platform_4_marked_position = self.current_platform_4_goal_pose.pose

  def crona2_lower_down(self, feedback):
    self.lower_down.start()

  def crona2_pick_up_ragdoll(self, m1_pose, m2_pose, m3_pose):
    x1 = m1_pose.position.x
    y1 = m1_pose.position.y
    x2 = m2_pose.position.x
    y2 = m2_pose.position.y
    x3 = m3_pose.position.x
    y3 = m3_pose.position.y
    theta = math.atan2(y3-y1,x3-x1)-math.pi/2
    print "rotation angle: ",theta
    self.move_towards_object.start(theta=theta,object_x=x2,object_y=y2)
    self.raise_arm.start(theta=theta,object_x=x2,object_y=y2)
    print "done raising arm"
    #self.move_and_scoop.start(theta=theta,object_x=x2,object_y=y2,object_x1=x1,object_y1=y1,object_x3=x3,object_y3=y3)
    self.move_and_scoop.start(theta=theta,object_x=x2,object_y=y2)
    print "done move and scoop"

  def crona2_move_to_platform(self, feedback):
    #self.gp.base_pose.x = 0.1 
    if self.platform_1_position_marked == 1 and self.platform_2_position_marked == 1 and self.platform_3_position_marked == 1 and self.platform_4_position_marked == 1:
	print "self.platform_1_marked_position: ",self.platform_1_marked_position
	print "self.platform_2_marked_position: ",self.platform_2_marked_position
	print "self.platform_3_marked_position: ",self.platform_3_marked_position
	print "self.platform_4_marked_position: ",self.platform_4_marked_position
	
	p1_x = self.platform_1_marked_position.position.x
	p1_y = self.platform_1_marked_position.position.y
	p2_x = self.platform_2_marked_position.position.x
	p2_y = self.platform_2_marked_position.position.y
	p3_x = self.platform_3_marked_position.position.x
        p3_y = self.platform_3_marked_position.position.y
        p4_x = self.platform_4_marked_position.position.x
        p4_y = self.platform_4_marked_position.position.y

	# more complicated...user can choose the corners arbitrarily	
	'''	
	dist = []
	dist += math.hypot(p1_x-p2_x,p1_y-p2_y)
	dist += math.hypot(p1_x-p3_x,p1_y-p3_y)
	dist += math.hypot(p1_x-p4_x,p1_y-p4_y)
	dist += math.hypot(p2_x-p3_x,p2_y-p3_y)
	dist += math.hypot(p2_x-p4_x,p2_y-p4_y)
	dist += math.hypot(p3_x-p4_x,p3_y-p4_y)
	d = copy.copy(dist)
	four_min_values = []
	for i in range(4):
	    four_min_values += min(d)
	    d.remove(min(d))

	if abs(four_min_values[0]-four_min_values[1]) < 0.2 and abs(four_min_values[2]-four_min_values[3]) < 0.2:
	    
	    dist_from_base = []	
	    dist_from_base += math.hypot(self.base_x-p1_x,self.base_y-p1_y)
	    dist_from_base += math.hypot(self.base_x-p2_x,self.base_y-p2_y)
	    dist_from_base += math.hypot(self.base_x-p3_x,self.base_y-p3_y)
	    dist_from_base += math.hypot(self.base_x-p4_x,self.base_y-p4_y)
	    min_p = dist_from_base.index(min(dist_from_base))

	    rectangle = []
	    rectangle += min_p

	    if min_p == 1:
		if dist[0] <= dist[1] and dist[0] <= dist[2]:
		    rectangle += 2
		elif dist[1] <= dist[2] and dist
	    
	    
	    self.gp.base_pose.x = 1.
            self.gp.base_pose.y = 0.8
            self.gp.base_pose.z = 1.57
            for i in range(100):
               self.goal_pos_pub.publish(self.gp)
	    
	else:
	    "Please mark the platform positions better"
	'''
	
	# simple...user selects the corners in the following order - 1 is the closest to the robot, 2 is the shortest length from 1, 3 is the medium length from 1, and 4 is the furthest from 1
	# robot aligns with 1 and 2, perpendicular to 1 and 3

	#x_midpoint_of_long_side = (p1_x+p3_x)/2
	#y_midpoint_of_long_side = (p1_y+p3_y)/2
	
	# even simpler...assume the platform doesn't rotate, user moves the markers to the corresponding corners
 		
	self.gp.base_pose.x = p1_x-0.25
	self.gp.base_pose.y = p1_y-0.7
	self.gp.base_pose.z = 1.57
	#self.gp.base_pose.x = 1.
        #self.gp.base_pose.y = 0.8
        #self.gp.base_pose.z = 1.57
        for i in range(100):
            self.goal_pos_pub.publish(self.gp)

    else:
	print "Need to mark positions first"

  def crona2_place(self, feedback):
    #self.gp.lforearm_pose.position.z = 0.45
    #self.gp.rforearm_pose.position.z = 0.45
    self.gp.lforearm_pose.position.x = 0.87
    self.gp.rforearm_pose.position.x = 0.87
    #self.gp.lforearm_pose.position.y = self.init_lforearm_pose.pose.position.y
    #self.gp.rforearm_pose.position.y = self.init_rforearm_pose.pose.position.y
    self.gp.lforearm_pose.position.y = 0.265
    self.gp.rforearm_pose.position.y = -0.265
    self.gp.lforearm_pose.position.z = 0.2
    self.gp.rforearm_pose.position.z = 0.2
    self.gp.lforearm_pose.orientation.x = 0.
    self.gp.lforearm_pose.orientation.y = -0.707
    self.gp.lforearm_pose.orientation.z = 0.
    self.gp.lforearm_pose.orientation.w = 0.707
    self.gp.rforearm_pose.orientation.x = 0.
    self.gp.rforearm_pose.orientation.y = -0.707
    self.gp.rforearm_pose.orientation.z = 0.
    self.gp.rforearm_pose.orientation.w = 0.707
    #self.gp.object_pose.position.x = 0.87
    self.gp.object_pose.position.x = 0.95
    self.gp.object_pose.position.y = 0.
    self.gp.object_pose.position.z = 0.4
    self.gp.object_pose.orientation.x = 0.
    self.gp.object_pose.orientation.y = 0.
    self.gp.object_pose.orientation.z = 0.
    self.gp.object_pose.orientation.w = 1.
	
    for i in range(100):
        self.goal_pos_pub.publish(self.gp)

  def crona2_place2(self, feedback):
    #self.gp.lforearm_pose.position.z = -0.07
    #self.gp.rforearm_pose.position.z = -0.07
    self.gp.lforearm_pose.position.z = -0.17
    self.gp.lforearm_pose.position.z = -0.17
    self.gp.object_pose.position.z = -0.17
    #self.gp.base_pose.x = -0.6
    #self.gp.base_pose.x = -0.2
    self.gp.base_pose.x = self.base_ja[0]-1.2
    self.gp.base_pose_time.data = 10
    for i in range(100):
        self.goal_pos_pub.publish(self.gp)

  def crona2_release(self, feedback):
    self.release_object.start()
    
  def crona2_return_home(self, feedback):
    self.return_home.start()

  def apply_random_wrench(self,feedback):
    for i in range(100):
    	self.apply_random_wrench_pub.publish(1)

  def object_goalHandler(self, feedback):
    rospy.loginfo("MPC Teleop: Publishing new goal. Position only.")
    self.gp.base_pose.x = self.current_object_goal_pose.pose.position.x
    self.gp.base_pose.y = self.current_object_goal_pose.pose.position.y
    for i in range(5):
        self.goal_pos_pub.publish(self.gp)

  def object_com_lift(self, feedback):
    self.gp.object_pose = self.current_object_goal_pose.pose
    self.gp.object_pose.orientation.x = 0.
    self.gp.object_pose.orientation.y = 0.
    self.gp.object_pose.orientation.z = 0.
    self.gp.object_pose.orientation.w = 1.
    for i in range(5):
        self.goal_pos_pub.publish(self.gp)

  ## Publishes an empty trajectory message.
  # This has the effect of flushing the stored trajectory and goal pose from the waypoint generator, stopping the controller motion.
  def stopArmHandler(self, feedback):
    self.stop_start_epc()
    self.goal_traj_pub.publish(PoseArray())
    rospy.loginfo("Stopping MPC")

  ## Enable the PR2 PPS sensors by adding the topics to the skin client.
  def enablePps(self):
    self.add_topic_pub.publish('/pr2_pps_right_sensor/taxels/forces')
    self.add_topic_pub.publish('/pr2_pps_left_sensor/taxels/forces')

  ## Enable the PR2 PPS sensors by removing the topics from the skin client.
  def disablePps(self):
    self.remove_topic_pub.publish('/pr2_pps_right_sensor/taxels/forces')
    self.remove_topic_pub.publish('/pr2_pps_left_sensor/taxels/forces')

  ## Open the PR2 gripper for this arm and enable the PPS sensors (assumes this is running on the PR2).
  def openGripperHandler(self, feedback):
    self.openGripperPR2()
    self.enablePps()

  ## Close the PR2 gripper for this arm and enable the PPS sensors (assumes this is running on the PR2).
  def closeGripperHandler(self, feedback):
    self.disablePps()
    self.closeGripperPR2()

  ## Zeroes the PR2 skin (eg, to correct for calibration errors).
  def zeroSkinHandler(self, feedback):
    self.zero_gripper_pub.publish(Empty())
    self.zero_gripper_right_link_pub.publish(Empty())
    self.zero_gripper_left_link_pub.publish(Empty())
    self.zero_gripper_palm_pub.publish(Empty())
    self.zero_forearm_pub.publish(Empty())
    self.zero_upperarm_pub.publish(Empty())
    self.zero_pps_left_pub.publish(Empty())
    self.zero_pps_right_pub.publish(Empty())

    self.zero_cody_meka_skin_pub.publish(Empty())
    self.zero_cody_fabric_forearm_pub.publish(Empty())
    self.zero_cody_fabric_wrist_pub.publish(Empty())

  def goal_feedback_rviz_cb(self, feedback):
#    print "goal_feedback_rviz"
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      #goal_pos_pub.publish(ps)
    self.server.applyChanges()

  def saveInitPoses(self, pose_msg):
    lforearm_ps = PoseStamped()
    lforearm_ps.header.frame_id = 'base_link'
    lforearm_ps.pose = pose_msg.lforearm_pose
    rforearm_ps = PoseStamped()
    rforearm_ps.header.frame_id = 'base_link'
    rforearm_ps.pose = pose_msg.rforearm_pose
    ragdoll_m1_ps = PoseStamped()
    ragdoll_m1_ps.header.frame_id = 'base_link'
    ragdoll_m1_ps.pose = pose_msg.ragdoll_m1_pose
    ragdoll_m2_ps = PoseStamped()
    ragdoll_m2_ps.header.frame_id = 'base_link'
    ragdoll_m2_ps.pose = pose_msg.ragdoll_m2_pose
    ragdoll_m3_ps = PoseStamped()
    ragdoll_m3_ps.header.frame_id = 'base_link'
    ragdoll_m3_ps.pose = pose_msg.ragdoll_m3_pose
    object_ps = PoseStamped()
    object_ps.header.frame_id = 'base_link'
    object_ps.pose = pose_msg.object_pose

    self.init_lforearm_ps = lforearm_ps
    self.init_rforearm_ps = rforearm_ps
    self.init_ragdoll_m1_ps = ragdoll_m1_ps
    self.init_ragdoll_m2_ps = ragdoll_m2_ps
    self.init_ragdoll_m3_ps = ragdoll_m3_ps
    self.init_object_ps = object_ps
    
    # update positions of interactive markers to follow the ragdoll
    self.init_poses_received = True
    self.server.setPose('object', object_ps.pose)
    #self.server.setPose('ragdoll_m1', ragdoll_m1_ps.pose)
    #self.server.setPose('ragdoll_m2', ragdoll_m2_ps.pose)
    #self.server.setPose('ragdoll_m3', ragdoll_m3_ps.pose)
    self.server.applyChanges()

    #self.poses_sub.unregister()
    #del(self.poses_sub) 

  def saveJointState(self, jtcs_msg):
    self.joint_state = jtcs_msg.actual.positions

  def larm_force_update(self,msg):
    self.larm_force[0] = msg.values_x[0]
    self.larm_force[1] = msg.values_y[0]
    self.larm_force[2] = msg.values_z[0]

  def rarm_force_update(self,msg):
    self.rarm_force[0] = msg.values_x[0]
    self.rarm_force[1] = msg.values_y[0]
    self.rarm_force[2] = msg.values_z[0]

  def publish_object_markers(self):
    self.platform_marker_pub.publish(self.platform_marker)
    self.ragdoll_markers.markers[0].pose = self.ragdoll_m1_pose 
    self.ragdoll_markers.markers[1].pose = self.ragdoll_m2_pose 
    self.ragdoll_markers.markers[2].pose = self.ragdoll_m3_pose 
    self.ragdoll_markers.markers[3].pose = self.ragdoll_lower_body_pose 
    self.ragdoll_markers.markers[4].pose = self.ragdoll_l_thigh_pose 
    self.ragdoll_markers.markers[5].pose = self.ragdoll_l_shin_pose
    self.ragdoll_markers.markers[6].pose = self.ragdoll_l_foot_pose
    self.ragdoll_markers.markers[7].pose = self.ragdoll_r_thigh_pose
    self.ragdoll_markers.markers[8].pose = self.ragdoll_r_shin_pose
    self.ragdoll_markers.markers[9].pose = self.ragdoll_r_foot_pose
    self.ragdoll_markers.markers[10].pose = self.ragdoll_middle_body_pose
    self.ragdoll_markers.markers[11].pose = self.ragdoll_upper_body_pose
    self.ragdoll_markers.markers[12].pose = self.ragdoll_l_arm_pose
    self.ragdoll_markers.markers[13].pose = self.ragdoll_l_wrist_pose
    self.ragdoll_markers.markers[14].pose = self.ragdoll_l_hand_pose
    self.ragdoll_markers.markers[15].pose = self.ragdoll_neck_pose
    self.ragdoll_markers.markers[16].pose = self.ragdoll_head_pose
    self.ragdoll_markers.markers[17].pose = self.ragdoll_r_arm_pose
    self.ragdoll_markers.markers[18].pose = self.ragdoll_r_wrist_pose
    self.ragdoll_markers.markers[19].pose = self.ragdoll_r_hand_pose
    self.ragdoll_markers_pub.publish(self.ragdoll_markers)

  def publish_ragdoll_random_wrench_marker(self):
    ragdoll_random_wrench_frame = copy.copy(self.ragdoll_random_wrench_frame)
    ragdoll_random_wrench_force_x = copy.copy(self.ragdoll_random_wrench_force_x)
    ragdoll_random_wrench_force_y = copy.copy(self.ragdoll_random_wrench_force_y)
    ragdoll_random_wrench_force_z = copy.copy(self.ragdoll_random_wrench_force_z)
    if ragdoll_random_wrench_frame != 'move_and_scoop':
    	self.ragdoll_random_wrench_marker.pose = eval('self.ragdoll_'+ragdoll_random_wrench_frame+'_pose')
    	self.ragdoll_random_wrench_marker_pub.publish(self.ragdoll_random_wrench_marker)
    
    	self.ragdoll_random_wrench_text_marker.pose.position = copy.copy(self.init_rforearm_ps.pose.position)
    	self.ragdoll_random_wrench_text_marker.pose.position.x = eval('self.ragdoll_'+ragdoll_random_wrench_frame+'_pose').position.x
    	self.ragdoll_random_wrench_text_marker.pose.position.y = eval('self.ragdoll_'+ragdoll_random_wrench_frame+'_pose').position.y
    	self.ragdoll_random_wrench_text_marker.pose.position.z = eval('self.ragdoll_'+ragdoll_random_wrench_frame+'_pose').position.z+0.4
    	self.ragdoll_random_wrench_text_marker.ns = 'ragdoll_random_wrench'
    	self.ragdoll_random_wrench_text_marker.scale.x = 0.2
    	self.ragdoll_random_wrench_text_marker.scale.y = 0.2
    	self.ragdoll_random_wrench_text_marker.scale.z = 0.2
    	self.ragdoll_random_wrench_text_marker.text = str(int(ragdoll_random_wrench_force_x))+', '+str(int(ragdoll_random_wrench_force_y))+', '+str(int(ragdoll_random_wrench_force_z))
    	self.ragdoll_random_wrench_text_marker_pub.publish(self.ragdoll_random_wrench_text_marker)    

    	self.ragdoll_random_wrench_text_marker2.pose.position = copy.copy(self.init_rforearm_ps.pose.position)
    	self.ragdoll_random_wrench_text_marker2.pose.position.x = eval('self.ragdoll_'+ragdoll_random_wrench_frame+'_pose').position.x
    	self.ragdoll_random_wrench_text_marker2.pose.position.y = eval('self.ragdoll_'+ragdoll_random_wrench_frame+'_pose').position.y
    	self.ragdoll_random_wrench_text_marker2.pose.position.z = eval('self.ragdoll_'+ragdoll_random_wrench_frame+'_pose').position.z+0.6
    	self.ragdoll_random_wrench_text_marker2.ns = 'ragdoll_random_wrench_frame'
    	self.ragdoll_random_wrench_text_marker2.scale.x = 0.2
    	self.ragdoll_random_wrench_text_marker2.scale.y = 0.2
    	self.ragdoll_random_wrench_text_marker2.scale.z = 0.2
    	self.ragdoll_random_wrench_text_marker2.text = ragdoll_random_wrench_frame
    	self.ragdoll_random_wrench_text_marker2_pub.publish(self.ragdoll_random_wrench_text_marker2)
    

  def publish_force_markers(self):
    self.larm_force_markers.markers[0].pose.position = self.init_lforearm_ps.pose.position
    #self.larm_force[0] = 1
    #self.larm_force[1] = 1
    #self.larm_force[2] = 1
    self.larm_force_markers.markers[0].scale.x = 0.75
    self.larm_force_markers.markers[0].scale.y = 0.75
    self.larm_force_markers.markers[0].scale.z = 0.005*self.larm_force[0]
    self.larm_force_markers.markers[0].ns = 'larm_x'
    self.larm_force_markers.markers[3].pose.position = copy.copy(self.init_lforearm_ps.pose.position)
    #self.larm_force_markers.markers[3].pose.position.x = self.larm_force_markers.markers[3].pose.position.x+(0.01)*abs(self.larm_force[0])+0.5
    self.larm_force_markers.markers[3].pose.position.x = self.larm_force_markers.markers[3].pose.position.x+0.3
    self.larm_force_markers.markers[3].pose.position.y = self.larm_force_markers.markers[3].pose.position.y+0.2
    self.larm_force_markers.markers[3].ns = 'larm_text_x'
    self.larm_force_markers.markers[3].scale.x = 0.2
    self.larm_force_markers.markers[3].scale.y = 0.2
    self.larm_force_markers.markers[3].scale.z = 0.2
    self.larm_force_markers.markers[3].text = str(int(self.larm_force[0]))

    self.larm_force_markers.markers[1].pose.position = self.init_lforearm_ps.pose.position
    self.larm_force_markers.markers[1].scale.x = 0.75
    self.larm_force_markers.markers[1].scale.y = 0.75
    self.larm_force_markers.markers[1].scale.z = 0.005*self.larm_force[1]
    self.larm_force_markers.markers[1].ns = 'larm_y'
    self.larm_force_markers.markers[4].pose.position = copy.copy(self.init_lforearm_ps.pose.position)
    #self.larm_force_markers.markers[4].pose.position.y = self.larm_force_markers.markers[4].pose.position.y+(0.01)*abs(self.larm_force[1])+0.5
    self.larm_force_markers.markers[4].pose.position.x = self.larm_force_markers.markers[3].pose.position.x
    self.larm_force_markers.markers[4].pose.position.y = self.larm_force_markers.markers[3].pose.position.y
    self.larm_force_markers.markers[4].pose.position.z = self.larm_force_markers.markers[3].pose.position.z+0.2
    self.larm_force_markers.markers[4].ns = 'larm_text_y'
    self.larm_force_markers.markers[4].scale.x = 0.2
    self.larm_force_markers.markers[4].scale.y = 0.2
    self.larm_force_markers.markers[4].scale.z = 0.2
    self.larm_force_markers.markers[4].text = str(int(self.larm_force[1]))

    self.larm_force_markers.markers[2].pose.position = self.init_lforearm_ps.pose.position
    self.larm_force_markers.markers[2].scale.x = 0.75
    self.larm_force_markers.markers[2].scale.y = 0.75
    self.larm_force_markers.markers[2].scale.z = 0.005*self.larm_force[2]
    self.larm_force_markers.markers[2].ns = 'larm_z'
    self.larm_force_markers.markers[5].pose.position = copy.copy(self.init_lforearm_ps.pose.position)
    self.larm_force_markers.markers[5].pose.position.x = self.larm_force_markers.markers[3].pose.position.x
    self.larm_force_markers.markers[5].pose.position.y = self.larm_force_markers.markers[3].pose.position.y
    self.larm_force_markers.markers[5].pose.position.z = self.larm_force_markers.markers[3].pose.position.z+0.4
    self.larm_force_markers.markers[5].ns = 'larm_text_z'
    self.larm_force_markers.markers[5].scale.x = 0.2
    self.larm_force_markers.markers[5].scale.y = 0.2
    self.larm_force_markers.markers[5].scale.z = 0.2
    self.larm_force_markers.markers[5].text = str(int(self.larm_force[2]))

    self.rarm_force_markers.markers[0].pose.position = self.init_rforearm_ps.pose.position
    self.rarm_force_markers.markers[0].scale.x = 0.75
    self.rarm_force_markers.markers[0].scale.y = 0.75
    self.rarm_force_markers.markers[0].scale.z = 0.005*self.rarm_force[0]
    self.rarm_force_markers.markers[0].ns = 'rarm_x'
    self.rarm_force_markers.markers[3].pose.position = copy.copy(self.init_rforearm_ps.pose.position)
    self.rarm_force_markers.markers[3].pose.position.x = self.rarm_force_markers.markers[3].pose.position.x+0.3
    self.rarm_force_markers.markers[3].pose.position.y = self.rarm_force_markers.markers[3].pose.position.y-0.2
    self.rarm_force_markers.markers[3].ns = 'rarm_text_x'
    self.rarm_force_markers.markers[3].scale.x = 0.2
    self.rarm_force_markers.markers[3].scale.y = 0.2
    self.rarm_force_markers.markers[3].scale.z = 0.2
    self.rarm_force_markers.markers[3].text = str(int(self.rarm_force[0]))

    self.rarm_force_markers.markers[1].pose.position = self.init_rforearm_ps.pose.position
    self.rarm_force_markers.markers[1].scale.x = 0.75
    self.rarm_force_markers.markers[1].scale.y = 0.75
    self.rarm_force_markers.markers[1].scale.z = 0.005*self.rarm_force[1]
    self.rarm_force_markers.markers[1].ns = 'rarm_y'
    self.rarm_force_markers.markers[4].pose.position = copy.copy(self.init_rforearm_ps.pose.position)
    self.rarm_force_markers.markers[4].pose.position.x = self.rarm_force_markers.markers[3].pose.position.x
    self.rarm_force_markers.markers[4].pose.position.y = self.rarm_force_markers.markers[3].pose.position.y
    self.rarm_force_markers.markers[4].pose.position.z = self.rarm_force_markers.markers[3].pose.position.z+0.2
    self.rarm_force_markers.markers[4].ns = 'rarm_text_y'
    self.rarm_force_markers.markers[4].scale.x = 0.2
    self.rarm_force_markers.markers[4].scale.y = 0.2
    self.rarm_force_markers.markers[4].scale.z = 0.2
    self.rarm_force_markers.markers[4].text = str(int(self.rarm_force[1]))

    self.rarm_force_markers.markers[2].pose.position = self.init_rforearm_ps.pose.position
    self.rarm_force_markers.markers[2].scale.x = 0.75
    self.rarm_force_markers.markers[2].scale.y = 0.75
    self.rarm_force_markers.markers[2].scale.z = 0.005*self.rarm_force[2]
    self.rarm_force_markers.markers[2].ns = 'rarm_z'
    self.rarm_force_markers.markers[5].pose.position = copy.copy(self.init_rforearm_ps.pose.position)
    self.rarm_force_markers.markers[5].pose.position.x = self.rarm_force_markers.markers[3].pose.position.x
    self.rarm_force_markers.markers[5].pose.position.y = self.rarm_force_markers.markers[3].pose.position.y
    self.rarm_force_markers.markers[5].pose.position.z = self.rarm_force_markers.markers[3].pose.position.z+0.4
    self.rarm_force_markers.markers[5].ns = 'rarm_text_z'
    self.rarm_force_markers.markers[5].scale.x = 0.2
    self.rarm_force_markers.markers[5].scale.y = 0.2
    self.rarm_force_markers.markers[5].scale.z = 0.2
    self.rarm_force_markers.markers[5].text = str(int(self.rarm_force[2]))

    self.larm_force_markers_pub.publish(self.larm_force_markers) 
    self.rarm_force_markers_pub.publish(self.rarm_force_markers) 
    
    m = Marker()
    m.header.frame_id = 'base_link'
    m.ns = 'goal'
    m.type = 0
    m.action = 0
    m.pose.position.x = 1
    m.pose.position.y = 1
    m.pose.position.z = 1
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1.
    m.scale.x = 1
    m.scale.y = 1
    m.scale.z = .1
    m.color.r = 0
    m.color.g = 1
    m.color.b = 0
    m.color.a = 1
    self.test_marker_pub.publish(m)

  def update_state(self,msg):
    self.base_ja = [msg.desired_position[0],msg.desired_position[1],np.degrees(msg.desired_position[2]).tolist()]
    self.torso_ja = [np.degrees(msg.desired_position[3]).tolist()]
    self.head_ja = [np.degrees(msg.desired_position[4]).tolist()]
    self.larm_ja = np.degrees(msg.desired_position[5:11]).tolist()
    self.rarm_ja = np.degrees(msg.desired_position[11:17]).tolist()

  def update_ragdoll_random_wrench(self,msg):
    self.ragdoll_random_wrench_frame = msg.frame_names[0]
    self.ragdoll_random_wrench_force_x = msg.force_x[0]
    self.ragdoll_random_wrench_force_y = msg.force_y[0]
    self.ragdoll_random_wrench_force_z = msg.force_z[0]

  def update_ragdoll_state(self,msg):
    self.ragdoll_m1_pose.position.x = msg.centers_x[2]
    self.ragdoll_m1_pose.position.y = msg.centers_y[2]
    self.ragdoll_m1_pose.position.z = msg.centers_z[2]
    self.ragdoll_m1_pose.orientation.x = msg.rotation_x[2]
    self.ragdoll_m1_pose.orientation.y = msg.rotation_y[2]
    self.ragdoll_m1_pose.orientation.z = msg.rotation_z[2]
    self.ragdoll_m1_pose.orientation.w = msg.rotation_w[2]

    self.ragdoll_m2_pose.position.x = msg.centers_x[3]
    self.ragdoll_m2_pose.position.y = msg.centers_y[3]
    self.ragdoll_m2_pose.position.z = msg.centers_z[3]
    self.ragdoll_m2_pose.orientation.x = msg.rotation_x[3]
    self.ragdoll_m2_pose.orientation.y = msg.rotation_y[3]
    self.ragdoll_m2_pose.orientation.z = msg.rotation_z[3]
    self.ragdoll_m2_pose.orientation.w = msg.rotation_w[3]

    self.ragdoll_m3_pose.position.x = msg.centers_x[4]
    self.ragdoll_m3_pose.position.y = msg.centers_y[4]
    self.ragdoll_m3_pose.position.z = msg.centers_z[4]
    self.ragdoll_m3_pose.orientation.x = msg.rotation_x[4]
    self.ragdoll_m3_pose.orientation.y = msg.rotation_y[4]
    self.ragdoll_m3_pose.orientation.z = msg.rotation_z[4]
    self.ragdoll_m3_pose.orientation.w = msg.rotation_w[4]

  def update_ragdoll_link_states(self,msg):
    self.ragdoll_lower_body_pose.position.x = msg.centers_x[0]
    self.ragdoll_lower_body_pose.position.y = msg.centers_y[0]
    self.ragdoll_lower_body_pose.position.z = msg.centers_z[0]
    self.ragdoll_lower_body_pose.orientation.x = msg.rotation_x[0]
    self.ragdoll_lower_body_pose.orientation.y = msg.rotation_y[0]
    self.ragdoll_lower_body_pose.orientation.z = msg.rotation_z[0]
    self.ragdoll_lower_body_pose.orientation.w = msg.rotation_w[0]

    self.ragdoll_l_thigh_pose.position.x = msg.centers_x[1]
    self.ragdoll_l_thigh_pose.position.y = msg.centers_y[1]
    self.ragdoll_l_thigh_pose.position.z = msg.centers_z[1]
    self.ragdoll_l_thigh_pose.orientation.x = msg.rotation_x[1]
    self.ragdoll_l_thigh_pose.orientation.y = msg.rotation_y[1]
    self.ragdoll_l_thigh_pose.orientation.z = msg.rotation_z[1]
    self.ragdoll_l_thigh_pose.orientation.w = msg.rotation_w[1]

    self.ragdoll_l_shin_pose.position.x = msg.centers_x[2]
    self.ragdoll_l_shin_pose.position.y = msg.centers_y[2]
    self.ragdoll_l_shin_pose.position.z = msg.centers_z[2]
    self.ragdoll_l_shin_pose.orientation.x = msg.rotation_x[2]
    self.ragdoll_l_shin_pose.orientation.y = msg.rotation_y[2]
    self.ragdoll_l_shin_pose.orientation.z = msg.rotation_z[2]
    self.ragdoll_l_shin_pose.orientation.w = msg.rotation_w[2]

    self.ragdoll_l_foot_pose.position.x = msg.centers_x[3]
    self.ragdoll_l_foot_pose.position.y = msg.centers_y[3]
    self.ragdoll_l_foot_pose.position.z = msg.centers_z[3]
    self.ragdoll_l_foot_pose.orientation.x = msg.rotation_x[3]
    self.ragdoll_l_foot_pose.orientation.y = msg.rotation_y[3]
    self.ragdoll_l_foot_pose.orientation.z = msg.rotation_z[3]
    self.ragdoll_l_foot_pose.orientation.w = msg.rotation_w[3]

    self.ragdoll_r_thigh_pose.position.x = msg.centers_x[4]
    self.ragdoll_r_thigh_pose.position.y = msg.centers_y[4]
    self.ragdoll_r_thigh_pose.position.z = msg.centers_z[4]
    self.ragdoll_r_thigh_pose.orientation.x = msg.rotation_x[4]
    self.ragdoll_r_thigh_pose.orientation.y = msg.rotation_y[4]
    self.ragdoll_r_thigh_pose.orientation.z = msg.rotation_z[4]
    self.ragdoll_r_thigh_pose.orientation.w = msg.rotation_w[4]

    self.ragdoll_r_shin_pose.position.x = msg.centers_x[5]
    self.ragdoll_r_shin_pose.position.y = msg.centers_y[5]
    self.ragdoll_r_shin_pose.position.z = msg.centers_z[5]
    self.ragdoll_r_shin_pose.orientation.x = msg.rotation_x[5]
    self.ragdoll_r_shin_pose.orientation.y = msg.rotation_y[5]
    self.ragdoll_r_shin_pose.orientation.z = msg.rotation_z[5]
    self.ragdoll_r_shin_pose.orientation.w = msg.rotation_w[5]

    self.ragdoll_r_foot_pose.position.x = msg.centers_x[6]
    self.ragdoll_r_foot_pose.position.y = msg.centers_y[6]
    self.ragdoll_r_foot_pose.position.z = msg.centers_z[6]
    self.ragdoll_r_foot_pose.orientation.x = msg.rotation_x[6]
    self.ragdoll_r_foot_pose.orientation.y = msg.rotation_y[6]
    self.ragdoll_r_foot_pose.orientation.z = msg.rotation_z[6]
    self.ragdoll_r_foot_pose.orientation.w = msg.rotation_w[6]

    self.ragdoll_middle_body_pose.position.x = msg.centers_x[7]
    self.ragdoll_middle_body_pose.position.y = msg.centers_y[7]
    self.ragdoll_middle_body_pose.position.z = msg.centers_z[7]
    self.ragdoll_middle_body_pose.orientation.x = msg.rotation_x[7]
    self.ragdoll_middle_body_pose.orientation.y = msg.rotation_y[7]
    self.ragdoll_middle_body_pose.orientation.z = msg.rotation_z[7]
    self.ragdoll_middle_body_pose.orientation.w = msg.rotation_w[7]

    self.ragdoll_upper_body_pose.position.x = msg.centers_x[8]
    self.ragdoll_upper_body_pose.position.y = msg.centers_y[8]
    self.ragdoll_upper_body_pose.position.z = msg.centers_z[8]
    self.ragdoll_upper_body_pose.orientation.x = msg.rotation_x[8]
    self.ragdoll_upper_body_pose.orientation.y = msg.rotation_y[8]
    self.ragdoll_upper_body_pose.orientation.z = msg.rotation_z[8]
    self.ragdoll_upper_body_pose.orientation.w = msg.rotation_w[8]

    self.ragdoll_l_arm_pose.position.x = msg.centers_x[9]
    self.ragdoll_l_arm_pose.position.y = msg.centers_y[9]
    self.ragdoll_l_arm_pose.position.z = msg.centers_z[9]
    self.ragdoll_l_arm_pose.orientation.x = msg.rotation_x[9]
    self.ragdoll_l_arm_pose.orientation.y = msg.rotation_y[9]
    self.ragdoll_l_arm_pose.orientation.z = msg.rotation_z[9]
    self.ragdoll_l_arm_pose.orientation.w = msg.rotation_w[9]
    
    self.ragdoll_l_wrist_pose.position.x = msg.centers_x[10]
    self.ragdoll_l_wrist_pose.position.y = msg.centers_y[10]
    self.ragdoll_l_wrist_pose.position.z = msg.centers_z[10]
    self.ragdoll_l_wrist_pose.orientation.x = msg.rotation_x[10]
    self.ragdoll_l_wrist_pose.orientation.y = msg.rotation_y[10]
    self.ragdoll_l_wrist_pose.orientation.z = msg.rotation_z[10]
    self.ragdoll_l_wrist_pose.orientation.w = msg.rotation_w[10]

    self.ragdoll_l_hand_pose.position.x = msg.centers_x[11]
    self.ragdoll_l_hand_pose.position.y = msg.centers_y[11]
    self.ragdoll_l_hand_pose.position.z = msg.centers_z[11]
    self.ragdoll_l_hand_pose.orientation.x = msg.rotation_x[11]
    self.ragdoll_l_hand_pose.orientation.y = msg.rotation_y[11]
    self.ragdoll_l_hand_pose.orientation.z = msg.rotation_z[11]
    self.ragdoll_l_hand_pose.orientation.w = msg.rotation_w[11]
    
    self.ragdoll_neck_pose.position.x = msg.centers_x[12]
    self.ragdoll_neck_pose.position.y = msg.centers_y[12]
    self.ragdoll_neck_pose.position.z = msg.centers_z[12]
    self.ragdoll_neck_pose.orientation.x = msg.rotation_x[12]
    self.ragdoll_neck_pose.orientation.y = msg.rotation_y[12]
    self.ragdoll_neck_pose.orientation.z = msg.rotation_z[12]
    self.ragdoll_neck_pose.orientation.w = msg.rotation_w[12]

    self.ragdoll_head_pose.position.x = msg.centers_x[13]
    self.ragdoll_head_pose.position.y = msg.centers_y[13]
    self.ragdoll_head_pose.position.z = msg.centers_z[13]
    self.ragdoll_head_pose.orientation.x = msg.rotation_x[13]
    self.ragdoll_head_pose.orientation.y = msg.rotation_y[13]
    self.ragdoll_head_pose.orientation.z = msg.rotation_z[13]
    self.ragdoll_head_pose.orientation.w = msg.rotation_w[13]

    self.ragdoll_r_arm_pose.position.x = msg.centers_x[14]
    self.ragdoll_r_arm_pose.position.y = msg.centers_y[14]
    self.ragdoll_r_arm_pose.position.z = msg.centers_z[14]
    self.ragdoll_r_arm_pose.orientation.x = msg.rotation_x[14]
    self.ragdoll_r_arm_pose.orientation.y = msg.rotation_y[14]
    self.ragdoll_r_arm_pose.orientation.z = msg.rotation_z[14]
    self.ragdoll_r_arm_pose.orientation.w = msg.rotation_w[14]
   
    self.ragdoll_r_wrist_pose.position.x = msg.centers_x[15]
    self.ragdoll_r_wrist_pose.position.y = msg.centers_y[15]
    self.ragdoll_r_wrist_pose.position.z = msg.centers_z[15]
    self.ragdoll_r_wrist_pose.orientation.x = msg.rotation_x[15]
    self.ragdoll_r_wrist_pose.orientation.y = msg.rotation_y[15]
    self.ragdoll_r_wrist_pose.orientation.z = msg.rotation_z[15]
    self.ragdoll_r_wrist_pose.orientation.w = msg.rotation_w[15]

    self.ragdoll_r_hand_pose.position.x = msg.centers_x[16]
    self.ragdoll_r_hand_pose.position.y = msg.centers_y[16]
    self.ragdoll_r_hand_pose.position.z = msg.centers_z[16]
    self.ragdoll_r_hand_pose.orientation.x = msg.rotation_x[16]
    self.ragdoll_r_hand_pose.orientation.y = msg.rotation_y[16]
    self.ragdoll_r_hand_pose.orientation.z = msg.rotation_z[16]
    self.ragdoll_r_hand_pose.orientation.w = msg.rotation_w[16]


  ## Initialise all publishers/subscribers
  def initComms(self, node_name):
    rospy.init_node(node_name)

    rospy.Subscriber('crona/sim_state',CronaState,self.update_state)
    #Initial hand configuration Subscriber
    self.poses_sub = rospy.Subscriber('haptic_mpc/robot_state', haptic_msgs.RobotHapticState, self.saveInitPoses)
    # Goal pose publisher.
    self.goal_pos_pub = rospy.Publisher("haptic_mpc/traj_pose", GoalPose)
    self.mpc_weights_pub = rospy.Publisher("haptic_mpc/weights", haptic_msgs.HapticMpcWeights)
    self.larm_force_markers_pub = rospy.Publisher("haptic_mpc/larm_force_markers",MarkerArray)
    self.ragdoll_random_wrench_marker_pub = rospy.Publisher("haptic_mpc/ragdoll_random_wrench_marker",Marker)
    self.ragdoll_random_wrench_text_marker_pub = rospy.Publisher("haptic_mpc/ragdoll_random_wrench_text_marker",Marker)
    self.ragdoll_random_wrench_text_marker2_pub = rospy.Publisher("haptic_mpc/ragdoll_random_wrench_text_marker2",Marker)
    self.platform_marker_pub = rospy.Publisher("haptic_mpc/platform_marker",Marker)
    self.ragdoll_markers_pub = rospy.Publisher("haptic_mpc/ragdoll_markers",MarkerArray)
    self.apply_random_wrench_pub = rospy.Publisher("/gazebo/apply_random_wrench",Bool)
    self.test_marker_pub = rospy.Publisher("haptic_mpc/test_marker",Marker)
    self.rarm_force_markers_pub = rospy.Publisher("haptic_mpc/rarm_force_markers",MarkerArray)
 
    rospy.Subscriber('/l_arm/crona/est_force',haptic_msgs.TaxelArray,self.larm_force_update)
    rospy.Subscriber('/r_arm/crona/est_force',haptic_msgs.TaxelArray,self.rarm_force_update)
    rospy.Subscriber('crona/sim_state',CronaState,self.update_state)
    rospy.Subscriber('gazebo/objectcog',RagdollObjectArray,self.update_ragdoll_state)
    rospy.Subscriber('gazebo/ragdollcog',RagdollObjectArray,self.update_ragdoll_link_states)
    rospy.Subscriber("/gazebo/ragdoll_wrench",WrenchArray,self.update_ragdoll_random_wrench)
    self.server = ims.InteractiveMarkerServer('teleop_rviz_server')

  ## Initialise the interactive marker based on what robot we're running on, and whether we use orientation or just position.
  def initMarkers(self):
    while not self.init_poses_received and not rospy.is_shutdown():
      rospy.sleep(1)
      rospy.loginfo("[mpc_teleops_rviz.py] Waiting for initial end effector pose")

    #self.lforearm_im = imu.make_6dof_marker(False, self.init_lforearm_pose, 0.5, (1., 1., 0.,0.4), 'sphere')
    #self.rforearm_im = imu.make_6dof_marker(False, self.init_rforearm_pose, 0.5, (1., 1., 0.,0.4), 'sphere')

    # set initial position of interactive markers
   
    self.init_ragdoll_m1_ps = PoseStamped()
    self.init_ragdoll_m2_ps = PoseStamped()
    self.init_ragdoll_m3_ps = PoseStamped()

    self.init_ragdoll_m1_ps.header.frame_id = 'world'
    self.init_ragdoll_m1_ps.pose.position.x = 3  
    self.init_ragdoll_m1_ps.pose.position.y = -1 
    self.init_ragdoll_m1_ps.pose.position.z = 0.5 

    self.init_ragdoll_m2_ps.header.frame_id = 'world'
    self.init_ragdoll_m2_ps.pose.position.x = 3
    self.init_ragdoll_m2_ps.pose.position.y = -0.5
    self.init_ragdoll_m2_ps.pose.position.z = 0.5

    self.init_ragdoll_m3_ps.header.frame_id = 'world'
    self.init_ragdoll_m3_ps.pose.position.x = 3
    self.init_ragdoll_m3_ps.pose.position.y = 0.5
    self.init_ragdoll_m3_ps.pose.position.z = 0.5
    

    init_platform_1_ps = PoseStamped()
    init_platform_1_ps.header.frame_id = 'world'
    init_platform_1_ps.pose.position.x = -3
    init_platform_1_ps.pose.position.y = -1
    init_platform_1_ps.pose.position.z = 0.5    

    init_platform_2_ps = PoseStamped()
    init_platform_2_ps.header.frame_id = 'world'
    init_platform_2_ps.pose.position.x = -3
    init_platform_2_ps.pose.position.y = -0.5
    init_platform_2_ps.pose.position.z = 0.5    

    init_platform_3_ps = PoseStamped()
    init_platform_3_ps.header.frame_id = 'world'
    init_platform_3_ps.pose.position.x = -3
    init_platform_3_ps.pose.position.y = 0.5
    init_platform_3_ps.pose.position.z = 0.5

    init_platform_4_ps = PoseStamped()
    init_platform_4_ps.header.frame_id = 'world'
    init_platform_4_ps.pose.position.x = -3
    init_platform_4_ps.pose.position.y = 1
    init_platform_4_ps.pose.position.z = 0.5

    self.ragdoll_m1_im = imu.make_3dof_marker_position(self.init_ragdoll_m1_ps, 0.5, (1., 1., 0.,0.4), 'sphere')
    self.ragdoll_m2_im = imu.make_3dof_marker_position(self.init_ragdoll_m2_ps, 0.5, (1., 1., 0.,0.4), 'sphere')
    self.ragdoll_m3_im = imu.make_3dof_marker_position(self.init_ragdoll_m3_ps, 0.5, (1., 1., 0.,0.4), 'sphere')
    self.object_im = imu.make_3dof_marker_position(self.init_object_ps, 0.5, (1., 1., 0.,0.4), 'sphere')

    self.platform_1_im = imu.make_3dof_marker_position(init_platform_1_ps, 0.5, (1., 1., 1., 0.5), 'sphere')
    self.platform_2_im = imu.make_3dof_marker_position(init_platform_2_ps, 0.5, (1., 1., 1., 0.5), 'sphere')
    self.platform_3_im = imu.make_3dof_marker_position(init_platform_3_ps, 0.5, (1., 1., 1., 0.5), 'sphere')
    self.platform_4_im = imu.make_3dof_marker_position(init_platform_4_ps, 0.5, (1., 1., 1., 0.5), 'sphere')

    #lforearm_ps = PoseStamped()
    #lforearm_ps.header = self.lforearm_im.header
    #lforearm_ps.pose = self.lforearm_im.pose
    #self.current_lforearm_goal_pose = lforearm_ps

    #rforearm_ps = PoseStamped()
    #rforearm_ps.header = self.rforearm_im.header
    #rforearm_ps.pose = self.rforearm_im.pose
    #self.current_rforearm_goal_pose = rforearm_ps

    ragdoll_m1_ps = PoseStamped()
    ragdoll_m1_ps.header = self.ragdoll_m1_im.header
    ragdoll_m1_ps.pose = self.ragdoll_m1_im.pose
    self.current_ragdoll_m1_goal_pose = ragdoll_m1_ps

    ragdoll_m2_ps = PoseStamped()
    ragdoll_m2_ps.header = self.ragdoll_m2_im.header
    ragdoll_m2_ps.pose = self.ragdoll_m2_im.pose
    self.current_ragdoll_m2_goal_pose = ragdoll_m2_ps

    ragdoll_m3_ps = PoseStamped()
    ragdoll_m3_ps.header = self.ragdoll_m3_im.header
    ragdoll_m3_ps.pose = self.ragdoll_m3_im.pose
    self.current_ragdoll_m3_goal_pose = ragdoll_m3_ps

    object_ps = PoseStamped()
    object_ps.header = self.object_im.header
    object_ps.pose = self.object_im.pose
    self.current_object_goal_pose = object_ps

    platform_1_ps = PoseStamped()
    platform_1_ps.header = self.platform_1_im.header
    platform_1_ps.pose = self.platform_1_im.pose
    self.current_platform_1_goal_pose = platform_1_ps

    platform_2_ps = PoseStamped()
    platform_2_ps.header = self.platform_2_im.header
    platform_2_ps.pose = self.platform_2_im.pose
    self.current_platform_2_goal_pose = platform_2_ps

    platform_3_ps = PoseStamped()
    platform_3_ps.header = self.platform_3_im.header
    platform_3_ps.pose = self.platform_3_im.pose
    self.current_platform_3_goal_pose = platform_3_ps

    platform_4_ps = PoseStamped()
    platform_4_ps.header = self.platform_4_im.header
    platform_4_ps.pose = self.platform_4_im.pose
    self.current_platform_4_goal_pose = platform_4_ps

    #self.lforearm_im.name = 'lforearm'
    #self.lforearm_im.description = 'lforearm'
    #self.server.insert(self.lforearm_im, self.lforearm_im_LocationCallback)

    #self.rforearm_im.name = 'rforearm'
    #self.rforearm_im.description = 'rforearm'
    #self.server.insert(self.rforearm_im, self.rforearm_im_LocationCallback)

    self.ragdoll_m1_im.name = 'ragdoll_m1'
    self.ragdoll_m1_im.description = 'ragdoll_m1'
    self.server.insert(self.ragdoll_m1_im, self.ragdoll_m1_im_LocationCallback)

    self.ragdoll_m2_im.name = 'ragdoll_m2'
    self.ragdoll_m2_im.description = 'ragdoll_m2'
    self.server.insert(self.ragdoll_m2_im, self.ragdoll_m2_im_LocationCallback)

    self.ragdoll_m3_im.name = 'ragdoll_m3'
    self.ragdoll_m3_im.description = 'ragdoll_m3'
    self.server.insert(self.ragdoll_m3_im, self.ragdoll_m3_im_LocationCallback)

    self.object_im.name = 'object'
    self.object_im.description = 'object'
    self.server.insert(self.object_im, self.object_im_LocationCallback)

    self.platform_1_im.name = 'platform_1'
    self.platform_1_im.description = 'platform_1'
    self.server.insert(self.platform_1_im, self.platform_1_im_LocationCallback)

    self.platform_2_im.name = 'platform_2'
    self.platform_2_im.description = 'platform_2'
    self.server.insert(self.platform_2_im, self.platform_2_im_LocationCallback)

    self.platform_3_im.name = 'platform_3'
    self.platform_3_im.description = 'platform_3'
    self.server.insert(self.platform_3_im, self.platform_3_im_LocationCallback)

    self.platform_4_im.name = 'platform_4'
    self.platform_4_im.description = 'platform_4'
    self.server.insert(self.platform_4_im, self.platform_4_im_LocationCallback)

    self.server.applyChanges()

   #-------- gripper functions ------------
  def moveGripperPR2(self, dist=0.08, effort = 15):
    self.gripper_action_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position=dist, max_effort = effort)))

  def openGripperPR2(self, dist=0.08):
    self.moveGripperPR2(dist, -1)

  def closeGripperPR2(self, dist=0., effort = 15):
    self.moveGripperPR2(dist, effort)

  def continuousUpdateHandler(self, feedback):
      self.continuous_update =  not self.continuous_update

  ## Initialise the menu used to control the arm behaviour.
  def initMenu(self):
    #self.lforearm_menu_handler = mh.MenuHandler()
    #self.lforearm_menu_handler.insert('Go', callback = self.lforearm_goalHandler)
    #imu.add_menu_handler(self.lforearm_im, self.lforearm_menu_handler, self.server) 

    #self.rforearm_menu_handler = mh.MenuHandler()
    #self.rforearm_menu_handler.insert('Go', callback = self.rforearm_goalHandler)
    #imu.add_menu_handler(self.rforearm_im, self.rforearm_menu_handler, self.server)

    self.ragdoll_m1_menu_handler = mh.MenuHandler()
    self.ragdoll_m1_menu_handler.insert('Mark Position', callback = self.ragdoll_m1_position)
    imu.add_menu_handler(self.ragdoll_m1_im, self.ragdoll_m1_menu_handler, self.server)

    self.ragdoll_m2_menu_handler = mh.MenuHandler()
    self.ragdoll_m2_menu_handler.insert('Mark Position', callback = self.ragdoll_m2_position)
    imu.add_menu_handler(self.ragdoll_m2_im, self.ragdoll_m2_menu_handler, self.server)

    self.ragdoll_m3_menu_handler = mh.MenuHandler()
    self.ragdoll_m3_menu_handler.insert('Mark Position', callback = self.ragdoll_m3_position)
    imu.add_menu_handler(self.ragdoll_m3_im, self.ragdoll_m3_menu_handler, self.server)

    self.object_menu_handler = mh.MenuHandler()
    #self.object_menu_handler.insert('Object Scoop', callback = self.object_goalHandler)
    self.object_menu_handler.insert('cRoNA2 Lower Down', callback = self.crona2_lower_down)
    self.object_menu_handler.insert('cRoNA2 Lift to Set Height', callback = self.crona2_lift_set_height)
    #self.object_menu_handler.insert('Object Lift', callback = self.object_com_lift)
    self.object_menu_handler.insert('cRoNA2 Move to Platform', callback = self.crona2_move_to_platform)
    self.object_menu_handler.insert('cRoNA2 Place', callback = self.crona2_place)
    self.object_menu_handler.insert('cRoNA2 Place2', callback = self.crona2_place2)
    self.object_menu_handler.insert('cRoNA2 Release Ragdoll', callback = self.crona2_release)
    self.object_menu_handler.insert('cRoNA2 Return Home', callback = self.crona2_return_home)
    self.object_menu_handler.insert('Apply Random Wrench to Ragdoll', callback = self.apply_random_wrench)
    imu.add_menu_handler(self.object_im, self.object_menu_handler, self.server)

    self.platform_1_menu_handler = mh.MenuHandler()
    self.platform_1_menu_handler.insert('Mark Position', callback = self.platform_1_position)
    imu.add_menu_handler(self.platform_1_im, self.platform_1_menu_handler, self.server)

    self.platform_2_menu_handler = mh.MenuHandler()
    self.platform_2_menu_handler.insert('Mark Position', callback = self.platform_2_position)
    imu.add_menu_handler(self.platform_2_im, self.platform_2_menu_handler, self.server)

    self.platform_3_menu_handler = mh.MenuHandler()
    self.platform_3_menu_handler.insert('Mark Position', callback = self.platform_3_position)
    imu.add_menu_handler(self.platform_3_im, self.platform_3_menu_handler, self.server)
 
    self.platform_4_menu_handler = mh.MenuHandler()
    self.platform_4_menu_handler.insert('Mark Position', callback = self.platform_4_position)
    imu.add_menu_handler(self.platform_4_im, self.platform_4_menu_handler, self.server)

  ## Start the interactive marker server (spin indefinitely).
  def start(self):
    mpc_ims.initComms("mpc_teleop_rviz")
    mpc_ims.initMarkers()
    mpc_ims.initMenu()
    rospy.loginfo('Haptic MPC interactive marker server started')
    while not rospy.is_shutdown():
      self.publish_ragdoll_random_wrench_marker()
      self.publish_force_markers()
      self.publish_object_markers()

if __name__ == '__main__':
  # Parse an options list specifying robot type
  #import optparse
  #p = optparse.OptionParser()
  #haptic_mpc_util.initialiseOptParser(p)
  #opt = haptic_mpc_util.getValidInput(p)

  # Initialise publishers/subscribers
  mpc_ims = MPCTeleopInteractiveMarkers()
  mpc_ims.start()
