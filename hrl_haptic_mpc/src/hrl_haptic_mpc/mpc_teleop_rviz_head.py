#!/usr/bin/env python

#   Copyright 2013 Georgia Tech Research Corporation
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
#  http://healthcare-robotics.com/

## @package hrl_haptic_mpc
# 
# @author Jeff Hawke 
# @version 0.1
# @copyright Apache 2.0

import math, numpy as np
import sys, optparse

import interactive_marker_util as imu

import roslib; roslib.load_manifest('hrl_haptic_mpc')
import rospy

import hrl_lib.transforms as tr
import haptic_mpc_util
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs

import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerFeedback, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray, Point, Quaternion
from std_msgs.msg import String, Bool, Empty, ColorRGBA
from pr2_controllers_msgs.msg import Pr2GripperCommandGoal, Pr2GripperCommandAction, Pr2GripperCommand, JointTrajectoryControllerState as JTCS
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionIKRequest
from kinematics_msgs.msg import PositionIKRequest

## RViz teleop interface to the controller. 
#
# Publishes goal poses on the appropriate topics and allows a user to teleop the arm controller.
class MPCTeleopInteractiveMarkers():
  def __init__(self, opt):
    self.opt = opt
    base_path = 'haptic_mpc'
    control_path = '/control_params'
    self.orient_weight = rospy.get_param(base_path + control_path + '/orientation_weight', 0)
    self.pos_weight = rospy.get_param(base_path + control_path + '/position_weight', 0)
    self.posture_weight = 1.0#rospy.get_param(base_path + control_path + '/posture_weight')    
    self.arm = opt.arm
    self.init_ee_pose_received = False
    self.continuous_update = False

  def _updateIKFeasibility(self, ps):
    p = PoseStamped()
    p.pose.orientation = ps.pose.orientation
    p.header.frame_id = '/torso_lift_link'
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
  def interactiveMarkerLocationCallback(self, feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
      ps = PoseStamped()
      ps.header.frame_id = feedback.header.frame_id
      ps.pose = feedback.pose
      self.current_goal_pose = ps

    if self.opt.robot == 'pr2':
      if feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        for marker in self.wp_im.controls[0].markers:
          marker.color = ColorRGBA(1,1,1,0.4)
        self.server.insert(self.wp_im)
        self.server.applyChanges()

      if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        self._updateIKFeasibility(ps)
        self.server.insert(self.wp_im)
        self.server.applyChanges()

      if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        if self.continuous_update:
          ps = PoseStamped()
          ps.header.frame_id = feedback.header.frame_id
          ps.pose = feedback.pose
          self.current_goal_pose = ps
          #weights_msg = haptic_msgs.HapticMpcWeights()
          #weights_msg.header.stamp = rospy.Time.now()
          #weights_msg.position_weight = self.pos_weight
          #weights_msg.orient_weight = self.orient_weight
          #self.mpc_weights_pub.publish(weights_msg) # Enable position and orientation tracking
          self.goal_pos_pub.publish(self.current_goal_pose)
        self.server.applyChanges()


  ## Publishes the current pose of the interactive marker as the goal pose for the MPC.
  # Also sets the orientation weight for the controller to 0.0 (ie, position only).
  def goalPositionHandler(self, feedback):
    rospy.loginfo("MPC Teleop: Publishing new goal. Position only.")
    weights_msg = haptic_msgs.HapticMpcWeights()
    weights_msg.header.stamp = rospy.Time.now()
    weights_msg.position_weight = self.pos_weight
    weights_msg.orient_weight = 0.0
    weights_msg.posture_weight = 0.0
    self.mpc_weights_pub.publish(weights_msg) # Enable position tracking only - disable orientation by setting the weight to 0
    self.goal_pos_pub.publish(self.current_goal_pose)
#    self.ros_pub.publish('go_to_way_point')

  ## Publishes the current pose of the interactive marker as the goal pose for the MPC.
  # Also enables the orientation weight (ie, position and orientation).
  def goalPositionOrientationHandler(self, feedback):
    rospy.loginfo("MPC Teleop: Publishing new goal. Position and Orientation.")
    weights_msg = haptic_msgs.HapticMpcWeights()
    weights_msg.header.stamp = rospy.Time.now()
    weights_msg.position_weight = self.pos_weight
    weights_msg.orient_weight = self.orient_weight
    self.mpc_weights_pub.publish(weights_msg) # Enable position and orientation tracking
    self.goal_pos_pub.publish(self.current_goal_pose)
 #   self.ros_pub.publish('orient_to_way_point')
    self.add_topic_pub = rospy.Publisher('haptic_mpc/add_taxel_array', String)
    self.remove_topic_pub = rospy.Publisher('haptic_mpc/remove_taxel_array', String)

  ## Publishes the current pose of the interactive marker as the Head pose for the MPC.
  # Also enables the orientation weight (ie, position and orientation).
  def gotoHeadHandler(self, feedback):
    rospy.loginfo("MPC Teleop: Go to Head. Position and Orientation.")
    weights_msg = haptic_msgs.HapticMpcWeights()
    weights_msg.header.stamp = rospy.Time.now()
    weights_msg.position_weight = self.pos_weight
    weights_msg.orient_weight = self.orient_weight
    self.mpc_weights_pub.publish(weights_msg) # Enable position and orientation tracking
    self.head_pos_pub.publish(self.current_goal_pose)
    
  ## Publishes the current pose of the interactive marker as the goal pose for a planner.
  # The planner should then
  def planGoalHandler(self, feedback):
    rospy.loginfo("MPC Teleop: Publishing new goal to the planner. Swapping to postural control")
    weights_msg = haptic_msgs.HapticMpcWeights()
    weights_msg.header.stamp = rospy.Time.now()
    weights_msg.position_weight = 0.0
    weights_msg.orient_weight = 0.0
    weights_msg.posture_weight = self.posture_weight
    self.mpc_weights_pub.publish(weights_msg) # Enable posture tracking
    self.planner_goal_pub.publish(self.current_goal_pose)

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

  def saveInitEEPose(self, ps_msg):
    self.init_ee_pose = ps_msg
    self.init_ee_pose_received = True
    self.ee_pose_sub.unregister()
    del(self.ee_pose_sub) 

  def saveJointState(self, jtcs_msg):
      self.joint_state = jtcs_msg.actual.positions

  ## Initialise all publishers/subscribers
  def initComms(self, node_name):
    rospy.init_node(node_name)

    #Initial hand configuration Subscriber
    self.ee_pose_sub = rospy.Subscriber('haptic_mpc/gripper_pose', PoseStamped, self.saveInitEEPose)
    # Goal pose publisher.
    self.goal_pos_pub = rospy.Publisher("haptic_mpc/goal_pose", PoseStamped, latch=True)
    self.mpc_weights_pub = rospy.Publisher("haptic_mpc/weights", haptic_msgs.HapticMpcWeights)
    self.goal_traj_pub = rospy.Publisher("haptic_mpc/goal_pose_array", PoseArray)
    self.planner_goal_pub = rospy.Publisher("haptic_mpc/planner_goal_pose", PoseStamped, latch=True)

    self.add_topic_pub = rospy.Publisher("haptic_mpc/add_taxel_array", String)
    self.remove_topic_pub = rospy.Publisher("haptic_mpc/remove_taxel_array", String)

    self.joint_state_sub = rospy.Subscriber("/l_arm_controller/state", JTCS, self.saveJointState)
    self.test_pos_pub = rospy.Publisher("haptic_mpc/test_pose", PoseStamped, latch=True)
    self.head_pos_pub = rospy.Publisher("haptic_mpc/head_pose", Posestamped, latch=True)
    if self.opt.robot =='pr2':
        self.pr2_ik_client = rospy.ServiceProxy("/pr2_left_arm_kinematics/get_ik", GetPositionIK, persistent=True)
        pr2_ik_info_client = rospy.ServiceProxy("/pr2_left_arm_kinematics/get_ik_solver_info",
                                                  GetKinematicSolverInfo)

        rospy.loginfo("[mpc_teleop_rviz]: Waiting for PR2_IK_services")
        try:
            rospy.wait_for_service("/pr2_left_arm_kinematics/get_ik", timeout=10.)
            rospy.wait_for_service("/pr2_left_arm_kinematics/get_ik_solver_info", timeout=10.)
        except ROSException as re:
            rospy.logwarn("[mpc_teleop_rviz]: Could not find PR2 IK Services before timeout (10s)")
        self.ik_solver_info = pr2_ik_info_client.call().kinematic_solver_info

    # These are deprecated and should be cleaned up.
    #self.wp_pose_pub = rospy.Publisher('/teleop_rviz/command/way_point_pose', PoseStamped)
    #self.ros_pub = rospy.Publisher('/epc_skin/command/behavior', String)
    #self.pause_pub = rospy.Publisher('/epc/pause', Bool)
    #self.stop_pub = rospy.Publisher('/epc/stop', Bool)

    self.open_pub = rospy.Publisher('open_gripper', Empty)
    self.close_pub = rospy.Publisher('close_gripper', Empty)

    self.disable_pub = rospy.Publisher('/pr2_fabric_gripper_sensor/disable_sensor', Empty)
    self.enable_pub = rospy.Publisher('/pr2_fabric_gripper_sensor/enable_sensor', Empty)

    # Zero PR2 fabric skin sensors
    self.zero_gripper_pub = rospy.Publisher('/pr2_fabric_gripper_sensor/zero_sensor', Empty)
    self.zero_gripper_left_link_pub = rospy.Publisher('/pr2_fabric_gripper_left_link_sensor/zero_sensor', Empty)
    self.zero_gripper_right_link_pub = rospy.Publisher('/pr2_fabric_gripper_right_link_sensor/zero_sensor', Empty)
    self.zero_gripper_palm_pub = rospy.Publisher('/pr2_fabric_gripper_palm_sensor/zero_sensor', Empty)
    self.zero_forearm_pub = rospy.Publisher('/pr2_fabric_forearm_sensor/zero_sensor', Empty)
    self.zero_upperarm_pub = rospy.Publisher('/pr2_fabric_upperarm_sensor/zero_sensor', Empty)
    self.zero_pps_left_pub = rospy.Publisher('/pr2_pps_left_sensor/zero_sensor', Empty)
    self.zero_pps_right_pub = rospy.Publisher('/pr2_pps_right_sensor/zero_sensor', Empty)
    # Zero Cody skin sensors (fabric + meka)
    self.zero_cody_meka_skin_pub = rospy.Publisher('/skin_patch_forearm_right/zero_sensor', Empty)
    self.zero_cody_fabric_forearm_pub = rospy.Publisher('/fabric_forearm_sensor/zero_sensor', Empty)
    self.zero_cody_fabric_wrist_pub = rospy.Publisher('/fabric_wrist_sensor/zero_sensor', Empty)
    if self.opt.robot == 'pr2':
	import actionlib
        self.gripper_action_client = actionlib.SimpleActionClient(self.arm+'_gripper_controller/gripper_action', Pr2GripperCommandAction)

    self.server = ims.InteractiveMarkerServer('teleop_rviz_server')

  ## Initialise the interactive marker based on what robot we're running on, and whether we use orientation or just position.
  def initMarkers(self):
    while not self.init_ee_pose_received and not rospy.is_shutdown():
      rospy.sleep(1)
      rospy.loginfo("[mpc_teleops_rviz.py] Waiting for initial end effector pose")
    ps = self.init_ee_pose

    #--- interactive marker for way point ---
    if self.opt.robot == "cody":
      if self.opt.orientation:
        self.wp_im = imu.make_6dof_gripper(False, ps, 0.28, (1., 1., 0.,0.4), "cody")
        #wp_im = imu.make_6dof_marker(False, ps, 0.15, (1., 1., 0.,0.4), 'sphere')
      else:
        self.wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif self.opt.robot == "pr2":
      if self.opt.orientation:
        #wp_im = imu.make_6dof_marker(False, ps, 0.15, (1., 1., 0.,0.4), 'sphere')
        self.wp_im = imu.make_6dof_gripper(False, ps, 0.28, (1., 1., 0.,0.4))
      else:
        self.wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif self.opt.robot == "darci" or self.opt.robot == "darci_sim":
      if self.opt.orientation:
        self.wp_im = imu.make_6dof_gripper(False, ps, 0.28, (1., 1., 0.,0.4), "darci")
      else:
        self.wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif self.opt.robot == "sim3":
      self.wp_im = imu.make_marker_position_xy(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif self.opt.robot == "simcody":
      if opt.orientation:
        self.wp_im = imu.make_6dof_gripper(False, ps, 0.28, (1., 1., 0.,0.4), "cody")
        #wp_im = imu.make_6dof_marker(False, ps, 0.15, (1., 1., 0.,0.4), 'sphere')
      else:
        self.wp_im = imu.make_3dof_marker_position(ps, 0.15, (1., 1., 0.,0.4), 'sphere')
    elif self.opt.robot == "crona":
      #ps.header.frame_id = "/torso_chest_link"
#      ps.header.frame_id = "/base_link" # testing
      self.wp_im = imu.make_6dof_marker(False, ps, 0.5, (1., 1., 0.,0.4), 'sphere')
    else:
      rospy.logerr('Please specify a robot type using the input arguments: -r <pr2, sim3, etc>')
      sys.exit()

    ps = PoseStamped()
    ps.header = self.wp_im.header
    ps.pose = self.wp_im.pose
    self.current_goal_pose = ps

    self.wp_im.name = 'way_point'
    self.wp_im.description = 'Waypoint'
    self.server.insert(self.wp_im, self.interactiveMarkerLocationCallback)
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
    self.wp_menu_handler = mh.MenuHandler()
    self.wp_menu_handler.insert('Go', callback = self.goalPositionHandler)
    if self.opt.robot != "sim3": # Sim can't orient in 6DOF as it's a 3DOF planar arm.
      self.wp_menu_handler.insert('Orient', callback = self.goalPositionOrientationHandler)
   # self.wp_menu_handler.insert('Plan to goal', callback = self.planGoalHandler)
    if self.opt.robot != "sim3":  # Stop doesn't work for sim - don't use it.
      self.wp_menu_handler.insert('Stop', callback = self.stopArmHandler)
    if self.opt.robot == "pr2": # Gripper commands are specific to the PR2
      self.wp_menu_handler.insert('Open Gripper', callback = self.openGripperHandler)
      self.wp_menu_handler.insert('Close Gripper', callback = self.closeGripperHandler)
      self.wp_menu_handler.insert('Continuous', callback = self.continuousUpdateHandler)
      self.wp_menu_handler.insert('Go_to_Head', callback = self.gotoHeadHandler)
    if self.opt.robot != "sim3": # Sim doesn't need zeroing ever, doesn't need to be present.
      self.wp_menu_handler.insert('Zero Skin', callback = self.zeroSkinHandler)

    imu.add_menu_handler(self.wp_im, self.wp_menu_handler, self.server) 

  ## Start the interactive marker server (spin indefinitely).
  def start(self):
    mpc_ims.initComms("mpc_teleop_rviz")
    mpc_ims.initMarkers()
    mpc_ims.initMenu()
    rospy.loginfo('Haptic MPC interactive marker server started')
    while not rospy.is_shutdown():
      rospy.spin()

if __name__ == '__main__':
  # Parse an options list specifying robot type
  import optparse
  p = optparse.OptionParser()
  haptic_mpc_util.initialiseOptParser(p)
  opt = haptic_mpc_util.getValidInput(p)

  # Initialise publishers/subscribers
  mpc_ims = MPCTeleopInteractiveMarkers(opt)
  mpc_ims.start()
