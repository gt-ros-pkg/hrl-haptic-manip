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

import roslib
roslib.load_manifest("LMPC")
import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.transforms as tr

#import epc_skin_math as esm # Core maths functions used by the MPC
import crona2_twoarm_mpc_math as esm # Core maths functions used by the MPC

import LMPC_msgs.msg as haptic_msgs
from sttr_msgs.msg import GoalPose, DebugMomentBalancing
#import hrl_haptic_manipulation_in_clutter_srvs.srv as haptic_srvs
import geometry_msgs.msg
import std_msgs.msg
import hrl_msgs.msg

import multiarray_to_matrix # Helper class with functions to convert a Float64MultiArray to a list of numpy matrices
import haptic_mpc_util # Utility script with helper functions used across all haptic mpc nodes.

import numpy as np
import threading
import copy
import itertools as it
import sys
import math
import signal

## Container class for the controller parameters.
#
# Some of these are parameters rather than control data, but they're small cf. the data which changes at each timestep.
class MPCData():
  def __init__(self, larm_q, lhand_x, lhand_x_g, lforearm_x, lforearm_x_g, rarm_q, rhand_x, rhand_x_g, rforearm_x, rforearm_x_g, larm_force, rarm_force, larm_jt, rarm_jt, torso_jt, torso_x, object_x, object_x_g, object_q_orient, object_q_g_orient, object_accel, object_dist_force, dist_g, ang_dist_g, goal_posture,
           lhand_q_orient, lhand_q_g_orient, lforearm_q_orient, lforearm_q_g_orient, rhand_q_orient, rhand_q_g_orient, rforearm_q_orient, rforearm_q_g_orient, 
	   larm_friction_p1_pose, larm_friction_p2_pose, larm_friction_p3_pose, rarm_friction_p1_pose, rarm_friction_p2_pose, rarm_friction_p3_pose,
           object_pos_weight, object_orient_weight, hand_pos_weight, hand_orient_weight, forearm_pos_weight, forearm_orient_weight, posture_weight, force_weight,
           force_reduction_goal,
           control_point_joint_num,
           Kc_l, Rc_l, Jc_l, lhand_J, lforearm_J, rhand_J, rforearm_J, larm_object_J, rarm_object_J, 
	   larm_friction_J1, larm_friction_J2, larm_friction_J3, rarm_friction_J1, rarm_friction_J2, rarm_friction_J3,
	   ragdoll_m1, ragdoll_m2, ragdoll_m3, ragdoll_m1_x, ragdoll_m1_q_orient, ragdoll_m2_x, ragdoll_m2_q_orient, ragdoll_m3_x, ragdoll_m3_q_orient, ragdoll_m2_J, ragdoll_m3_J,
           delta_f_min, delta_f_max,
           larm_phi_curr, larm_K_j, rarm_phi_curr, rarm_K_j,
           loc_l, n_l, f_l, f_n,
           jerk_opt_weight, max_force_mag,
           larm_jep, rarm_jep, time_step, stop, miu, object_height, object_width, object_depth, object_mass,
	   object_lower_body_pos, object_l_thigh_pos, object_r_thigh_pos, object_upper_body_pos):

    self.larm_q = larm_q           
    self.lhand_x = lhand_x        
    self.lhand_x_g = lhand_x_g       
    self.lforearm_x = lforearm_x
    self.lforearm_x_g = lforearm_x_g
    self.rarm_q = rarm_q
    self.rhand_x = rhand_x
    self.rhand_x_g = rhand_x_g
    self.rforearm_x = rforearm_x
    self.rforearm_x_g = rforearm_x_g
    self.larm_force = larm_force
    self.rarm_force = rarm_force
    self.larm_jt = larm_jt
    self.rarm_jt = rarm_jt
    self.torso_jt = torso_jt
    self.torso_x = torso_x
    self.object_x = object_x
    self.object_x_g = object_x_g
    self.object_q_orient = object_q_orient
    self.object_q_g_orient = object_q_g_orient
    self.object_accel = object_accel
    self.object_dist_force = object_dist_force
    self.dist_g = dist_g  # dist to goal
    self.ang_dist_g = ang_dist_g #angular distance to goal
    self.goal_posture = goal_posture
    self.lhand_q_orient = lhand_q_orient 
    self.lhand_q_g_orient = lhand_q_g_orient
    self.lforearm_q_orient = lforearm_q_orient
    self.lforearm_q_g_orient = lforearm_q_g_orient
    self.rhand_q_orient = rhand_q_orient
    self.rhand_q_g_orient = rhand_q_g_orient
    self.rforearm_q_orient = rforearm_q_orient
    self.rforearm_q_g_orient = rforearm_q_g_orient
    self.larm_friction_p1_pose = larm_friction_p1_pose
    self.larm_friction_p2_pose = larm_friction_p2_pose
    self.larm_friction_p3_pose = larm_friction_p3_pose
    self.rarm_friction_p1_pose = rarm_friction_p1_pose
    self.rarm_friction_p2_pose = rarm_friction_p2_pose
    self.rarm_friction_p3_pose = rarm_friction_p3_pose
    self.object_pos_weight = object_pos_weight
    self.object_orient_weight = object_orient_weight
    self.hand_pos_weight = hand_pos_weight
    self.hand_orient_weight = hand_orient_weight
    self.forearm_pos_weight = forearm_pos_weight
    self.forearm_orient_weight = forearm_orient_weight
    self.posture_weight = posture_weight
    self.force_weight = force_weight
    self.force_reduction_goal = force_reduction_goal
    self.control_point_joint_num = control_point_joint_num
    self.Kc_l = Kc_l
    self.Rc_l = Rc_l
    self.Jc_l = Jc_l
    self.lhand_J = lhand_J
    self.lforearm_J = lforearm_J
    self.rhand_J = rhand_J
    self.rforearm_J = rforearm_J
    self.larm_object_J = larm_object_J
    self.rarm_object_J = rarm_object_J
    self.larm_friction_J1 = larm_friction_J1
    self.larm_friction_J2 = larm_friction_J2
    self.larm_friction_J3 = larm_friction_J3
    self.rarm_friction_J1 = rarm_friction_J1
    self.rarm_friction_J2 = rarm_friction_J2
    self.rarm_friction_J3 = rarm_friction_J3
    self.ragdoll_m1 = ragdoll_m1
    self.ragdoll_m2 = ragdoll_m2
    self.ragdoll_m3 = ragdoll_m3
    self.ragdoll_m1_x = ragdoll_m1_x
    self.ragdoll_m1_q_orient = ragdoll_m1_q_orient
    self.ragdoll_m2_x = ragdoll_m2_x
    self.ragdoll_m2_q_orient = ragdoll_m2_q_orient
    self.ragdoll_m3_x = ragdoll_m3_x
    self.ragdoll_m3_q_orient = ragdoll_m3_q_orient
    self.ragdoll_m2_J = ragdoll_m2_J
    self.ragdoll_m3_J = ragdoll_m3_J
    self.delta_f_min = delta_f_min
    self.delta_f_max = delta_f_max
    self.larm_phi_curr = larm_phi_curr
    self.larm_K_j = larm_K_j
    self.rarm_phi_curr = rarm_phi_curr
    self.rarm_K_j = rarm_K_j
    self.loc_l = loc_l
    self.n_l = n_l
    self.f_l = f_l
    self.f_n = f_n
    self.jerk_opt_weight = jerk_opt_weight
    self.max_force_mag = max_force_mag
    self.larm_jep = larm_jep
    self.rarm_jep = rarm_jep
    self.time_step = time_step
    self.stop = stop
    self.miu = miu
    self.object_height = object_height
    self.object_width = object_width
    self.object_depth = object_depth
    self.object_mass = object_mass
    self.object_lower_body_pos = object_lower_body_pos
    self.object_l_thigh_pos = object_l_thigh_pos
    self.object_r_thigh_pos = object_r_thigh_pos
    self.object_upper_body_pos = object_upper_body_pos

  ## String representation of the data structure. Useful for debugging.
  def __str__(self):
    string = ""
    string += "MPC Data Structure:"
    string += "\nq: \t\t%s" % str(self.q)
    string += "\nx_h: \t\t%s" % str(self.x_h)
    string += "\nx_g: \t\t%s" % str(self.x_g)
    string += "\ndist_g: \t\t%s" % str(self.dist_g)  # dist to goal
    string += "\nang_dist_g: \t\t%s" % str(self.ang_dist_g)  # dist to goal
    string += "\nq_h_orient: \t\t%s" % str(self.q_h_orient) # end effector orientation
    string += "\nq_g_orient: \t\t%s" % str(self.q_g_orient)
    string += "\nposition_weight: \t\t%s" % str(self.position_weight)
    string += "\norient_weight: \t\t%s" % str(self.orient_weight)
    string += "\nforearm_orient_weight: \t\t%s" % str(self.forearm_orient_weight)
    string += "\ncontrol_point_joint_num: \t\t%s" % str(self.control_point_joint_num)
    string += "\nKc_l: \t\t%s" % str(self.Kc_l)
    string += "\nRc_l: \t\t%s" % str(self.Rc_l)
    string += "\nJc_l: \t\t%s" % str(self.Jc_l)
    string += "\nJe: \t\t%s" % str(self.larm_Je)
    string += "\nJe: \t\t%s" % str(self.larm_J_forearm)
    string += "\nJe: \t\t%s" % str(self.rarm_Je)
    string += "\nJe: \t\t%s" % str(self.rarm_J_forearm)
    string += "\nJe: \t\t%s" % str(self.object_J)
    string += "\ndelta_f_min: \t\t%s" % str(self.delta_f_min)
    string += "\ndelta_f_max: \t\t%s" % str(self.delta_f_max)
    string += "\nphi_curr: \t\t%s" % str(self.phi_curr)
    string += "\nK_j: \t\t%s" % str(self.K_j)
    string += "\nloc_l: \t\t%s" % str(self.loc_l)
    string += "\nn_l: \t\t%s" % str(self.n_l)
    string += "\nf_l: \t\t%s" % str(self.f_l)
    string += "\nf_n: \t\t%s" % str(self.f_n)
    string += "\njerk_opt_weight: \t\t%s" % str(self.jerk_opt_weight)
    string += "\nmax_force_mag: \t\t%s" % str(self.max_force_mag)
    string += "\njep: \t\t%s" % str(self.jep)
    string += "\ntime_step: \t\t%s" % str(self.time_step)
    string += "\nstop: \t\t%s" % str(self.stop)
    return string

## Haptic Model Predictive Controller class.
class HapticMPC():
  ## Constructor
  # @param opt optparse options. Should be created using the helper utility script for reliability.
  # @param node_name Name used for the ROS node.
  def __init__(self, opt, node_name="haptic_mpc"):
    rospy.loginfo("Initialising Haptic MPC")
    self.opt = opt

    self.state_lock = threading.RLock() ## Haptic state lock
    self.goal_lock = threading.RLock() ## Goal state lock
    self.monitor_lock = threading.RLock() ## Monitor state lock
    self.gain_lock = threading.RLock() ## Controller gain state lock
    self.posture_lock = threading.RLock() ## Goal posture lock.
    self.object_lock = threading.RLock() ## Goal posture lock.
    self.mpc_data = None ## MPCData data structure.
    self.msg = None ## Haptic state message

    # Haptic State parameters - from a RobotHapticState listener
    self.last_msg_time = None
    self.timeout = 0.50 # If the last time the MPC heard a state message was >50ms ago -> Stop!
    self.waiting_to_resume = False
    self.waiting_for_no_errors = False
    self.mpc_state = None ## Current MPC state. Stored as a list of strings
    self.mpc_error = None ## Current MPC errors. Stored as a list of strings

    self.larm_joint_names = []
    self.larm_joint_angles = []
    self.larm_desired_joint_angles = []
    self.larm_joint_velocities = []
    self.larm_joint_stiffness = []
    self.larm_joint_damping = []

    self.rarm_joint_names = []
    self.rarm_joint_angles = []
    self.rarm_desired_joint_angles = []
    self.rarm_joint_velocities = []
    self.rarm_joint_stiffness = []
    self.rarm_joint_damping = []

    self.larm_end_effector_pos = None
    self.larm_end_effector_orient_quat = None
    self.larm_forearm_pos = None
    self.larm_forearm_orient_quat = None

    self.rarm_end_effector_pos = None
    self.rarm_end_effector_orient_quat = None
    self.rarm_forearm_pos = None
    self.rarm_forearm_orient_quat = None

    self.larm_force = None
    self.rarm_force = None

    self.object_x = None
    self.object_q_orient = None

    self.object_lower_body_pos = None
    self.object_l_thigh_pos = None
    self.object_r_thigh_pos = None
    self.object_upper_body_pos = None

    self.skin_data = []
    self.Jc = []
    self.lhand_J = []
    self.lforearm_J = []
    self.rhand_J = []
    self.rforearm_J = []
    self.larm_object_J = []
    self.rarm_object_J = []

    self.time_step = 0.01 # seconds. NB: This is a default which is set by the "start" function.

    # Trajectory goal position - from a PoseStamped listener
    self.object_x_g = None
    self.object_q_g_orient = None
    self.lhand_x_g = None
    self.lhand_q_g_orient = None
    self.rhand_x_g = None
    self.rhand_q_g_orient = None
    self.lforearm_x_g = None
    self.lforearm_q_g_orient = None
    self.rforearm_x_g = None
    self.rforearm_q_g_orient = None
    self.goal_posture = None

    # Control parameters - read from parameter server
    self.object_pos_weight = 1.0
    self.object_orient_weight = 1.0
    self.hand_pos_weight = 1.0
    self.hand_orient_weight = 1.0
    self.forearm_pos_weight = 1.0
    self.forearm_orient_weight = 1.0
    self.posture_weight = 0.0
    self.orientation_weight_scaling_radius = np.radians(30.0) #degrees
    self.position_weight_scaling_radius = 0.1 # meters

    self.position_step_scaling_radius = 0.25 #meters
    self.orientation_step_scaling_radius = np.radians(30.0) #degrees
    self.posture_step = np.radians(0.5) # radians (?)

    self.deadzone_distance = 0.005 # 5mm
    self.deadzone_angle = np.radians(10.0) # 10 degrees
    self.currently_in_deadzone = False
    self.angle_reset_threshold = np.radians(15.0) #degrees
    
    #************* This value determines max equilibrium angle, change to increase max torque output *****
    self.angle_constraint_threshold = np.radians(1000.0) #degrees

    self.mpc_enabled = True

    self.robot_state_msg_received = False

    # Jacobian MultiArray to Matrix converter
    self.ma_to_m = multiarray_to_matrix.MultiArrayConverter()

  ## Read parameters from the ROS parameter server and store them.
  def initControlParametersFromServer(self):
    base_path = 'haptic_mpc'
    control_path = '/control_params'

    rospy.loginfo("Haptic MPC: Initialising controller parameters from server. Path: %s" % (base_path+control_path))
    # controller parameters
    # Force limits for the controller.
    self.allowable_contact_force = rospy.get_param(base_path + control_path + '/allowable_contact_force') # Max force allowed by the controller
    self.max_delta_force_mag = rospy.get_param(base_path + control_path + '/max_delta_force_mag') # Max change in force allowed.
    self.stopping_force = rospy.get_param(base_path + control_path + '/stopping_force') # Completely shut down if this exceeded

    self.goal_velocity_for_hand = rospy.get_param(base_path + control_path + '/goal_velocity_for_hand')
    self.goal_ang_velocity_for_hand = np.radians(rospy.get_param(base_path + control_path + '/goal_ang_velocity_for_hand'))
    self.deadzone_distance = rospy.get_param(base_path + control_path + '/deadzone_distance')
    self.deadzone_angle = np.radians(rospy.get_param(base_path + control_path + '/deadzone_angle'))
    self.deadzone_posture = rospy.get_param(base_path + control_path + '/deadzone_posture')
    self.deadzone_alpha = rospy.get_param(base_path + control_path + '/deadzone_alpha')
    self.angle_reset_threshold = np.radians(rospy.get_param(base_path + control_path + '/angle_reset_threshold'))
    self.angle_constraint_threshold = np.radians(rospy.get_param(base_path + control_path + '/angle_constraint_threshold'))

    # stiffness parameters
    self.static_contact_stiffness_estimate = rospy.get_param(base_path + control_path + '/static_contact_stiffness_estimate')
    self.estimate_contact_stiffness = rospy.get_param(base_path + control_path + '/estimate_contact_stiffness')

    self.object_pos_weight = rospy.get_param(base_path + control_path + '/object_position_weight')
    self.object_orient_weight = rospy.get_param(base_path + control_path + '/object_orientation_weight')
    self.hand_pos_weight = rospy.get_param(base_path + control_path + '/hand_position_weight')
    self.hand_orient_weight = rospy.get_param(base_path + control_path + '/hand_orientation_weight')
    self.forearm_pos_weight = rospy.get_param(base_path + control_path + '/forearm_position_weight')
    self.forearm_orient_weight = rospy.get_param(base_path + control_path + '/forearm_orientation_weight')
    self.force_weight = rospy.get_param(base_path + control_path + '/force_reduction_weight')
    self.jerk_opt_weight = rospy.get_param(base_path + control_path + '/jerk_opt_weight')
    self.position_weight_scaling_radius = rospy.get_param(base_path + control_path + '/position_weight_scaling_radius')
    self.orientation_weight_scaling_radius = np.radians(rospy.get_param(base_path + control_path + '/orientation_weight_scaling_radius'))

    self.position_step_scaling_radius = rospy.get_param(base_path + control_path + '/position_step_scaling_radius')
    self.orientation_step_scaling_radius = np.radians(rospy.get_param(base_path + control_path + '/orientation_step_scaling_radius'))

    self.max_theta_step = rospy.get_param(base_path + control_path + '/posture_step_size')
    self.theta_step_scale = rospy.get_param(base_path + control_path + '/posture_step_scale')

    self.force_reduction_goal = rospy.get_param(base_path + control_path + '/force_reduction_goal')

    self.frequency = rospy.get_param(base_path + control_path + '/frequency')

    self.miu = rospy.get_param(base_path + control_path + '/miu')
    self.object_height = rospy.get_param(base_path + control_path + '/height')
    self.object_width = rospy.get_param(base_path + control_path + '/width')
    self.object_depth = rospy.get_param(base_path + control_path + '/depth')
    self.object_mass = rospy.get_param(base_path + control_path + '/mass')

  ## Initialise the robot joint limits.
  # This relies on something (usually the haptic state node) pushing the appropriate joint limits to the param server before execution.
  # NB: All joint limits are specified in degrees and converted to radians here (easier to understand)
  def initRobot(self):
    base_path = 'haptic_mpc'

    rospy.loginfo("Haptic MPC: Waiting for joint limit parameters to be set")
    while rospy.has_param(base_path + '/larm_joint_limits/max') == False or rospy.has_param(base_path + '/larm_joint_limits/min') == False:
      rospy.sleep(2.0)
    rospy.loginfo("Haptic MPC: Got larm joint limits, continuing")
    self.larm_joint_limits_max = np.radians(rospy.get_param(base_path + '/larm_joint_limits/max'))
    self.larm_joint_limits_min = np.radians(rospy.get_param(base_path + '/larm_joint_limits/min'))
  
    while rospy.has_param(base_path + '/rarm_joint_limits/max') == False or rospy.has_param(base_path + '/rarm_joint_limits/min') == False:
      rospy.sleep(2.0)
    rospy.loginfo("Haptic MPC: Got rarm joint limits, continuing")
    self.rarm_joint_limits_max = np.radians(rospy.get_param(base_path + '/rarm_joint_limits/max'))
    self.rarm_joint_limits_min = np.radians(rospy.get_param(base_path + '/rarm_joint_limits/min'))

  ## getSkinData accessor function.
  # @return A copy of the skin_data dictionary, containing the latest taxel data.
  def getSkinData(self):
    with self.state_lock:
      skin_data = copy.copy(self.skin_data)
    return skin_data

  ## getJointAngles accessor function.
  # @return A copy of the buffered joint angles list.
  def getJointAngles(self):
    with self.state_lock:
      larm_joint_angles = copy.copy(self.larm_joint_angles)
    return larm_joint_angles

  ## getDesiredJointAngles accessor function.
  # @return A copy of the desired joint angles list, ie, the current arm controller setpoint.
  def getDesiredJointAngles(self):
    with self.state_lock:
      larm_desired_joint_angles = copy.copy(self.larm_desired_joint_angles)
    return larm_desired_joint_angles

  ## getJointStiffness accessor function.
  # @return A copy of the joint stiffness parameters used by the controller.
  def getJointStiffness(self):
    with self.state_lock:
      larm_joint_stiffness = copy.copy(self.larm_joint_stiffness)
    return larm_joint_stiffness

  ## getMPCData accessor function.
  # Returns a copy of the control data structure.
  def getMPCData(self):
    with lock:
      return copy.copy(self.mpc_data)

  ## Builds the MPC data structure from the individual parameters
  # This is just a convenience data structure to amalgamate the data.
  # Also does data validation.
  def initMPCData(self):
    # Copy the latest data received from the robot haptic state.
    # To ensure the data is synchronised, all necessary data is copied with the lock rather than using the accessor functions.
    with self.state_lock:
      larm_q = copy.copy(self.larm_joint_angles)
      larm_q_des = copy.copy(self.larm_desired_joint_angles)
      rarm_q = copy.copy(self.rarm_joint_angles)
      rarm_q_des = copy.copy(self.rarm_desired_joint_angles)
      larm_jt = copy.copy(self.larm_joint_torques)
      rarm_jt = copy.copy(self.rarm_joint_torques)
      torso_jt = copy.copy(self.torso_joint_torque)
      Jc = copy.copy(self.Jc)
      skin_data = copy.copy(self.skin_data)
      lhand_x = copy.copy(self.lhand_x)
      lhand_q_orient = copy.copy(self.lhand_q_orient)
      lforearm_x = copy.copy(self.lforearm_x)
      lforearm_q_orient = copy.copy(self.lforearm_q_orient)
      rhand_x = copy.copy(self.rhand_x)
      rhand_q_orient = copy.copy(self.rhand_q_orient)
      rforearm_x = copy.copy(self.rforearm_x)
      rforearm_q_orient = copy.copy(self.rforearm_q_orient)
      larm_force = copy.copy(self.larm_force)
      rarm_force = copy.copy(self.rarm_force)
      torso_x = copy.copy(self.torso_x)
      object_x = copy.copy(self.object_x)
      object_q_orient = copy.copy(self.object_q_orient)
      object_accel = copy.copy(self.object_accel)
      object_dist_force = copy.copy(self.object_dist_force)
      ragdoll_m1 = copy.copy(self.ragdoll_m1)
      ragdoll_m2 = copy.copy(self.ragdoll_m2)
      ragdoll_m3 = copy.copy(self.ragdoll_m3)
      ragdoll_m1_x = copy.copy(self.ragdoll_m1_x)
      ragdoll_m1_q_orient = copy.copy(self.ragdoll_m1_q_orient)
      ragdoll_m2_x = copy.copy(self.ragdoll_m2_x)
      ragdoll_m2_q_orient = copy.copy(self.ragdoll_m2_q_orient)
      ragdoll_m3_x = copy.copy(self.ragdoll_m3_x)
      ragdoll_m3_q_orient = copy.copy(self.ragdoll_m3_q_orient)
      larm_friction_p1_pose = copy.copy(self.larm_friction_p1_pose)
      larm_friction_p2_pose = copy.copy(self.larm_friction_p2_pose)
      larm_friction_p3_pose = copy.copy(self.larm_friction_p3_pose)
      rarm_friction_p1_pose = copy.copy(self.rarm_friction_p1_pose)
      rarm_friction_p2_pose = copy.copy(self.rarm_friction_p2_pose)
      rarm_friction_p3_pose = copy.copy(self.rarm_friction_p3_pose)
      larm_joint_stiffness = copy.copy(self.larm_joint_stiffness)
      rarm_joint_stiffness = copy.copy(self.rarm_joint_stiffness)
      lhand_J = copy.copy(self.lhand_J)
      lforearm_J = copy.copy(self.lforearm_J)
      rhand_J = copy.copy(self.rhand_J)
      rforearm_J = copy.copy(self.rforearm_J)
      larm_object_J = copy.copy(self.larm_object_J)
      rarm_object_J = copy.copy(self.rarm_object_J)
      larm_friction_J1 = copy.copy(self.larm_friction_J1)
      larm_friction_J2 = copy.copy(self.larm_friction_J2)
      larm_friction_J3 = copy.copy(self.larm_friction_J3)
      rarm_friction_J1 = copy.copy(self.rarm_friction_J1)
      rarm_friction_J2 = copy.copy(self.rarm_friction_J2)
      rarm_friction_J3 = copy.copy(self.rarm_friction_J3)
      ragdoll_m2_J = copy.copy(self.ragdoll_m2_J)
      ragdoll_m3_J = copy.copy(self.ragdoll_m3_J)
      object_lower_body_pos = copy.copy(self.object_lower_body_pos)
      object_l_thigh_pos = copy.copy(self.object_l_thigh_pos)
      object_r_thigh_pos = copy.copy(self.object_r_thigh_pos)
      object_upper_body_pos = copy.copy(self.object_upper_body_pos)

    # Copy the goal parameters.
    with self.goal_lock:
      object_x_g = copy.copy(self.object_x_g)
      object_q_g_orient = copy.copy(self.object_q_g_orient)
      lhand_x_g = copy.copy(self.lhand_x_g)
      lhand_q_g_orient = copy.copy(self.lhand_q_g_orient)
      rhand_x_g = copy.copy(self.rhand_x_g)
      rhand_q_g_orient = copy.copy(self.rhand_q_g_orient)
      lforearm_x_g = copy.copy(self.lforearm_x_g)
      lforearm_q_g_orient = copy.copy(self.lforearm_q_g_orient)
      rforearm_x_g = copy.copy(self.rforearm_x_g)
      rforearm_q_g_orient = copy.copy(self.rforearm_q_g_orient)

    with self.posture_lock:
      goal_posture = copy.copy(self.goal_posture)

    n_l = haptic_mpc_util.getNormals(skin_data)
    f_l = haptic_mpc_util.getValues(skin_data)
    loc_l = haptic_mpc_util.getLocations(skin_data)

    # Control params
    k_default = self.static_contact_stiffness_estimate
    dist_g = self.time_step * self.goal_velocity_for_hand
    ang_dist_g = self.time_step * self.goal_ang_velocity_for_hand

    # Compute various matrices.
    Kc_l = self.contactStiffnessMatrix(n_l, k_default) # Using default k_est_min and k_est_max
    Rc_l = self.contactForceTransformationMatrix(n_l)

    # calculate the normal components of the current contact
    # forces
    tmp_l = [R_ci * f_ci for R_ci, f_ci in it.izip(Rc_l, f_l)]
    f_n = np.matrix([tmp_i[0,0] for tmp_i in tmp_l]).T

    delta_f_min, delta_f_max = self.deltaForceBounds(f_n,
                                                       max_pushing_force = self.allowable_contact_force,
                                                       max_pulling_force = self.allowable_contact_force,
                                                       max_pushing_force_increase = self.max_delta_force_mag,
                                                       max_pushing_force_decrease = self.max_delta_force_mag,
                                                       min_decrease_when_over_max_force = 0.,
                                                       max_decrease_when_over_max_force = 10.0)

    larm_q_des_matrix = (np.matrix(larm_q_des)[0:len(larm_q_des)]).T

    larm_K_j = np.diag(larm_joint_stiffness)

    rarm_q_des_matrix = (np.matrix(rarm_q_des)[0:len(rarm_q_des)]).T

    rarm_K_j = np.diag(rarm_joint_stiffness)

    '''
    if len(lhand_J) >= 1: # Je could potentially be a list of end effector jacobians. We only care about the first one in this implementation.
      lhand_J = lhand_J[0]
    if len(rhand_J) >= 1: # Je could potentially be a list of end effector jacobians. We only care about the first one in this implementation.
      rarm_Je = rarm_Je[0]
    '''

    # Calculate MPC orientation/position weight. This is a bad hack to essentially damp the response as it approaches the goal.
    with self.gain_lock:
	object_pos_weight = self.object_pos_weight
	object_orient_weight = self.object_orient_weight
	hand_pos_weight = self.hand_pos_weight
        hand_orient_weight = self.hand_orient_weight
	forearm_pos_weight = self.forearm_pos_weight
        forearm_orient_weight = self.forearm_orient_weight
        posture_weight = self.posture_weight
    planar = False

    with self.object_lock:
	miu = self.miu
   	object_height = self.object_height
   	object_width = self.object_width
   	object_depth = self.object_depth
	object_mass = self.object_mass

    # Evalute current goals to give the controller some meaningful input
    # If the goal is non-existant, use the current position/orientation/posture as the goal.
    if object_x_g == None:
      object_x_g = object_x
    if object_q_g_orient == None:
      object_q_g_orient = object_q_orient
    if lhand_x_g == None:
      lhand_x_g = lhand_x
    if lhand_q_g_orient == None:
      lhand_q_g_orient = lhand_q_orient
    if rhand_x_g == None:
      rhand_x_g = rhand_x
    if rhand_q_g_orient == None:
      rhand_q_g_orient = rhand_q_orient
    if lforearm_x_g == None:
      lforearm_x_g = lforearm_x
    if lforearm_q_g_orient == None:
      lforearm_q_g_orient = lforearm_q_orient
    if rforearm_x_g == None:
      rforearm_x_g = rforearm_x
    if rforearm_q_g_orient == None:
      rforearm_q_g_orient = rforearm_q_orient

    # Posture
    if goal_posture == None:
      goal_posture = larm_q

    jerk_opt_weight = self.jerk_opt_weight

    
    object_angle_error = ut.quat_angle(object_q_orient, object_q_g_orient)
    object_dist_goal = np.linalg.norm(object_x_g - object_x)
    pos_weight_scale = 1.
    orient_weight_scale = 1.
    if object_dist_goal < self.position_weight_scaling_radius:
      pos_weight_scale = object_dist_goal / self.position_weight_scaling_radius
      object_pos_weight *= pos_weight_scale
    if object_angle_error < self.orientation_weight_scaling_radius:
      orient_weight_scale = object_angle_error / self.orientation_weight_scaling_radius
      object_orient_weight *= orient_weight_scale

    lforearm_angle_error = ut.quat_angle(lforearm_q_orient, lforearm_q_g_orient)
    lforearm_dist_goal = np.linalg.norm(lforearm_x_g - lforearm_x)
    rforearm_angle_error = ut.quat_angle(rforearm_q_orient, rforearm_q_g_orient)
    rforearm_dist_goal = np.linalg.norm(rforearm_x_g - rforearm_x)
    forearm_angle_error = (lforearm_angle_error+rforearm_angle_error)/2.
    forearm_dist_goal = (lforearm_dist_goal+rforearm_dist_goal)/2.
    pos_weight_scale = 1.
    orient_weight_scale = 1.
    if forearm_dist_goal < self.position_weight_scaling_radius:
      pos_weight_scale = forearm_dist_goal / self.position_weight_scaling_radius
      forearm_pos_weight *= pos_weight_scale
    if forearm_angle_error < self.orientation_weight_scaling_radius:
      orient_weight_scale = forearm_angle_error / self.orientation_weight_scaling_radius
      forearm_orient_weight *= orient_weight_scale
    

    # Initialise and return the mpc data structure.
    mpc_data = MPCData( larm_q = larm_q,
                        lhand_x = lhand_x, # numpy array
                        lhand_x_g = lhand_x_g, # numpy array
			lforearm_x = lforearm_x,
			lforearm_x_g = lforearm_x_g,
			rarm_q = rarm_q,
			rhand_x = rhand_x, 
			rhand_x_g = rhand_x_g, 
			rforearm_x = rforearm_x,
			rforearm_x_g = rforearm_x_g,
			larm_force = larm_force,
			rarm_force = rarm_force,
			larm_jt = larm_jt,
			rarm_jt = rarm_jt,
			torso_jt = torso_jt,
			torso_x = torso_x,
			object_x = object_x,
			object_x_g = object_x_g,
			object_q_orient = object_q_orient,
			object_q_g_orient = object_q_g_orient,
			object_accel = object_accel,
			object_dist_force = object_dist_force,
                        dist_g = dist_g,
                        ang_dist_g = ang_dist_g,
                        goal_posture = goal_posture,
                        lhand_q_orient = lhand_q_orient,
                        lhand_q_g_orient = lhand_q_g_orient,
			lforearm_q_orient = lforearm_q_orient,
                        lforearm_q_g_orient = lforearm_q_g_orient,
			rhand_q_orient = rhand_q_orient,
                        rhand_q_g_orient = rhand_q_g_orient,
			rforearm_q_orient = rforearm_q_orient,
			rforearm_q_g_orient = rforearm_q_g_orient,
			larm_friction_p1_pose = larm_friction_p1_pose,
                        larm_friction_p2_pose = larm_friction_p2_pose,
                        larm_friction_p3_pose = larm_friction_p3_pose,
		        rarm_friction_p1_pose = rarm_friction_p1_pose,
                        rarm_friction_p2_pose = rarm_friction_p2_pose,
                        rarm_friction_p3_pose = rarm_friction_p3_pose,
                        object_pos_weight = object_pos_weight,
                        object_orient_weight = object_orient_weight,
			hand_pos_weight = hand_pos_weight,
                        hand_orient_weight = hand_orient_weight,
			forearm_pos_weight = forearm_pos_weight,
                        forearm_orient_weight = forearm_orient_weight,
                        posture_weight = posture_weight,
                        force_weight = self.force_weight,
                        force_reduction_goal = self.force_reduction_goal, # default desired delta for force reduction.
                        control_point_joint_num = len(larm_q), # number of joints
                        Kc_l = Kc_l,
                        Rc_l = Rc_l,
                        Jc_l = Jc,
                        lhand_J = lhand_J, # NB: Je is a list of end effector jacobians (to use the same utils functions as Jc)
			lforearm_J = lforearm_J,
                        rhand_J = rhand_J, # NB: Je is a list of end effector jacobians (to use the same utils functions as Jc)
                        rforearm_J = rforearm_J,
 			larm_object_J = larm_object_J,
 			rarm_object_J = rarm_object_J,
                        delta_f_min = delta_f_min,
                        delta_f_max = delta_f_max,
                        larm_phi_curr = larm_q_des_matrix,
                        larm_K_j = larm_K_j,
			rarm_phi_curr = rarm_q_des_matrix,
                        rarm_K_j = rarm_K_j,
			larm_friction_J1 = larm_friction_J1,
                        larm_friction_J2 = larm_friction_J2,
                        larm_friction_J3 = larm_friction_J3,
			rarm_friction_J1 = rarm_friction_J1,
                        rarm_friction_J2 = rarm_friction_J2,
                        rarm_friction_J3 = rarm_friction_J3,
			ragdoll_m1 = ragdoll_m1,
			ragdoll_m2 = ragdoll_m2,
			ragdoll_m3 = ragdoll_m3,
		   	ragdoll_m1_x = ragdoll_m1_x,
		   	ragdoll_m1_q_orient = ragdoll_m1_q_orient,
		   	ragdoll_m2_x = ragdoll_m2_x,
		   	ragdoll_m2_q_orient = ragdoll_m2_q_orient,
		   	ragdoll_m3_x = ragdoll_m3_x,
		   	ragdoll_m3_q_orient = ragdoll_m3_q_orient,
			ragdoll_m2_J = ragdoll_m2_J,
			ragdoll_m3_J = ragdoll_m3_J,
                        loc_l = loc_l, # Contact locations from taxels. From skin client.
                        n_l = n_l, # Normals for force locations
                        f_l = f_l, # Cartesian force components for contacts
                        f_n = f_n, # Normal force for contacts
                        jerk_opt_weight = jerk_opt_weight, # From control params
                        max_force_mag = self.allowable_contact_force, # From control params
                        larm_jep = larm_q_des,
                        rarm_jep = rarm_q_des,
                        time_step = self.time_step,
                        stop = False,
			miu = miu,
			object_height = object_height,
			object_width = object_width,
			object_depth = object_depth,	
			object_mass = object_mass,
			object_lower_body_pos = object_lower_body_pos,
			object_l_thigh_pos = object_l_thigh_pos,
			object_r_thigh_pos = object_r_thigh_pos,
			object_upper_body_pos = object_upper_body_pos)

    return mpc_data

  ## Update the position/orientation weights used by the controller.
  # @param msg HapticMpcWeights message object
  def updateWeightsCallback(self, msg):
    with self.gain_lock:
      rospy.loginfo("Updating MPC weights. Pos: %s, Orient: %s, Posture: %s"
                    %(str(msg.position_weight), str(msg.orient_weight), str(msg.posture_weight)))
      self.pos_weight = msg.position_weight
      self.orient_weight = msg.orient_weight
      self.posture_weight = msg.posture_weight
      self.mpc_weights_pub.publish(msg) # Echo the currently used weights. Used as an ACK, basically.

  ## Store the current trajectory goal. The controller will always attempt a linear path to this.
  # @param msg A PoseStamped message
  def goalPoseCallback(self, msg):
    with self.goal_lock:
      if msg.object_pose.position.x != 0 or msg.object_pose.position.y != 0 or msg.object_pose.position.z != 0 or msg.object_pose.orientation.x != 0 or msg.object_pose.orientation.y != 0 or msg.object_pose.orientation.z != 0 or msg.object_pose.orientation.w != 0:
      	self.object_x_g = np.matrix([[msg.object_pose.position.x], [msg.object_pose.position.y], [msg.object_pose.position.z]])
      	self.object_q_g_orient = [msg.object_pose.orientation.x, msg.object_pose.orientation.y, msg.object_pose.orientation.z, msg.object_pose.orientation.w]
      if msg.lforearm_pose.position.x != 0 or msg.lforearm_pose.position.y != 0 or msg.lforearm_pose.position.z != 0 or msg.lforearm_pose.orientation.x != 0 or msg.lforearm_pose.orientation.y != 0 or msg.lforearm_pose.orientation.z != 0 or msg.lforearm_pose.orientation.w != 0:
      	self.lforearm_x_g = np.matrix([[msg.lforearm_pose.position.x], [msg.lforearm_pose.position.y], [msg.lforearm_pose.position.z]])
     	self.lforearm_q_g_orient = [msg.lforearm_pose.orientation.x, msg.lforearm_pose.orientation.y, msg.lforearm_pose.orientation.z, msg.lforearm_pose.orientation.w]
      if msg.rforearm_pose.position.x != 0 or msg.rforearm_pose.position.y != 0 or msg.rforearm_pose.position.z != 0 or msg.rforearm_pose.orientation.x != 0 or msg.rforearm_pose.orientation.y != 0 or msg.rforearm_pose.orientation.z != 0 or msg.rforearm_pose.orientation.w != 0:
      	self.rforearm_x_g = np.matrix([[msg.rforearm_pose.position.x], [msg.rforearm_pose.position.y], [msg.rforearm_pose.position.z]])
     	self.rforearm_q_g_orient = [msg.rforearm_pose.orientation.x, msg.rforearm_pose.orientation.y, msg.rforearm_pose.orientation.z, msg.rforearm_pose.orientation.w]

  ## Store the current posture goal. The controller attempts to achieve this posture.
  # TODO: Define message type. Ideally just joint angles. Float64 list?
  # @param msg An array of Float64s. Type hrl_msgs.FloatArray.
  def goalPostureCallback(self, msg):
    with self.posture_lock:
      self.goal_posture = msg.data # Store it as a python list of floats

  ## Store the state information from the monitor node. Allows the control to be somewhat stateful.
  # The state and error fields are lists of strings indicating some state.
  # @param msg HapticMpcState message object
  def mpcMonitorCallback(self, msg):
    with self.monitor_lock:
      self.mpc_state = msg.state
      self.mpc_error = msg.error

  ## Store the robot haptic state.
  # @param msg RobotHapticState message object
  def robotStateCallback(self, msg):
    self.robot_state_msg_received = True
 
    with self.state_lock:
      self.last_msg_time = rospy.Time.now() # timeout for the controller

      self.msg = msg
      self.larm_joint_names = msg.larm_joint_names
      self.larm_joint_angles = list(msg.larm_joint_angles)
      self.larm_desired_joint_angles = list(msg.larm_desired_joint_angles)
      self.larm_joint_velocities= list(msg.larm_joint_velocities)
      self.larm_joint_stiffness = list(msg.larm_joint_stiffness)
      self.larm_joint_damping = list(msg.larm_joint_damping)
      self.larm_joint_torques = list(msg.larm_joint_torques)

      self.rarm_joint_names = msg.rarm_joint_names
      self.rarm_joint_angles = list(msg.rarm_joint_angles)
      self.rarm_desired_joint_angles = list(msg.rarm_desired_joint_angles)
      self.rarm_joint_velocities= list(msg.rarm_joint_velocities)
      self.rarm_joint_stiffness = list(msg.rarm_joint_stiffness)
      self.rarm_joint_damping = list(msg.rarm_joint_damping)
      self.rarm_joint_torques = list(msg.rarm_joint_torques)

      self.torso_joint_torque = msg.torso_joint_torque
      self.torso_x = np.matrix([[msg.torso_pose.position.x], [msg.torso_pose.position.y], [msg.torso_pose.position.z]])

      self.ragdoll_m1 = msg.ragdoll_m1
      self.ragdoll_m2 = msg.ragdoll_m2
      self.ragdoll_m3 = msg.ragdoll_m3

      self.lhand_x = np.matrix([[msg.l_hand_pose.position.x], [msg.l_hand_pose.position.y], [msg.l_hand_pose.position.z]])
      self.lhand_q_orient = [msg.l_hand_pose.orientation.x, msg.l_hand_pose.orientation.y, msg.l_hand_pose.orientation.z, msg.l_hand_pose.orientation.w]
      self.lforearm_x = np.matrix([[msg.lforearm_pose.position.x], [msg.lforearm_pose.position.y], [msg.lforearm_pose.position.z]])
      self.lforearm_q_orient = [msg.lforearm_pose.orientation.x, msg.lforearm_pose.orientation.y, msg.lforearm_pose.orientation.z, msg.lforearm_pose.orientation.w]

      self.rhand_x = np.matrix([[msg.r_hand_pose.position.x], [msg.r_hand_pose.position.y], [msg.r_hand_pose.position.z]])
      self.rhand_q_orient = [msg.r_hand_pose.orientation.x, msg.r_hand_pose.orientation.y, msg.r_hand_pose.orientation.z, msg.r_hand_pose.orientation.w]
      self.rforearm_x = np.matrix([[msg.rforearm_pose.position.x], [msg.rforearm_pose.position.y], [msg.rforearm_pose.position.z]])
      self.rforearm_q_orient = [msg.rforearm_pose.orientation.x, msg.rforearm_pose.orientation.y, msg.rforearm_pose.orientation.z, msg.rforearm_pose.orientation.w]
  
      self.object_x = np.matrix([[msg.object_pose.position.x], [msg.object_pose.position.y], [msg.object_pose.position.z]])
      self.object_q_orient = [msg.object_pose.orientation.x, msg.object_pose.orientation.y, msg.object_pose.orientation.z, msg.object_pose.orientation.w]
      self.object_lower_body_pos = np.matrix([[msg.object_lower_body_pose.position.x], [msg.object_lower_body_pose.position.y], [msg.object_lower_body_pose.position.z]])
      self.object_l_thigh_pos = np.matrix([[msg.object_l_thigh_pose.position.x], [msg.object_l_thigh_pose.position.y], [msg.object_l_thigh_pose.position.z]])
      self.object_r_thigh_pos = np.matrix([[msg.object_r_thigh_pose.position.x], [msg.object_r_thigh_pose.position.y], [msg.object_r_thigh_pose.position.z]])
      self.object_upper_body_pos = np.matrix([[msg.object_upper_body_pose.position.x], [msg.object_upper_body_pose.position.y], [msg.object_upper_body_pose.position.z]])

      self.ragdoll_m1_x = np.matrix([[msg.ragdoll_m1_pose.position.x], [msg.ragdoll_m1_pose.position.y], [msg.ragdoll_m1_pose.position.z]])
      self.ragdoll_m1_q_orient = [msg.ragdoll_m1_pose.orientation.x,msg.ragdoll_m1_pose.orientation.y,msg.ragdoll_m1_pose.orientation.z,msg.ragdoll_m1_pose.orientation.w]
      self.ragdoll_m2_x = np.matrix([[msg.ragdoll_m2_pose.position.x], [msg.ragdoll_m2_pose.position.y], [msg.ragdoll_m2_pose.position.z]])
      self.ragdoll_m2_q_orient = [msg.ragdoll_m2_pose.orientation.x,msg.ragdoll_m2_pose.orientation.y,msg.ragdoll_m2_pose.orientation.z,msg.ragdoll_m2_pose.orientation.w]
      self.ragdoll_m3_x = np.matrix([[msg.ragdoll_m3_pose.position.x], [msg.ragdoll_m3_pose.position.y], [msg.ragdoll_m3_pose.position.z]])
      self.ragdoll_m3_q_orient = [msg.ragdoll_m3_pose.orientation.x,msg.ragdoll_m3_pose.orientation.y,msg.ragdoll_m3_pose.orientation.z,msg.ragdoll_m3_pose.orientation.w]
 
      #self.object_accel =  np.matrix([[msg.object_accel.x], [msg.object_accel.y], [msg.object_accel.z]])
      self.object_accel =  np.matrix([[0.], [0.], [0.]])
      self.object_dist_force = np.matrix([[msg.object_dist_force.x], [msg.object_dist_force.y], [msg.object_dist_force.z]])

      self.skin_data = msg.skins

      self.larm_force = np.matrix([[msg.larm_object_force.x], [msg.larm_object_force.y], [msg.larm_object_force.z]])
      self.rarm_force = np.matrix([[msg.rarm_object_force.x], [msg.rarm_object_force.y], [msg.rarm_object_force.z]])

      self.larm_friction_p1_pose = np.matrix([[msg.larm_friction_p1_pose.position.x], [msg.larm_friction_p1_pose.position.y], [msg.larm_friction_p1_pose.position.z]])
      self.larm_friction_p2_pose = np.matrix([[msg.larm_friction_p2_pose.position.x], [msg.larm_friction_p2_pose.position.y], [msg.larm_friction_p2_pose.position.z]])
      self.larm_friction_p3_pose = np.matrix([[msg.larm_friction_p3_pose.position.x], [msg.larm_friction_p3_pose.position.y], [msg.larm_friction_p3_pose.position.z]])
      self.rarm_friction_p1_pose = np.matrix([[msg.rarm_friction_p1_pose.position.x], [msg.rarm_friction_p1_pose.position.y], [msg.rarm_friction_p1_pose.position.z]])
      self.rarm_friction_p2_pose = np.matrix([[msg.rarm_friction_p2_pose.position.x], [msg.rarm_friction_p2_pose.position.y], [msg.rarm_friction_p2_pose.position.z]])
      self.rarm_friction_p3_pose = np.matrix([[msg.rarm_friction_p3_pose.position.x], [msg.rarm_friction_p3_pose.position.y], [msg.rarm_friction_p3_pose.position.z]])

      self.lhand_J = self.ma_to_m.multiArrayToMatrixList(msg.larm_end_effector_jacobian)
      self.lforearm_J = self.ma_to_m.multiArrayToMatrixList(msg.lforearm_jacobian)
      self.rhand_J = self.ma_to_m.multiArrayToMatrixList(msg.rarm_end_effector_jacobian)
      self.rforearm_J = self.ma_to_m.multiArrayToMatrixList(msg.rforearm_jacobian)
      self.larm_object_J = self.ma_to_m.multiArrayToMatrixList(msg.larm_object_jacobian)
      self.rarm_object_J = self.ma_to_m.multiArrayToMatrixList(msg.rarm_object_jacobian)
      self.Jc = self.ma_to_m.multiArrayToMatrixList(msg.contact_jacobians)

      self.larm_friction_J1 = self.ma_to_m.multiArrayToMatrixList(msg.larm_friction_jacobian1)
      self.larm_friction_J2 = self.ma_to_m.multiArrayToMatrixList(msg.larm_friction_jacobian2)
      self.larm_friction_J3 = self.ma_to_m.multiArrayToMatrixList(msg.larm_friction_jacobian3)
      self.rarm_friction_J1 = self.ma_to_m.multiArrayToMatrixList(msg.rarm_friction_jacobian1)
      self.rarm_friction_J2 = self.ma_to_m.multiArrayToMatrixList(msg.rarm_friction_jacobian2)
      self.rarm_friction_J3 = self.ma_to_m.multiArrayToMatrixList(msg.rarm_friction_jacobian3)

      self.ragdoll_m2_J = self.ma_to_m.multiArrayToMatrixList(msg.ragdoll_m2_jacobian)
      self.ragdoll_m3_J = self.ma_to_m.multiArrayToMatrixList(msg.ragdoll_m3_jacobian)
  
  ## Interpolate a step towards the given goal orientation.
  # @param q_h_orient The current hand orientation as a quaternion in list form: [x,y,z,w]
  # @param q_g_orient The current goal orientation as a quaternion in list form: [x,y,z,w]
  # @return A desired change in orientation as a delta:
  def goalOrientationInQuat(self, q_h_orient, q_g_orient, ang_dist_g):
    # If the goal position is invalid, return the current end effector position
    #if not q_g_orient:
    #  q_g_orient = q_h_orient
    ang = ut.quat_angle(q_h_orient, q_g_orient)
    ang_mag = abs(ang)
    if ang_mag < 1e-6:
        return np.matrix([0.,0.,0.]).T

    if ang_mag > self.orientation_step_scaling_radius:
      slerp_fraction = ang_dist_g / ang_mag #move the fraction equal to the max ang vel per step
    else:
      slerp_fraction = ang_dist_g / self.orientation_step_scaling_radius
  
    interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, slerp_fraction)
    delta_q_des = tr.tft.quaternion_multiply(interp_q_goal, tr.tft.quaternion_inverse(q_h_orient))
    return np.matrix(delta_q_des[:3]).T

  ## Interpolate a step towards the given goal position.
  # @return A goal delta in position as a numpy matrix.
  # @param x_h The current hand position as a numpy matrix: [x,y,z]
  # @param x_g The current goal position as a numpy matrix: [x,y,z]
  # @param dist_g The max delta position per step (vel x freq)
  def goalMotionForHand(self, x_h, x_g, dist_g):
    # Goal should be None if no goal pose has been heard yet, or a numpy column vector.
    if x_g == None or x_g.size == 0:
      x_g = x_h
    err = (x_g - x_h).A
    err_mag = np.linalg.norm(err)
    if err_mag > self.position_step_scaling_radius:
      delta_x_g = dist_g * (err / err_mag)
    else:
      delta_x_g = dist_g * (err / self.position_step_scaling_radius)
    return delta_x_g

  def goalMotionForForearm(self, x_forearm, q_forearm_orient, ang_dist_g):
    #forearm_goal_orient = [0.707,0.,0.707,0.]
    #self.forearm_goal_orient = [0.,-0.70682518,0.,0.70738827]
    slerp_fraction = 0.01
    interp_q_goal = tr.tft.quaternion_slerp(q_forearm_orient, self.forearm_goal_orient, slerp_fraction)
    delta_forearm_g_orient = tr.tft.quaternion_multiply(interp_q_goal, tr.tft.quaternion_inverse(q_forearm_orient))
    delta_forearm_g = np.matrix([0.,0.,0.]+[delta_forearm_g_orient[0],delta_forearm_g_orient[1],delta_forearm_g_orient[2]]).reshape(6,1)    
    return delta_forearm_g



  def goalMotionForFollower(self, larm_x_h, larm_delta_x, rarm_x_h):
    dist_btwn_arms = np.matrix([[0.], [0.52], [0.]])
    #dist_btwn_arms = np.matrix([[0.], [0.6], [0.]])
    new_larm_x_h = larm_x_h+larm_delta_x[:3]
    delta_arm_pos = new_larm_x_h-rarm_x_h
    delta_pos_follower_g = delta_arm_pos-dist_btwn_arms
    return delta_pos_follower_g*0.5

  ## Process the goal posture input before sending it to the controller.
  # @param goal_posture List of desired joint angles. Should be
  # @param current_posture List of current joint angles
  #
  def goalDeltaPosture(self, goal_posture, current_posture, max_theta_step, num_steps_scale):
    assert max_theta_step >= 0
    assert num_steps_scale >= 0
    # NB: max_theta_step, num_steps_scale must both be POSITIVE.
    delta_theta_des = np.matrix(goal_posture).T - np.matrix(current_posture).T # error term.
   # delta_theta_des = 0.1 * delta_theta_des # scale the absolute error

# ONE SCALING METHOD
    for i in range(len(delta_theta_des)):
      theta = delta_theta_des[i] # theta is the error relative to the desired posture on each joint.
      # num_steps_scale is a parameter which represents how many steps out the delta will be scaled down.
      if abs(theta) > num_steps_scale * max_theta_step:
        theta = max_theta_step * theta/abs(theta) # If we're more than the num steps away, just cap the theta step.
      else:
        theta = theta/num_steps_scale
        # Eg, if we have a step size of 1.0 deg, num_steps is 10.0
        # If we're 8 steps from the goal, we only command 0.8 deg rather than 1.0 (8/10)
      delta_theta_des[i] = theta

    return delta_theta_des


  ## Main control calculation.
  # @param mpc_dat An MPCData object, populated with relevant control parameters and current data
  # @return A list of joint equilibrium point deltas, ie, a desired increment to the current joint position(s).
  def deltaQpJepGen(self, mpc_dat):

    forearm_q_g_orient = [0.,-0.707,0.,0.707]
    # If we have invalid control data, do nothing
    if mpc_dat == None:
      return None

    # If we're within the deadzone band, do nothing
    object_dist_to_goal = np.linalg.norm(mpc_dat.object_x - mpc_dat.object_x_g)
    object_ang_to_goal = ut.quat_angle(mpc_dat.object_q_orient, mpc_dat.object_q_g_orient)
    #print "object_dist_to_goal: ",object_dist_to_goal
    lhand_dist_to_goal = np.linalg.norm(mpc_dat.lhand_x - mpc_dat.lhand_x_g)
    lhand_ang_to_goal = ut.quat_angle(mpc_dat.lhand_q_orient, mpc_dat.lhand_q_g_orient)
    lforearm_dist_to_goal = np.linalg.norm(mpc_dat.lforearm_x - mpc_dat.lforearm_x_g)
    lforearm_ang_to_goal = ut.quat_angle(mpc_dat.lforearm_q_orient, mpc_dat.lforearm_q_g_orient)
    rhand_dist_to_goal = np.linalg.norm(mpc_dat.rhand_x - mpc_dat.rhand_x_g)
    rhand_ang_to_goal = ut.quat_angle(mpc_dat.rhand_q_orient, mpc_dat.rhand_q_g_orient)
    rforearm_dist_to_goal = np.linalg.norm(mpc_dat.rforearm_x - mpc_dat.rforearm_x_g)
    rforearm_ang_to_goal = ut.quat_angle(mpc_dat.rforearm_q_orient, mpc_dat.rforearm_q_g_orient)
    posture_to_goal = np.linalg.norm(np.matrix(mpc_dat.goal_posture).T - np.matrix(mpc_dat.larm_q).T)

    d_g = hrl_msgs.msg.FloatArray()
    d_g.header.stamp = rospy.Time.now()
    d_g.data += [object_dist_to_goal, object_ang_to_goal]
    self.dist_to_goal_pub.publish(d_g)

    #h = std_msgs.msg.Header()
    #h.stamp = rospy.Time.now()
    #self.mpc_weights_pub.publish(h, mpc_dat.position_weight, mpc_dat.orient_weight, mpc_dat.posture_weight)

    #print "posture_to_goal: ",posture_to_goal
    #print "object_dist_to_goal: ",object_dist_to_goal,"(",self.deadzone_distance,")"
    #print "object_ang_to_goal: ",object_ang_to_goal, "(",self.deadzone_angle,")"
    #print "lforearm_dist_to_goal: ",lforearm_dist_to_goal
    #print "lforearm_ang_to_goal: ",lforearm_ang_to_goal
    #print "rforearm_dist_to_goal: ",rforearm_dist_to_goal
    #print "rforearm_ang_to_goal: ",rforearm_ang_to_goal
    dist_g = mpc_dat.dist_g # The computed step based on the controller frequency and desired cartesian velocity.
    ang_dist_g = mpc_dat.ang_dist_g

    # Deadzone on pose motion
    in_deadzone = False
    # If position and orientation, most common scenario
    
    vector1 = np.array(mpc_dat.rarm_friction_p2_pose-mpc_dat.rarm_friction_p1_pose)
    vector2 = np.array(mpc_dat.larm_friction_p3_pose-mpc_dat.rarm_friction_p1_pose)
    vector1 = vector1.reshape(1,3)
    vector2 = vector2.reshape(1,3)
    friction_norm = np.cross(vector1[0],vector2[0])/np.linalg.norm(np.cross(vector1[0],vector2[0]))
    alpha = math.acos(np.dot(np.array([0,0,1]),friction_norm))

    #print "alpha_to_goal: ",alpha,"(",self.deadzone_alpha,")"

    ##### DEADZONE DEFINITION #####
    #if abs(object_dist_to_goal) < self.deadzone_distance: 
#	in_deadzone = True
    #if abs(object_dist_to_goal) < self.deadzone_distance and abs(object_ang_to_goal) < self.deadzone_angle: 
    #	in_deadzone = True
    #if abs(forearm_dist_to_goal) < self.deadzone_distance and abs(forearm_ang_to_goal) < self.deadzone_angle:
    #	in_deadzone = True
    if abs(lforearm_dist_to_goal) < self.deadzone_distance and abs(lforearm_ang_to_goal) < self.deadzone_angle and abs(rforearm_dist_to_goal) < self.deadzone_distance and abs(rforearm_ang_to_goal) < self.deadzone_angle:
    #if abs(lforearm_dist_to_goal) < self.deadzone_distance and abs(rforearm_dist_to_goal) < self.deadzone_distance:
	in_deadzone = True


    # Don't command any pose motion
    self.deadzone_pub.publish(self.currently_in_deadzone)
    if in_deadzone:
      self.larm_J_h = 0.
      self.rarm_J_h = 0.
      dist_g = 0.0
      ang_dist_g = 0.0
      if self.currently_in_deadzone == False:
        rospy.loginfo("MPC entered deadzone: pos %s (%s); orient %s (%s)" % (str(object_dist_to_goal),
                                                                             str(self.deadzone_distance),
                                                                             str(np.degrees(object_ang_to_goal)),
                                                                             str(np.degrees(self.deadzone_angle))
                                                                             ))
      self.currently_in_deadzone = True

      # If we're in the deadzone and have no forces, return zeroes.
      #if len(mpc_dat.loc_l) == 0:
      return [0.0] * 14
	

    else:
      self.currently_in_deadzone = False
    # Generate the position/orientation deltas. These are slewed to set step size.
    delta_object_pos_g  = self.goalMotionForHand(mpc_dat.object_x,
                                          mpc_dat.object_x_g,
                                          dist_g)

    delta_object_orient_g = self.goalOrientationInQuat(mpc_dat.object_q_orient,
                                                mpc_dat.object_q_g_orient,
                                                ang_dist_g)

    delta_lforearm_pos_g = self.goalMotionForHand(mpc_dat.lforearm_x,
                                          mpc_dat.lforearm_x_g,
                                          dist_g)

    delta_lforearm_orient_g = self.goalOrientationInQuat(mpc_dat.lforearm_q_orient,
                                                mpc_dat.lforearm_q_g_orient,
                                                ang_dist_g)

    delta_rforearm_pos_g = self.goalMotionForHand(mpc_dat.rforearm_x,
                                          mpc_dat.rforearm_x_g,
                                          dist_g)

    delta_rforearm_orient_g = self.goalOrientationInQuat(mpc_dat.rforearm_q_orient,
                                                mpc_dat.rforearm_q_g_orient,
                                                ang_dist_g)

    m2_x_g = np.matrix([[0.79],[-0.205],[0.607]])
    m2_orient_g = [0.5,0.5,0.5,0.5]
    m3_x_g = np.matrix([[0.79],[0.424],[0.607]])
    m3_orient_g = [0.5,0.5,0.5,0.5] 

    delta_m2_pos_g = self.goalMotionForHand(mpc_dat.ragdoll_m2_x,
                                          m2_x_g,
                                          dist_g)

    delta_m2_orient_g = self.goalOrientationInQuat(mpc_dat.ragdoll_m2_q_orient,
                                                m2_orient_g,
                                                ang_dist_g)

    delta_m3_pos_g = self.goalMotionForHand(mpc_dat.ragdoll_m3_x,
                                          m3_x_g,
                                          dist_g)

    delta_m3_orient_g = self.goalOrientationInQuat(mpc_dat.ragdoll_m3_q_orient,
                                                m3_orient_g,
                                                ang_dist_g)
 
    #print "rarm_q_forearm_orient: ",mpc_dat.rarm_q_forearm_orient
    #print "delta_r_forearm_orient_g: ",delta_r_forearm_orient_g

# TODO - NEW MATHS. Explicitly apply weights - it's much easier to understand.
    # combine position/orientation goals
    delta_object_x_g = np.vstack( (delta_object_pos_g, delta_object_orient_g) )
    delta_lforearm_x_g = np.vstack( (delta_lforearm_pos_g, delta_lforearm_orient_g) ) 
    delta_rforearm_x_g = np.vstack( (delta_rforearm_pos_g, delta_rforearm_orient_g) ) 
    delta_m2_x_g = np.vstack( (delta_m2_pos_g, delta_m2_orient_g) )
    delta_m3_x_g = np.vstack( (delta_m3_pos_g, delta_m3_orient_g) )
    
    # Transformation to change Jacobian orientation from angular velocity to delta quaternion
    lforearm_J = mpc_dat.lforearm_J[0]
    T_quat = 0.5 * (mpc_dat.lforearm_q_orient[3]*np.matrix(np.eye(3))-haptic_mpc_util.getSkewMatrix(mpc_dat.lforearm_q_orient[0:3]))
    #T_quat2 = -0.5 * (np.matrix(mpc_dat.larm_q_forearm_orient[0:3]))
    #larm_J_f_last_row = T_quat2*larm_J_f[3:]
    lforearm_J[3:] = T_quat*lforearm_J[3:]

    rforearm_J = mpc_dat.rforearm_J[0]
    T_quat = 0.5 * (mpc_dat.rforearm_q_orient[3]*np.matrix(np.eye(3))-haptic_mpc_util.getSkewMatrix(mpc_dat.rforearm_q_orient[0:3]))
    #T_quat2 = -0.5 * (np.matrix(mpc_dat.larm_q_forearm_orient[0:3]))
    #larm_J_f_last_row = T_quat2*larm_J_f[3:]
    rforearm_J[3:] = T_quat*rforearm_J[3:]

    if self.opt.behavior != 'lifting_no_obj_data':   
	larm_object_J = copy.copy(mpc_dat.larm_object_J[0])
        T_quat = 0.5 * (mpc_dat.object_q_orient[3]*np.matrix(np.eye(3))-haptic_mpc_util.getSkewMatrix(mpc_dat.object_q_orient[0:3]))
        larm_object_J[3:] = T_quat*larm_object_J[3:]

	rarm_object_J = mpc_dat.rarm_object_J[0]
        T_quat = 0.5 * (mpc_dat.object_q_orient[3]*np.matrix(np.eye(3))-haptic_mpc_util.getSkewMatrix(mpc_dat.object_q_orient[0:3]))
        rarm_object_J[3:] = T_quat*rarm_object_J[3:]

    	ragdoll_m2_J = mpc_dat.ragdoll_m2_J[0]
    	T_quat = 0.5 * (mpc_dat.ragdoll_m2_q_orient[3]*np.matrix(np.eye(3))-haptic_mpc_util.getSkewMatrix(mpc_dat.ragdoll_m2_q_orient[0:3]))
    	ragdoll_m2_J[3:] = T_quat*ragdoll_m2_J[3:]

    	ragdoll_m3_J = mpc_dat.ragdoll_m3_J[0]
    	T_quat = 0.5 * (mpc_dat.ragdoll_m3_q_orient[3]*np.matrix(np.eye(3))-haptic_mpc_util.getSkewMatrix(mpc_dat.ragdoll_m3_q_orient[0:3]))
    	ragdoll_m3_J[3:] = T_quat*ragdoll_m3_J[3:]
 
    else:
	larm_object_J = np.matrix(np.zeros((6,7)))
	rarm_object_J = np.matrix(np.zeros((6,7)))
	ragdoll_m2_J = np.matrix(np.zeros((6,7)))
	ragdoll_m3_J = np.matrix(np.zeros((6,7)))

    # if both position and orientation (dim = 6, not 3), there are more competing terms in cost function
    # increase the weight on force reduction to compensate
    #if delta_x_g.shape[0] == 6:
    #   mpc_dat.force_weight *= 10

    n_joints = mpc_dat.larm_K_j.shape[0]

    '''
    # If torque constraints are violated - ie Q_des is far from Q - then reset Q_des to Q
    q_diff = np.array(mpc_dat.jep) - np.array(mpc_dat.q)
    max_q_diff = np.max(np.abs(q_diff))
    if max_q_diff > self.angle_reset_threshold: # Reset JEP to Q.
      rospy.loginfo("JEPs too far from current position - capping them to limit")
      new_q_des = np.clip(mpc_dat.jep,
                            mpc_dat.q - self.angle_reset_threshold,
                            mpc_dat.q + self.angle_reset_threshold)
      self.publishDesiredJointAngles(new_q_des.tolist())
      return None

    # If force limit is exceeded, exit and shutdown.  There may be a better behavior.
    if len(mpc_dat.f_l) != 0:
        if np.max(mpc_dat.f_l) > self.stopping_force:
          rospy.logwarn("Haptic MPC: MAXIMUM ALLOWABLE FORCE EXCEEDED.  SHUTTING DOWN!")
          rospy.signal_shutdown("Haptic MPC: MAXIMUM ALLOWABLE FORCE EXCEEDED.  SHUTTING DOWN!")
    '''

    #h = std_msgs.msg.Header()
    #h.stamp = rospy.Time.now()
    #self.mpc_weights_pub.publish(h, mpc_dat.position_weight, mpc_dat.orient_weight, mpc_dat.posture_weight)

    # Postural term input
    delta_theta_des = self.goalDeltaPosture(mpc_dat.goal_posture, mpc_dat.larm_q, self.max_theta_step, self.theta_step_scale)
    if len(delta_theta_des) > n_joints: # Trim to the number of DOFs being used for this - not necessarily the number of joints present, eg Cody 5DOF
      delta_theta_des = delta_theta_des[0:n_joints]

    #lb, ub, m1, m2, m3, m4 = esm.convert_to_qp_posture(larm_J_h, rarm_J_h, # 6xDOF end effector jacobian

    '''
    cost_quadratic_matrices, cost_linear_matrices, \
    constraint_matrices, constraint_vectors, \
    constraint_matrices_eq, constraint_vectors_eq, lb, ub = esm.convert_to_qp_posture(larm_J_h, rarm_J_h, # 6xDOF end effector jacobian
						   larm_J_f, rarm_J_f,
						   mpc_dat.larm_x_forearm, mpc_dat.rarm_x_forearm,
						   mpc_dat.larm_q_h_orient, mpc_dat.rarm_q_h_orient,
						   mpc_dat.larm_q_forearm_orient, mpc_dat.rarm_q_forearm_orient,	
						   delta_l_forearm_orient_g, delta_r_forearm_orient_g,
                                                   mpc_dat.Jc_l, # list of contacts
                                                   mpc_dat.larm_K_j, mpc_dat.rarm_K_j, # joint_stiffnesses (DOFxDOF)
                                                   mpc_dat.Kc_l, # list of 3x3s
                                                   mpc_dat.Rc_l, # list of 3x3s
                                                   mpc_dat.delta_f_min, # num contacts * 1
                                                   mpc_dat.delta_f_max,
                                                   mpc_dat.larm_phi_curr, mpc_dat.rarm_phi_curr, # DOF
                                                   delta_x_g, mpc_dat.f_n, mpc_dat.larm_q, mpc_dat.rarm_q,
                                                   self.larm_joint_limits_min, self.rarm_joint_limits_min, # joint limits (used to be kinematics object)
                                                   self.larm_joint_limits_max, self.rarm_joint_limits_max,
                                                   mpc_dat.jerk_opt_weight, #between 0.000000001 and .0000000001, cool things happened
                                                   mpc_dat.max_force_mag,
                                                   delta_theta_des,
                                                   mpc_dat.posture_weight,
                                                   mpc_dat.position_weight,
                                                   mpc_dat.orient_weight,
                                                   mpc_dat.force_weight,
                                                   mpc_dat.forearm_orient_weight,
                                                   mpc_dat.force_reduction_goal,
                                                   self.angle_constraint_threshold,
						   mpc_dat.miu,
						   mpc_dat.object_height,
						   mpc_dat.object_width,
						   mpc_dat.object_depth,
						   mpc_dat.larm_friction_p1_pose,
                                                   mpc_dat.larm_friction_J1,
                                                   mpc_dat.larm_friction_p2_pose,
                                                   mpc_dat.larm_friction_J2,
                                                   mpc_dat.larm_friction_p3_pose,
                                                   mpc_dat.larm_friction_J3,
						   mpc_dat.rarm_friction_p1_pose,
                                                   mpc_dat.rarm_friction_J1,
                                                   mpc_dat.rarm_friction_p2_pose,
                                                   mpc_dat.rarm_friction_J2,
                                                   mpc_dat.rarm_friction_p3_pose,
                                                   mpc_dat.rarm_friction_J3,
                                                   mpc_dat.object_mass,
                                                   mpc_dat.object_x,
                                                   mpc_dat.object_accel,
                                                   mpc_dat.object_dist_force,
						   mpc_dat.object_lower_body_pos,
						   mpc_dat.object_l_thigh_pos,
						   mpc_dat.object_r_thigh_pos,
						   mpc_dat.object_upper_body_pos,
						   mpc_dat.ragdoll_m1,
						   mpc_dat.ragdoll_m2,
						   mpc_dat.ragdoll_m3,
						   mpc_dat.ragdoll_m1_pos,
						   mpc_dat.ragdoll_m1_orient,
						   mpc_dat.ragdoll_m2_pos,
						   mpc_dat.ragdoll_m2_orient,
						   mpc_dat.ragdoll_m3_pos,
						   mpc_dat.ragdoll_m3_orient,
						   ragdoll_m2_J,
						   ragdoll_m3_J,
						   delta_m2_pos_g,
						   delta_m2_orient_g,
						   delta_m3_pos_g,
                                                   delta_m3_orient_g)

    '''

    cost_quadratic_matrices, cost_linear_matrices, \
    constraint_matrices, constraint_vectors, \
    constraint_matrices_eq, constraint_vectors_eq, lb, ub, \
    current_larm_moment, current_rarm_moment, delta_larm_moment, delta_rarm_moment, \
    delta_lforearm_x, delta_rforearm_x, delta_rarm_moment_x_force1, delta_rarm_moment_x_force2, larm_force, rarm_force, estimated_weight = esm.convert_to_qp_posture(mpc_dat, larm_object_J, rarm_object_J, 
                                                   lforearm_J, rforearm_J, 
						   ragdoll_m2_J, ragdoll_m3_J,
						   delta_object_x_g,
						   delta_lforearm_x_g, delta_rforearm_x_g, 
						   delta_m2_x_g, delta_m3_x_g, 
						   self.larm_joint_limits_min, self.rarm_joint_limits_min,
                                                   self.larm_joint_limits_max, self.rarm_joint_limits_max,
						   self.angle_constraint_threshold)
						   


    #self.larm_J_f = J_f
    #self.q_forearm_orient = mpc_dat.q_forearm_orient
    #lb = lb[0:n_joints]
    #ub = ub[0:n_joints] # Trim the lower/upper bounds to the number of joints being used (may be less than DOF - eg Cody).

    delta_phi_opt, opt_error, feasible = esm.solve_qp(cost_quadratic_matrices,
                                                      cost_linear_matrices,
                                                      constraint_matrices,
                                                      constraint_vectors,
						      constraint_matrices_eq,
                                                      constraint_vectors_eq,
                                                      lb, ub,
                                                      debug_qp=False)

    # Updated joint positions.
    #print "delta_phi_opt: ",delta_phi_opt
    delta_phi = np.zeros(14)
    delta_phi = delta_phi_opt.T.tolist()[0]
    #print "delta_phi: ",delta_phi

    debug_moment_msg = DebugMomentBalancing()
    debug_moment_msg.estimated_weight = estimated_weight
    debug_moment_msg.larm_force += (larm_force).T.tolist()[0]
    debug_moment_msg.rarm_force += (rarm_force).T.tolist()[0]
    debug_moment_msg.current_larm_moment = current_larm_moment[0][0]
    debug_moment_msg.current_rarm_moment = current_rarm_moment[0][0]
    debug_moment_msg.current_net_moment = current_larm_moment[0][0]+current_rarm_moment[0][0]
    debug_moment_msg.delta_larm_moment = delta_larm_moment*np.matrix(delta_phi[:7]).T
    debug_moment_msg.delta_rarm_moment = delta_rarm_moment*np.matrix(delta_phi[7:]).T
    debug_moment_msg.delta_net_moment = delta_larm_moment*np.matrix(delta_phi[:7]).T+delta_rarm_moment*np.matrix(delta_phi[7:]).T
    debug_moment_msg.delta_lforearm_x += (delta_lforearm_x*np.matrix(delta_phi[:7]).T).T.tolist()[0]
    debug_moment_msg.delta_rforearm_x += (delta_rforearm_x*np.matrix(delta_phi[7:]).T).T.tolist()[0]
    debug_moment_msg.predicted_larm_moment = current_larm_moment[0][0]+delta_larm_moment*np.matrix(delta_phi[:7]).T
    debug_moment_msg.predicted_rarm_moment = current_rarm_moment[0][0]+delta_rarm_moment*np.matrix(delta_phi[7:]).T
    debug_moment_msg.predicted_net_moment = current_larm_moment[0][0]+delta_larm_moment*np.matrix(delta_phi[:7]).T+current_rarm_moment[0][0]+delta_rarm_moment*np.matrix(delta_phi[7:]).T
    self.debug_moment_balancing_pub.publish(debug_moment_msg)
    
    '''
    # print moments before delta_phi
    print "current larm_moment: ",current_larm_moment[0][0] 
    print "current rarm_moment: ",current_rarm_moment[0][0] 
    print "current net moment: ",current_larm_moment[0][0]+current_rarm_moment[0][0]

    # print delta moments resulting from delta_phi
    print "delta larm_moment: ",delta_larm_moment*np.matrix(delta_phi[:7]).T
    print "delta rarm_moment: ",delta_rarm_moment*np.matrix(delta_phi[7:]).T
    print "delta net moment: ",delta_larm_moment*np.matrix(delta_phi[:7]).T+delta_rarm_moment*np.matrix(delta_phi[7:]).T

    # print delta moment contributions from rarm_moment compoenents
    print "delta rarm_moment_force1: ",delta_rarm_moment_x_force1*np.matrix(delta_phi[7:]).T
    print "delta rarm_moment_force2: ",delta_rarm_moment_x_force2*np.matrix(delta_phi[7:]).T

    # print delta forearm positions resulting from delta_phi
    print "delta lforearm_x: ",delta_lforearm_x*np.matrix(delta_phi[:7]).T
    print "delta_rforearm_x: ",delta_rforearm_x*np.matrix(delta_phi[7:]).T

    # print moments after delta_phi
    print "predicted larm_moment: ",current_larm_moment[0][0]+delta_larm_moment*np.matrix(delta_phi[:7]).T
    print "predicted rarm_moment: ",current_rarm_moment[0][0]+delta_rarm_moment*np.matrix(delta_phi[7:]).T
    print "predicted net_moment: ",current_larm_moment[0][0]+delta_larm_moment*np.matrix(delta_phi[:7]).T+current_rarm_moment[0][0]+delta_rarm_moment*np.matrix(delta_phi[7:]).T
    '''


    #print "new larm_forearm_orient: ",larm_J_f*np.matrix(delta_phi[:7]).reshape(7,1)
    #print "new rarm_forearm_orient: ",rarm_J_f*np.matrix(delta_phi[7:14]).reshape(7,1)

    '''
    print "test1: ",np.matrix(m1).reshape(1,14)*np.matrix(delta_phi).reshape(14,1)
    print "test2: ",np.matrix(m2).reshape(1,14)*np.matrix(delta_phi).reshape(14,1)
    print "test3: ",np.matrix(m3).reshape(1,14)*np.matrix(delta_phi).reshape(14,1)
    print "test4: ",np.matrix(m4).reshape(1,14)*np.matrix(delta_phi).reshape(14,1)

    # calculate alpha based on plane from three points on arm and plane
    vector1 = np.array(mpc_dat.rarm_friction_p2_pose-mpc_dat.rarm_friction_p1_pose)
    vector2 = np.array(mpc_dat.larm_friction_p3_pose-mpc_dat.rarm_friction_p1_pose)
    vector1 = vector1.reshape(1,3)
    vector2 = vector2.reshape(1,3)
    friction_norm = np.cross(vector1[0],vector2[0])/np.linalg.norm(np.cross(vector1[0],vector2[0]))
    tangent_direction = np.cross(np.cross(np.array([0,0,1]),friction_norm),friction_norm)
    tangent_direction = tangent_direction/np.linalg.norm(tangent_direction)
    alpha = math.acos(np.dot(np.array([0,0,1]),friction_norm))
    if tangent_direction[0] < 0:
        alpha = -alpha

    f_J1 = np.hstack((np.matrix(np.zeros((3,7))),mpc_dat.rarm_friction_J1[0][:3]))
    f_J2 = np.hstack((np.matrix(np.zeros((3,7))),mpc_dat.rarm_friction_J2[0][:3]))
    f_J3 = np.hstack((mpc_dat.larm_friction_J3[0][:3],np.matrix(np.zeros((3,7)))))
    
    d_v2v1_d_theta = (f_J2-f_J1)
    d_v3v1_d_theta = (f_J3-f_J1)
    d_alpha_d_theta = []
    d_f2_d_theta = []
    d_tdir_d_theta = []
    for i in range(14):
        mult_mat = np.matrix(np.zeros(14)).reshape(14,1)
        mult_mat[i] = 1.
        temp1 = np.array(d_v2v1_d_theta*mult_mat).reshape(1,3)
        temp2 = np.array(d_v3v1_d_theta*mult_mat).reshape(1,3)
        temp = (np.cross(temp1,vector2)+np.cross(vector1,temp2))/np.linalg.norm(np.cross(vector1[0],vector2[0]))
        d_f2_d_theta.append(temp.tolist()[0])
        if alpha < 0:
            d_alpha_d_theta_num = np.dot(np.array([0,0,1]),np.array(temp).reshape(1,3)[0])
        else:
            d_alpha_d_theta_num = -np.dot(np.array([0,0,1]),np.array(temp).reshape(1,3)[0])
        d_alpha_d_theta_den = math.sqrt(1-np.dot(np.array([0,0,1]),friction_norm)**2)
        d_alpha_d_theta.append(float(d_alpha_d_theta_num/d_alpha_d_theta_den))
        d_tdir_d_theta_num = np.cross(np.cross(np.array([0,0,1]),np.array(temp).reshape(1,3)[0]),friction_norm)+np.cross(np.cross(np.array([0,0,1]),friction_norm),temp)
        d_tdir_d_theta.append(d_tdir_d_theta_num.tolist()[0])

    # Calculate current alpha/tangent direction and how alpha/tangent direction change in time
    d_alpha_d_theta = np.matrix(d_alpha_d_theta)
    d_f2_d_theta = np.matrix(d_f2_d_theta).T
    d_tdir_d_theta = np.matrix(d_tdir_d_theta).T

    delta_vector1 = np.matrix(d_v2v1_d_theta)*np.matrix(delta_phi).reshape(14,1)
    pred_vector1 = (vector1+delta_vector1.reshape(1,3))
    delta_vector2 = np.matrix(d_v3v1_d_theta)*np.matrix(delta_phi).reshape(14,1)
    pred_vector2 = (vector2+delta_vector2.reshape(1,3))
    delta_friction_norm = d_f2_d_theta*np.matrix(delta_phi).reshape(14,1)
    pred_friction_norm = (friction_norm+delta_friction_norm.reshape(1,3))/np.linalg.norm(friction_norm+delta_friction_norm.reshape(1,3))
    delta_alpha = d_alpha_d_theta*np.matrix(delta_phi).reshape(14,1)
    predicted_alpha = alpha+delta_alpha
    delta_tangent_direction = d_tdir_d_theta*np.matrix(delta_phi).reshape(14,1)
    pred_tangent_direction = (tangent_direction+delta_tangent_direction.reshape(1,3))/np.linalg.norm(tangent_direction+delta_tangent_direction.reshape(1,3))

    
    print "vector1: ",vector1
    print "delta vector1: ",delta_vector1
    print "pred_vector1: ",pred_vector1
    print "vector2: ",vector2
    print "delta vector2: ",delta_vector2
    print "pred_vector2: ",pred_vector2
    print "friction_norm: ",friction_norm
    print "delta friction_norm: ",delta_friction_norm
    print "predicted friction_norm: ",pred_friction_norm
    print "alpha (vector calculated): ",alpha 
    print "delta alpha: ",delta_alpha
    print "d_alpha_d_theta: ",d_alpha_d_theta
    print "predicted alpha (vector calculated): ",predicted_alpha
    print "tangent_direction: ",tangent_direction
    print "d_tdir_d_theta: ",d_tdir_d_theta
    print "delta tangent_direction: ",delta_tangent_direction
    print "predicted tangent_direction: ",pred_tangent_direction
    '''

    # warn if JEP goes beyond joint limits
    #if self.robot_kinematics.within_joint_limits(mpc_dat.jep) == False:
    #  rospy.logwarn('Outside joint limits. They will be clamped later...')
    #  rospy.logwarn("Limits: %s" % str(self.robot_kinematics.joint_lim_dict))
    #  rospy.logwarn("Current JEP: %s" % str(mpc_dat.jep))

    #mechanical_reduction = 1.
    #for i in range(len(delta_phi)):
#	delta_phi[i] = delta_phi[i]*mechanical_reduction

    #print "delta_forearm_pos: ",np.hstack((larm_J_f,[[0.],[0.],[0.],[0.],[0.],[0.]]))*np.matrix(delta_phi).reshape(7,1)

    #print "larm delta forearm roll: ",delta_phi[5]
    #print "rarm delta forearm roll: ",delta_phi[12]
    return delta_phi # Return a joint position delta
    #return mpc_dat.jep # Return the an absolute joint position - old implementation

  ## Computes the contact stiffness matrix, K_ci.
  # @param n_l List of normal forces
  # #param k_default Default stiffness, N/m. Default is typically 200.0-1000.0
  def contactStiffnessMatrix(self, n_l, k_default, k_est_min=100.0, k_est_max=100000.0):
    # This computes the contact stiffness matrices, K_ci
    #
    # K_ci are size 3 x 3
    # K_c is size 3n x 3n
    Kc_l = []
    for n_ci in it.izip(n_l):
      # n_ci is a unit vector located at the point of contact,
      # or at the center of the taxel, that is normal to the
      # surface of the robot and points away from the surface of
      # the robot.
      #
      # This should result in a stiffness matrix that has a
      # stiffness of k in the direction of the normal and a
      # stiffness of 0 in directions orthogonal to the normal.
      #
      # If the contact location moves away from the current
      # contact location along the arm's surface normal, then
      # the magnitude of the force applied by the robot to the
      # environment should increase proportionally to the
      # distance the contact location has moved along the
      # surface normal. Given our conventions, this motion
      # projected onto the surface normal, n_ci, would result in
      # a positive value. Moreover, since we represent forces as
      # the force applied by the robot to the environment, the
      # resulting force vector should point in the same
      # direction.
      #
      n_ci = np.nan_to_num(n_ci)
      K_ci = k_default * np.outer(n_ci, n_ci)

      Kc_l.append(np.matrix(K_ci))
    return Kc_l

  ## Calculates the force transformation matrix from a list of force normals
  # @param n_l List of force normals
  # @return List of transformation matrices (rotations)
  def contactForceTransformationMatrix(self, n_l):
    # Compute R_c, which as originally conceived, would consist of
    # rotation matrices that transform contact forces to the local
    # contact frame R_c f_c_global = f_c_local
    #
    # For now, R_c instead ignores all forces other than the force
    # normal to the surface of the robot's arm at the point of
    # contact. R_c is only used for the constraint matrices. So,
    # R_ci recovers the component of the contact force, f_ci, that
    # is normal to the surface. If f_ci points toward the arm,
    # then it represents the robot pulling on the environment. If
    # f_ci points away from the arm, then it represents the robot
    # pushing on the environment.
    #
    # R_ci f_ci = f_norm_scalar
    # R_ci is size 1 x 3
    # R_c is size n x 3n
    #
    Rc_l = []
    for n_ci in n_l:
      # n_ci is a unit vector that is normal to the surface of
      # the robot and points away from the surface of the robot.
      R_ci = np.matrix(np.zeros((1,3)))
      R_ci[:] = n_ci[:].T
      Rc_l.append(R_ci)
    return Rc_l

  ## Compute bounds for delta_f
  # @retval delta_f_min Minimum bound
  # @retval delta_f_max Maximum bound
  def deltaForceBounds(self, f_n,
                     max_pushing_force, max_pulling_force,
                     max_pushing_force_increase, max_pushing_force_decrease,
                     min_decrease_when_over_max_force,
                     max_decrease_when_over_max_force):
    # Compute bounds for delta_f:  delta_f_max and delta_f_min
    #
    # all inputs should be positive
    assert (max_pushing_force >= 0.0), "delta_f_bounds: max_pushing_force = %f < 0.0" % max_pushing_force
    assert (max_pushing_force_increase >= 0.0), "delta_f_bounds: max_pushing_force_increase = %f < 0.0" % max_pushing_force_increase
    assert (max_pushing_force_decrease >= 0.0), "delta_f_bounds: max_pushing_force_decrease = %f < 0.0" % max_pushing_force_decrease
    assert (max_pulling_force >= 0.0), "delta_f_bounds: max_pulling_force = %f < 0.0" % max_pulling_force
    assert (min_decrease_when_over_max_force >= 0.0), "delta_f_bounds: min_decrease_when_over_max_force = %f < 0.0" % min_decrease_when_over_max_force
    assert (max_decrease_when_over_max_force >= 0.0), "delta_f_bounds: max_decrease_when_over_max_force = %f < 0.0" % max_decrease_when_over_max_force

    # Set delta_f_max. The change to the normal components of
    # the contact forces must be less than these
    # values. delta_f_max limits the magnitude of the force
    # with which the robot can push on the environment at each
    # of its contact locations.
    #
    # f_max is size n x 1
    #
    # Compute how much the contact force can change before it hits
    # the maximum, and limit the expected increase in force by
    # this quantity.
    n = f_n.shape[0]
    f_max = max_pushing_force * np.matrix(np.ones((n,1)))
    delta_f_max = f_max - f_n
    # Also incorporate constraint on the expected increase in the
    # contact force for each contact. This limits the rate of
    # increase in pushing force.
    delta_f_max = np.minimum(delta_f_max, 
                             max_pushing_force_increase * np.matrix(np.ones((n,1))))

    # Set delta_f_min. The change to the normal components of
    # the contact forces must be greater than these
    # values. delta_f_min limits the magnitude of the force
    # with which the robot can pull on the environment at each
    # of its contact locations.
    #
    # f_min is size n x 1
    #
    f_min = -max_pulling_force * np.matrix(np.ones((n,1)))
    delta_f_min = f_min - f_n
    # Also incorporate constraint on the expected change of
    # the contact force for each contact
    delta_f_min = np.maximum(delta_f_min,
                             -max_pushing_force_decrease * np.matrix(np.ones((n,1))))

    # # Setting negative values of delta_f_min to large negative
    # # numbers so that adhesive forces are not a binding constraint
    delta_f_min[np.where(delta_f_min<=0)]=-10000

    # If a force has exceeded the maximum use special constraints.
    over_max = f_n > max_pushing_force
    if over_max.any():
      # at least one of the contact forces is over the maximum allowed
      delta_f_max[over_max] = -min_decrease_when_over_max_force
      delta_f_min[over_max] = -max_decrease_when_over_max_force

    return delta_f_min, delta_f_max

  ## Publishes a list of Deltas for the desired joint positions.
  # @param ctrl_data List of desired joint position deltas to be published
  def larm_publishDeltaControlValues(self, ctrl_data):
    msg = hrl_msgs.msg.FloatArrayBare()
    msg.data = ctrl_data
    self.larm_delta_q_des_pub.publish(msg)

  def rarm_publishDeltaControlValues(self, ctrl_data):
    msg = hrl_msgs.msg.FloatArrayBare()
    msg.data = ctrl_data
    self.rarm_delta_q_des_pub.publish(msg)

  def publishDeltaControlValues(self, ctrl_data):
    msg = hrl_msgs.msg.FloatArrayBare()
    msg.data = ctrl_data
    self.delta_q_des_pub.publish(msg)

  ## Publish a desired joint position list directly (rather than deltas)
  # @param ctrl_data List of desired joint positions to be published (not deltas).
  def publishDesiredJointAngles(self, ctrl_data):
    msg = hrl_msgs.msg.FloatArrayBare()
    msg.data = ctrl_data
    self.q_des_pub.publish(msg)

  ## Main control function. Builds the control data structure and passes it to the control calculation routine
  def updateController(self):
    # Check for controller enabled
    if not self.mpc_enabled:
      return

    # Check for haptic state msg timeout
    with self.state_lock:
      time_since_last_msg = rospy.Time.now() - self.last_msg_time # will always exist as we block on startup until a haptic state msg is heard.

    # Timeout based on time since the last heard message.
    # TODO: Replace this with a freshness parameter on the skin data. More flexible than a timeout.
#    if time_since_last_msg.to_sec() > self.timeout:
#      if self.waiting_to_resume == False:
#        rospy.logwarn("MPC hasn't heard a haptic state messages for %s s. Stopping control effort." % str(time_since_last_msg.to_sec()))
#        self.waiting_to_resume = True
#        #return from updateController without actually doing anything
#        return
#    else:
#      if self.waiting_to_resume == True:
#        self.waiting_to_resume = False
#        rospy.logwarn("MPC resumed control - haptic state messages received again.")
#
    # Listen to a state topic from the monitor node and respond to errors.
    with self.monitor_lock:
      mpc_state = copy.copy(self.mpc_state)
      mpc_error = copy.copy(self.mpc_error)

    if mpc_error != None and len(mpc_error) > 0:
      if self.waiting_for_no_errors == False:
        rospy.logwarn("Haptic MPC: MPC monitor detected error conditions. Stopping control effort.\nState:\n%s\nErrors:\n%s" % (str(mpc_state), str(mpc_error)))
        self.waiting_for_no_errors = True
      return
    else:
      if self.waiting_for_no_errors == True:
        self.waiting_for_no_errors = False
        rospy.logwarn("Haptic MPC: MPC resumed control - errors cleared.")

    # If good, run controller.
    '''
    base_torso_threshold = 0.1
    arm_threshold = 0.01
    try:
        #print rospy.get_namespace(),np.linalg.norm(self.delta_theta_des) 
        #print np.linalg.norm(self.delta_theta_des2)
        #print rospy.get_namespace(),np.linalg.norm(self.delta_theta_des2) - np.linalg.norm(self.delta_theta_des)
        if self.goal_status[0] - max(self.goal_status[1],self.goal_status[2])> base_torso_threshold: 
	    #print rospy.get_namespace()
            desired_joint_pos = None
	elif "r_arm" in rospy.get_namespace() and self.goal_status[1]-self.goal_status[2] > arm_threshold:
	    desired_joint_pos = None
	elif "l_arm" in rospy.get_namespace() and self.goal_status[2]-self.goal_status[1] > arm_threshold:
	    desired_joint_pos = None
	else:
	    mpc_data = self.initMPCData()
            desired_joint_pos = self.deltaQpJepGen(mpc_data) # deltas
    except:
	mpc_data = self.initMPCData()
        desired_joint_pos = self.deltaQpJepGen(mpc_data) # deltas
    '''
    mpc_data = self.initMPCData()
    desired_joint_pos = self.deltaQpJepGen(mpc_data) # deltas
    '''
    if desired_joint_pos == None:
      larm_desired_joint_pos = [0.0] * len(self.larm_joint_angles)
      #print "self.rarm_joint_angles: ",self.rarm_joint_angles
      rarm_desired_joint_pos = [0.0] * len(self.rarm_joint_angles[1:])
    else:
      #print "desired_joint_pos: ",desired_joint_pos
      larm_desired_joint_pos = desired_joint_pos[:7]
      rarm_desired_joint_pos = desired_joint_pos[8:]
    '''
    if desired_joint_pos == None:
	desired_joint_pos = [0.0] * len(self.larm_joint_angles)+[0.0] * len(self.rarm_joint_angles[1:])
    else:
	desired_joint_pos = desired_joint_pos[:7]+desired_joint_pos[8:]
    #print "larm_desired_joint_pos: ",larm_desired_joint_pos
    #print "rarm_desired_joint_pos: ",rarm_desired_joint_pos 
    #print "new object_pos (LEFT): ",self.larm_J_h*np.matrix(desired_joint_pos[:7]).reshape(7,1)
    #print "new object_pos (RIGHT): ",self.rarm_J_h*np.matrix(desired_joint_pos[7:]).reshape(7,1)
    #print self.q_forearm_orient 
    #print "J_f: ",self.J_f
    #print "larm_desired_joint_pos: ",larm_desired_joint_pos
    #t = np.hstack((self.J_f[0],[[0.],[0.],[0.],[0.],[0.],[0.]]))*np.matrix(desired_joint_pos).reshape(7,1)
    #print "commanded quaternion: ",self.J_f*np.matrix(desired_joint_pos[:-1]).reshape(6,1)
    #print (t[3:]).reshape(1,3).tolist()[0]
    #print "commanded angle: ",np.degrees((t[3:]).reshape(1,3).tolist()[0][2])
    #rospy.sleep(0.1)
    #print "self.J_h: ",self.J_h
    #print "commanded delta angles: ", self.J_h*np.matrix(desired_joint_pos).reshape(7,1)
    #print "object_J: ",self.object_J
    #print "delta object pos: ",self.object_J[0]*np.matrix(larm_desired_joint_pos).reshape(7,1)
    #raw_input("Press Enter")
    #rospy.sleep(0.1)
    #print rospy.get_namespace(),np.linalg.norm(desired_joint_pos)
    #self.larm_publishDeltaControlValues(larm_desired_joint_pos)
    #self.rarm_publishDeltaControlValues(rarm_desired_joint_pos)
    self.publishDeltaControlValues(desired_joint_pos)

  ## Initialise the ROS communications - init node, subscribe to the robot state and goal pos, publish JEPs
  def initComms(self, node_name):
    rospy.init_node(node_name)
    self.robot_state_sub = rospy.Subscriber("haptic_mpc/robot_state", haptic_msgs.RobotHapticState, self.robotStateCallback)
    self.goal_pose_sub = rospy.Subscriber("haptic_mpc/traj_pose", GoalPose, self.goalPoseCallback)
    self.goal_posture_sub = rospy.Subscriber("haptic_mpc/goal_posture", hrl_msgs.msg.FloatArray, self.goalPostureCallback)
    self.goal_status_sub = rospy.Subscriber("/crona/goal_status",hrl_msgs.msg.FloatArray,self.goal_status_callback)
    
    self.debug_moment_balancing_pub = rospy.Publisher("haptic_mpc/debug_moment_balancing", DebugMomentBalancing)

    #self.larm_delta_q_des_pub= rospy.Publisher("haptic_mpc/larm_delta_q_des", hrl_msgs.msg.FloatArrayBare)
    #self.rarm_delta_q_des_pub= rospy.Publisher("haptic_mpc/rarm_delta_q_des", hrl_msgs.msg.FloatArrayBare)
    self.delta_q_des_pub= rospy.Publisher("haptic_mpc/delta_q_des", hrl_msgs.msg.FloatArrayBare) 
    
    self.deadzone_pub= rospy.Publisher("haptic_mpc/deadzone", std_msgs.msg.Bool)
    
    self.delta_theta_des_pub= rospy.Publisher("haptic_mpc/delta_theta_des", hrl_msgs.msg.FloatArray)
    
    self.q_des_pub = rospy.Publisher("haptic_mpc/q_des", hrl_msgs.msg.FloatArrayBare)

    self.dist_to_goal_pub = rospy.Publisher("haptic_mpc/dist_to_goal", hrl_msgs.msg.FloatArray)

    self.mpc_monitor_sub = rospy.Subscriber("haptic_mpc/mpc_state", haptic_msgs.HapticMpcState, self.mpcMonitorCallback)

    self.mpc_weights_sub = rospy.Subscriber("haptic_mpc/weights", haptic_msgs.HapticMpcWeights, self.updateWeightsCallback)
    self.mpc_weights_pub = rospy.Publisher("haptic_mpc/current_weights", haptic_msgs.HapticMpcWeights, latch=True)

    self.mpc_terminate_sub = rospy.Subscriber("/haptic_mpc/terminate",std_msgs.msg.Bool,self.mpc_terminate_callback)

  def mpc_terminate_callback(self,msg):
    if msg.data == 1:
        self.mpc_terminate = 1

    #self.enable_mpc_srv = rospy.Service("haptic_mpc/enable_mpc", haptic_srvs.EnableHapticMPC, self.enableHapticMPC)

  def goal_status_callback(self,msg):
    self.goal_status = msg.data

  ## Enable Haptic MPC service handler (default is enabled).
  #def enableHapticMPC(req):
  #  if req.new_state == "enabled":
  #    self.mpc_enabled = True
  #  else:
  #    self.mpc_enabled = False
#
#    return haptic_srvs.EnableHapticMPCResponse(self.mpc_state)

  ## Handler for Ctrl-C signals. Some of the ROS spin loops don't respond well to
  # Ctrl-C without this.
  def signal_handler(self, signal, frame):
    print 'Ctrl+C pressed - exiting'
    sys.exit(0)


  ## Start the control loop once the controller is initialised.
  def start(self):
    signal.signal(signal.SIGINT, self.signal_handler) # Catch Ctrl-Cs

    self.initRobot()
    self.initComms("haptic_mpc")
    rospy.sleep(1.0) # Delay to allow the ROS node to initialise properly.

    self.initControlParametersFromServer()

    r = rospy.Rate(self.frequency)
    self.time_step = 1.0/self.frequency

    rospy.loginfo("Haptic MPC: Waiting for Robot Haptic State message")
    while not self.getJointAngles():
      r.sleep()
    rospy.loginfo("Haptic MPC: Got Robot Haptic State message")

    rospy.loginfo("Haptic MPC: Resetting desired joint angles to current position")
    self.publishDesiredJointAngles(self.getJointAngles())

    # Main control loop
    rospy.loginfo("Haptic MPC: Starting MPC")

    self.mpc_terminate = 0
    while self.robot_state_msg_received == False:
	pass
 
    while self.mpc_terminate == 0:
      self.updateController()
      #rospy.spin() # For debug - run once
      r.sleep()

if __name__== "__main__":
  import optparse
  p = optparse.OptionParser()
  haptic_mpc_util.initialiseOptParser(p)
  opt = haptic_mpc_util.getValidInput(p)

  mpc_controller = HapticMPC(opt, "haptic_mpc")
  mpc_controller.start()

