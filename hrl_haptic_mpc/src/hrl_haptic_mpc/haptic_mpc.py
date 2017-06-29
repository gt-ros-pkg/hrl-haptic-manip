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

# @package hrl_haptic_mpc
# @author Jeff Hawke
# @version 0.1
# @copyright Apache 2.0

import numpy as np
import threading
import copy
import itertools as it
import sys
import signal

import rospy
import geometry_msgs.msg
import std_msgs.msg
import hrl_msgs.msg

import hrl_lib.util as ut
import hrl_lib.transforms as tr

import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import hrl_haptic_manipulation_in_clutter_srvs.srv as haptic_srvs

from hrl_haptic_mpc import epc_skin_math as esm  # Core maths functions used by the MPC
from hrl_haptic_mpc import multiarray_to_matrix  # Helper class with functions to convert a Float64MultiArray to a list of numpy matrices
from hrl_haptic_mpc import haptic_mpc_util  # Utility script with helper functions used across all haptic mpc nodes.

QUEUE_SIZE = 1000


class MPCData(object):
    """ Container class for controller parameters."""
    # Some of these are parameters rather than control data, but they're small cf. the data which changes at each timestep.
    def __init__(self, q, x_h, x_g, dist_g, ang_dist_g, goal_posture,
                 q_h_orient, q_g_orient,
                 position_weight, orient_weight, posture_weight, force_weight,
                 force_reduction_goal,
                 control_point_joint_num,
                 Kc_l, Rc_l, Jc_l, Je,
                 delta_f_min, delta_f_max,
                 phi_curr, K_j,
                 loc_l, n_l, f_l, f_n,
                 jerk_opt_weight, max_force_mag,
                 jep, time_step, stop):

        self.q = q            # Joint angles
        self.x_h = x_h        # end effector position
        self.x_g = x_g        # end effector goal
        self.dist_g = dist_g  # dist to goal
        self.ang_dist_g = ang_dist_g  # angular distance to goal
        self.goal_posture = goal_posture
        self.q_h_orient = q_h_orient  # end effector orientation
        self.q_g_orient = q_g_orient
        self.position_weight = position_weight
        self.orient_weight = orient_weight
        self.posture_weight = posture_weight
        self.force_weight = force_weight
        self.force_reduction_goal = force_reduction_goal
        self.control_point_joint_num = control_point_joint_num
        self.Kc_l = Kc_l
        self.Rc_l = Rc_l
        self.Jc_l = Jc_l
        self.Je = Je
        self.delta_f_min = delta_f_min
        self.delta_f_max = delta_f_max
        self.phi_curr = phi_curr
        self.K_j = K_j
        self.loc_l = loc_l
        self.n_l = n_l
        self.f_l = f_l
        self.f_n = f_n
        self.jerk_opt_weight = jerk_opt_weight
        self.max_force_mag = max_force_mag
        self.jep = jep
        self.time_step = time_step
        self.stop = stop

    # String representation of the data structure. Useful for debugging.
    def __str__(self):
        string = "MPC Data Structure:"
        string += "\nq: \t\t%s" % str(self.q)
        string += "\nx_h: \t\t%s" % str(self.x_h)
        string += "\nx_g: \t\t%s" % str(self.x_g)
        string += "\ndist_g: \t\t%s" % str(self.dist_g)  # dist to goal
        string += "\nang_dist_g: \t\t%s" % str(self.ang_dist_g)  # dist to goal
        string += "\nq_h_orient: \t\t%s" % str(self.q_h_orient)  # end effector orientation
        string += "\nq_g_orient: \t\t%s" % str(self.q_g_orient)
        string += "\nposition_weight: \t\t%s" % str(self.position_weight)
        string += "\norient_weight: \t\t%s" % str(self.orient_weight)
        string += "\ncontrol_point_joint_num: \t\t%s" % str(self.control_point_joint_num)
        string += "\nKc_l: \t\t%s" % str(self.Kc_l)
        string += "\nRc_l: \t\t%s" % str(self.Rc_l)
        string += "\nJc_l: \t\t%s" % str(self.Jc_l)
        string += "\nJe: \t\t%s" % str(self.Je)
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


class HapticMPC(object):
    """Haptic Model Predictive Controller class."""
    # Constructor
    # @param opt optparse options. Should be created using the helper utility script for reliability.
    # @param node_name Name used for the ROS node.
    def __init__(self, opt, node_name="haptic_mpc"):
        rospy.loginfo("Initialising Haptic MPC")
        self.opt = opt

        self.verbose = opt.verbose
        self.state_lock = threading.RLock()  # Haptic state lock
        self.goal_lock = threading.RLock()  # Goal state lock
        self.monitor_lock = threading.RLock()  # Monitor state lock
        self.gain_lock = threading.RLock()  # Controller gain state lock
        self.posture_lock = threading.RLock()  # Goal posture lock.
        self.msg = None  # Haptic state message

        # Haptic State parameters - from a RobotHapticState listener
        self.last_msg_time = None
        self.timeout = 0.50  # If the last time the MPC heard a state message was >50ms ago -> Stop!
        self.waiting_to_resume = False
        self.waiting_for_no_errors = False
        self.mpc_state = None  # Current MPC state. Stored as a list of strings
        self.mpc_error = None  # Current MPC errors. Stored as a list of strings

        self.joint_names = []
        self.joint_angles = []
        self.desired_joint_angles = []
        self.joint_velocities = []
        self.joint_stiffness = []
        self.joint_damping = []

        self.end_effector_pos = None
        self.end_effector_orient_quat = None

        self.skin_data = []
        self.Jc = []
        self.Je = []

        self.time_step = 0.01  # seconds. NB: This is a default which is set by the "start" function.

        # Trajectory goal position - from a PoseStamped listener
        self.goal_pos = None
        self.goal_orient_quat = None
        self.goal_posture = None

        self.currently_in_deadzone = False
        self.mpc_enabled = True

        # Jacobian MultiArray to Matrix converter
        self.ma_to_m = multiarray_to_matrix.MultiArrayConverter()

    # Read parameters from the ROS parameter server and store them.
    def initControlParametersFromServer(self):
        base_path = 'haptic_mpc'
        control_path = '/control_params'

        rospy.loginfo("Haptic MPC: Initialising controller parameters from server. Path: %s", base_path+control_path)
        # controller parameters
        # Force limits for the controller.
        self.allowable_contact_force = rospy.get_param(base_path + control_path + '/allowable_contact_force')  # Max force allowed by the controller
        self.max_delta_force_mag = rospy.get_param(base_path + control_path + '/max_delta_force_mag')  # Max change in force allowed.
        self.stopping_force = rospy.get_param(base_path + control_path + '/stopping_force')  # Completely shut down if this exceeded

        self.goal_velocity_for_hand = rospy.get_param(base_path + control_path + '/goal_velocity_for_hand')
        self.goal_ang_velocity_for_hand = np.radians(rospy.get_param(base_path + control_path + '/goal_ang_velocity_for_hand'))
        self.deadzone_distance = rospy.get_param(base_path + control_path + '/deadzone_distance')
        self.deadzone_angle = np.radians(rospy.get_param(base_path + control_path + '/deadzone_angle'))
        self.angle_reset_threshold = np.radians(rospy.get_param(base_path + control_path + '/angle_reset_threshold'))
        self.angle_constraint_threshold = np.radians(rospy.get_param(base_path + control_path + '/angle_constraint_threshold'))

        # stiffness parameters
        self.static_contact_stiffness_estimate = rospy.get_param(base_path + control_path + '/static_contact_stiffness_estimate')
        self.estimate_contact_stiffness = rospy.get_param(base_path + control_path + '/estimate_contact_stiffness')

        self.orient_weight = rospy.get_param(base_path + control_path + '/orientation_weight')
        self.pos_weight = rospy.get_param(base_path + control_path + '/position_weight')
        self.posture_weight = 0.0  # Default to 'pose' settings, will be set by msg from waypoint_gen if posture goal received
        self.force_weight = rospy.get_param(base_path + control_path + '/force_reduction_weight')
        self.jerk_opt_weight = rospy.get_param(base_path + control_path + '/jerk_opt_weight')
        self.position_weight_scaling_radius = rospy.get_param(base_path + control_path + '/position_weight_scaling_radius')
        self.orientation_weight_scaling_radius = np.radians(rospy.get_param(base_path + control_path + '/orientation_weight_scaling_radius'))
        self.mpc_weights_pub.publish(std_msgs.msg.Header(), self.pos_weight, self.orient_weight, self.posture_weight)

        self.position_step_scaling_radius = rospy.get_param(base_path + control_path + '/position_step_scaling_radius')
        self.orientation_step_scaling_radius = np.radians(rospy.get_param(base_path + control_path + '/orientation_step_scaling_radius'))

        self.max_theta_step = rospy.get_param(base_path + control_path + '/posture_step_size')
        self.theta_step_scale = rospy.get_param(base_path + control_path + '/posture_step_scale')

        self.force_reduction_goal = rospy.get_param(base_path + control_path + '/force_reduction_goal')

        self.frequency = rospy.get_param(base_path + control_path + '/frequency')

    # Initialise the robot joint limits.
    # This relies on something (usually the haptic state node) pushing the appropriate joint limits to the param server before execution.
    # NB: All joint limits are specified in degrees and converted to radians here (easier to understand)
    def initRobot(self):
        base_path = rospy.get_namespace() + 'haptic_mpc'
        joint_limits_max_param = base_path + '/' + self.opt.robot + '/joint_limits/max'
        joint_limits_min_param = base_path + '/' + self.opt.robot + '/joint_limits/min'

        while not (rospy.has_param(joint_limits_max_param) and
                   rospy.has_param(joint_limits_min_param)):
            rospy.loginfo("[%s] Waiting for joint limit parameters to be set", rospy.get_name())
            rospy.sleep(2.0)
        self.joint_limits_max = np.array([np.radians(ang) if isinstance(ang, float) else np.radians(36000) for ang in rospy.get_param(joint_limits_max_param)])
        self.joint_limits_min = np.array([np.radians(ang) if isinstance(ang, float) else np.radians(-36000) for ang in rospy.get_param(joint_limits_min_param)])
        rospy.loginfo("[%s]Got joint limits, continuing.", rospy.get_name())

    # getSkinData accessor function.
    # @return A copy of the skin_data dictionary, containing the latest taxel data.
    def getSkinData(self):
        with self.state_lock:
            skin_data = copy.copy(self.skin_data)
        return skin_data

    # getJointAngles accessor function.
    # @return A copy of the buffered joint angles list.
    def getJointAngles(self):
        with self.state_lock:
            joint_angles = copy.copy(self.joint_angles)
        return joint_angles

    # getDesiredJointAngles accessor function.
    # @return A copy of the desired joint angles list, ie, the current arm controller setpoint.
    def getDesiredJointAngles(self):
        with self.state_lock:
            desired_joint_angles = copy.copy(self.desired_joint_angles)
        return desired_joint_angles

    # getJointStiffness accessor function.
    # @return A copy of the joint stiffness parameters used by the controller.
    def getJointStiffness(self):
        with self.state_lock:
            joint_stiffness = copy.copy(self.joint_stiffness)
        return joint_stiffness

    # Builds the MPC data structure from the individual parameters
    # This is just a convenience data structure to amalgamate the data.
    # Also does data validation.
    def initMPCData(self):
        # Copy the latest data received from the robot haptic state.
        # To ensure the data is synchronised, all necessary data is copied with the lock rather than using the accessor functions.
        with self.state_lock:
            q = copy.copy(self.joint_angles)
            q_des = copy.copy(self.desired_joint_angles)
            Jc = copy.copy(self.Jc)
            skin_data = copy.copy(self.skin_data)
            ee_pos = copy.copy(self.end_effector_pos)
            ee_orient_quat = copy.copy(self.end_effector_orient_quat)
            joint_stiffness = copy.copy(self.joint_stiffness)
            Je = copy.copy(self.Je)

        # Copy the goal parameters.
        with self.goal_lock:
            goal_pos = copy.copy(self.goal_pos)
            goal_orient_quat = copy.copy(self.goal_orient_quat)

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
        Kc_l = self.contactStiffnessMatrix(n_l, k_default)  # Using default k_est_min and k_est_max
        Rc_l = self.contactForceTransformationMatrix(n_l)

        # calculate the normal components of the current contact forces
        tmp_l = [R_ci * f_ci for R_ci, f_ci in it.izip(Rc_l, f_l)]
        f_n = np.matrix([tmp_i[0, 0] for tmp_i in tmp_l]).T

        delta_f_min, delta_f_max = self.deltaForceBounds(f_n,
                                                         max_pushing_force=self.allowable_contact_force,
                                                         max_pulling_force=self.allowable_contact_force,
                                                         max_pushing_force_increase=self.max_delta_force_mag,
                                                         max_pushing_force_decrease=self.max_delta_force_mag,
                                                         min_decrease_when_over_max_force=0.,
                                                         max_decrease_when_over_max_force=10.0)

        q_des_matrix = (np.matrix(q_des)[0:len(q_des)]).T

        K_j = np.diag(joint_stiffness)

        if len(Je) >= 1:  # Je could potentially be a list of end effector jacobians. We only care about the first one in this implementation.
            Je = Je[0]

        # Calculate MPC orientation/position weight. This is a bad hack to essentially damp the response as it approaches the goal.
        with self.gain_lock:
            orient_weight = self.orient_weight
            position_weight = self.pos_weight
            posture_weight = self.posture_weight
        jerk_opt_weight = self.jerk_opt_weight  # Not adjustable during runtime, don't need lock

        # Evalute current goals to give the controller some meaningful input
        # If the goal is non-existant, use the current position/orientation/posture as the goal.
        # Position
        x_h = ee_pos  # numpy array
        x_g = goal_pos if goal_pos is not None else x_h
        cart_error = np.linalg.norm(x_g - x_h)

        # Orientation
        q_h_orient = ee_orient_quat
        q_g_orient = goal_orient_quat if goal_orient_quat is not None else q_h_orient
        angle_error = abs(ut.quat_angle(q_h_orient, q_g_orient))

        # Posture
        if goal_posture is None:
            goal_posture = q

        position_weight_scale_factor = 1.0
        orient_weight_scale_factor = 1.0
        if orient_weight != 0:
            if cart_error < self.position_weight_scaling_radius:
                position_weight_scale_factor = cart_error / self.position_weight_scaling_radius
                position_weight *= position_weight_scale_factor
            if angle_error < self.orientation_weight_scaling_radius:
                orient_weight_scale_factor = angle_error / self.orientation_weight_scaling_radius
                orient_weight *= orient_weight_scale_factor

        # PMG - NOV 2015 - Commented out.  System seems to perform better with constant jerk_opt_weight
        # the position and orientation weights scale down near goal, slowing the controller as jerk_opt_weight
        # becomes relatively more important
        # Reduce the weight of jerk reduction term so that it doesn't dominate.
        # This function is arbitrary, gives good behavior on PR2
        jerk_opt_weight *= np.mean([position_weight_scale_factor, orient_weight_scale_factor])
        # Original re-weighting function used for scaling jerk_opt_weight
        # jerk_opt_weight = max(jerk_opt_weight * position_weight * 2, jerk_opt_weight)

        # Initialise and return the mpc data structure.
        mpc_data = MPCData(q=q,
                           x_h=x_h,  # numpy array
                           x_g=x_g,  # numpy array
                           dist_g=dist_g,
                           ang_dist_g=ang_dist_g,
                           goal_posture=goal_posture,
                           q_h_orient=q_h_orient,
                           q_g_orient=q_g_orient,
                           position_weight=position_weight,
                           orient_weight=orient_weight,
                           posture_weight=posture_weight,
                           force_weight=self.force_weight,
                           force_reduction_goal=self.force_reduction_goal,  # default desired delta for force reduction.
                           control_point_joint_num=len(q),  # number of joints
                           Kc_l=Kc_l,
                           Rc_l=Rc_l,
                           Jc_l=Jc,
                           Je=Je,  # NB: Je is a list of end effector jacobians (to use the same utils functions as Jc)
                           delta_f_min=delta_f_min,
                           delta_f_max=delta_f_max,
                           phi_curr=q_des_matrix,
                           K_j=K_j,
                           loc_l=loc_l,  # Contact locations from taxels. From skin client.
                           n_l=n_l,  # Normals for force locations
                           f_l=f_l,  # Cartesian force components for contacts
                           f_n=f_n,  # Normal force for contacts
                           jerk_opt_weight=jerk_opt_weight,  # From control params
                           max_force_mag=self.allowable_contact_force,  # From control params
                           jep=q_des,
                           time_step=self.time_step,
                           stop=False)
        return mpc_data

    # Update the position/orientation weights used by the controller.
    # @param msg HapticMpcWeights message object
    def updateWeightsCallback(self, msg):
        with self.gain_lock:
            rospy.loginfo("Updating MPC weights. Pos: %s, Orient: %s, Posture: %s",
                          msg.position_weight, msg.orient_weight, msg.posture_weight)
            self.pos_weight = msg.position_weight
            self.orient_weight = msg.orient_weight
            self.posture_weight = msg.posture_weight
            self.mpc_weights_pub.publish(msg)  # Echo the currently used weights. Used as an ACK, basically.

    # Store the current trajectory goal. The controller will always attempt a linear path to this.
    # @param msg A PoseStamped message
    def goalPoseCallback(self, msg):
        with self.goal_lock:
            self.goal_pos = np.matrix([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])
            self.goal_orient_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

    # Store the current posture goal. The controller attempts to achieve this posture.
    # TODO: Define message type. Ideally just joint angles. Float64 list?
    # @param msg An array of Float64s. Type hrl_msgs.FloatArray.
    def goalPostureCallback(self, msg):
        with self.posture_lock:
            self.goal_posture = msg.data  # Store it as a python list of floats

    # Store the state information from the monitor node. Allows the control to be somewhat stateful.
    # The state and error fields are lists of strings indicating some state.
    # @param msg HapticMpcState message object
    def mpcMonitorCallback(self, msg):
        with self.monitor_lock:
            self.mpc_state = msg.state
            self.mpc_error = msg.error

    # Store the robot haptic state.
    # @param msg RobotHapticState message object
    def robotStateCallback(self, msg):
        with self.state_lock:
            self.last_msg_time = rospy.Time.now()  # timeout for the controller
            self.msg = msg
            self.joint_names = msg.joint_names
            self.joint_angles = list(msg.joint_angles)
            self.desired_joint_angles = list(msg.desired_joint_angles)
            self.joint_velocities = list(msg.joint_velocities)
            self.joint_stiffness = list(msg.joint_stiffness)
            self.joint_damping = list(msg.joint_damping)
            self.end_effector_pos = np.matrix([[msg.hand_pose.position.x], [msg.hand_pose.position.y], [msg.hand_pose.position.z]])
            self.end_effector_orient_quat = [msg.hand_pose.orientation.x, msg.hand_pose.orientation.y, msg.hand_pose.orientation.z, msg.hand_pose.orientation.w]
            self.skin_data = msg.skins
            self.Je = self.ma_to_m.multiArrayToMatrixList(msg.end_effector_jacobian)
            self.Jc = self.ma_to_m.multiArrayToMatrixList(msg.contact_jacobians)

    # Interpolate a step towards the given goal orientation.
    # @param q_h_orient The current hand orientation as a quaternion in list form: [x,y,z,w]
    # @param q_g_orient The current goal orientation as a quaternion in list form: [x,y,z,w]
    # @return A desired change in orientation as a delta:
    def goalOrientationInQuat(self, q_h_orient, q_g_orient, ang_dist_g):
        # If the goal position is invalid, return the current end effector position
        if not q_g_orient:
            q_g_orient = q_h_orient
        ang = ut.quat_angle(q_h_orient, q_g_orient)
        ang_mag = abs(ang)
        if ang_mag < 1e-6:
            return np.matrix([0., 0., 0.]).T

        if ang_mag > self.orientation_step_scaling_radius:
            slerp_fraction = ang_dist_g / ang_mag  # move the fraction equal to the max ang vel per step
        else:
            slerp_fraction = ang_dist_g / self.orientation_step_scaling_radius

        interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, slerp_fraction)
        delta_q_des = tr.tft.quaternion_multiply(interp_q_goal, tr.tft.quaternion_inverse(q_h_orient))
        return np.matrix(delta_q_des[0:3]).T

    # Interpolate a step towards the given goal position.
    # @return A goal delta in position as a numpy matrix.
    # @param x_h The current hand position as a numpy matrix: [x,y,z]
    # @param x_g The current goal position as a numpy matrix: [x,y,z]
    # @param dist_g The max delta position per step (vel x freq)
    def goalMotionForHand(self, x_h, x_g, dist_g):
        # Goal should be None if no goal pose has been heard yet, or a numpy column vector.
        if x_g is None or x_g.size == 0:
            x_g = x_h
        err = (x_g - x_h).A
        err_mag = np.linalg.norm(err)
        if err_mag > self.position_step_scaling_radius:
            delta_x_g = dist_g * (err / err_mag)
        else:
            delta_x_g = dist_g * (err / self.position_step_scaling_radius)
        return delta_x_g

    # Process the goal posture input before sending it to the controller.
    # @param goal_posture List of desired joint angles. Should be
    # @param current_posture List of current joint angles
    #
    def goalDeltaPosture(self, goal_posture, current_posture, max_theta_step, num_steps_scale):
        assert max_theta_step >= 0
        assert num_steps_scale >= 0
        # NB: max_theta_step, num_steps_scale must both be POSITIVE.
        delta_theta_des = np.matrix(goal_posture).T - np.matrix(current_posture).T  # error term.
        # delta_theta_des = 0.1 * delta_theta_des # scale the absolute error

        # ONE SCALING METHOD
        for i in range(len(delta_theta_des)):
            theta = delta_theta_des[i]  # theta is the error relative to the desired posture on each joint.
            # num_steps_scale is a parameter which represents how many steps out the delta will be scaled down.
            if abs(theta) > num_steps_scale * max_theta_step:
                theta = max_theta_step * theta/abs(theta)  # If we're more than the num steps away, just cap the theta step.
            else:
                theta = theta/num_steps_scale
                # Eg, if we have a step size of 1.0 deg, num_steps is 10.0
                # If we're 8 steps from the goal, we only command 0.8 deg rather than 1.0 (8/10)
            delta_theta_des[i] = theta

        return delta_theta_des

    # Main control calculation.
    # @param mpc_dat An MPCData object, populated with relevant control parameters and current data
    # @return A list of joint equilibrium point deltas, ie, a desired increment to the current joint position(s).
    def deltaQpJepGen(self, mpc_dat):
        # If we have invalid control data, do nothing
        if mpc_dat is None:
            return None

        # If we're within the deadzone band, do nothing
        dist_to_goal = np.linalg.norm(mpc_dat.x_h - mpc_dat.x_g)
        ang_to_goal = ut.quat_angle(mpc_dat.q_h_orient, mpc_dat.q_g_orient)
        dist_g = mpc_dat.dist_g  # The computed step based on the controller frequency and desired cartesian velocity.
        ang_dist_g = mpc_dat.ang_dist_g

        # ################################## #
        # Added term to never enter deadzone when using posture control (posture_weight > 0) Phil + Daehyung 1MAY16
        in_deadzone = True
        if mpc_dat.position_weight > 0.0 and dist_to_goal > self.deadzone_distance:
            in_deadzone = False
        if mpc_dat.orient_weight > 0.0 and abs(ang_to_goal > self.deadzone_angle):
            in_deadzone = False
        if mpc_dat.posture_weight > 0.0:
            in_deadzone = False
        # #################################### #

        # Don't command any pose motion
        if in_deadzone:
            dist_g = 0.0
            ang_dist_g = 0.0
            if not self.currently_in_deadzone:
                if self.verbose:
                    rospy.loginfo("MPC entered deadzone: pos %s (%s); orient %s (%s)",
                                  dist_to_goal,
                                  self.deadzone_distance,
                                  np.degrees(ang_to_goal),
                                  np.degrees(self.deadzone_angle))
                self.currently_in_deadzone = True
                self.mpc_deadzone_pub.publish(True)

            # If we're in the deadzone and have no forces, return zeroes.
            if len(mpc_dat.loc_l) == 0:
                return [0.0] * len(self.joint_angles)
        else:
            if self.currently_in_deadzone:
                self.currently_in_deadzone = False
                self.mpc_deadzone_pub.publish(False)

        # Generate the position/orientation deltas. These are slewed to set step size.
        delta_pos_g = self.goalMotionForHand(mpc_dat.x_h, mpc_dat.x_g, dist_g)
        delta_orient_g = self.goalOrientationInQuat(mpc_dat.q_h_orient, mpc_dat.q_g_orient, ang_dist_g)
        # combine position/orientation goals
        delta_x_g = np.vstack((delta_pos_g, delta_orient_g))

        J_h = copy.copy(mpc_dat.Je)
        # PMG - NOV 2015 - Changed from tranform based on hand quaternion to error quaternion
        # This seems to fix a number of issues, and seems reasonable
        # See Springer Robotics (Modelling, Planning, and Control) Sec. 3.7 for details on this math
        # Also see "Closed-loop manipulator control using quaternion feedback", Yuan 1988
        # Original
#        T_quat = 0.5 * (mpc_dat.q_h_orient[3] * np.matrix(np.eye(3)) -
#                        haptic_mpc_util.getSkewMatrix(mpc_dat.q_h_orient[0:3]))
        # New version
        w = 1 - np.linalg.norm(delta_orient_g)  # not returned above, but re-find from unit normalization
        T_quat = 0.5 * (w * np.matrix(np.eye(3)) -
                        haptic_mpc_util.getSkewMatrix(delta_orient_g[:3]))
        J_h[3:] = T_quat*J_h[3:]

        # if both position and orientation (dim = 6, not 3), there are more competing terms in cost function
        # increase the weight on force reduction to compensate
        if delta_x_g.shape[0] == 6:
            mpc_dat.force_weight *= 10

        n_joints = mpc_dat.K_j.shape[0]
        J_h[:, mpc_dat.control_point_joint_num:] = 0.
        J_h = np.matrix(J_h[:, 0:n_joints])  # comes into play with Cody and the wrist cover

        # If torque constraints are violated - ie Q_des is far from Q - then reset Q_des to Q
        q_diff = np.array(mpc_dat.jep) - np.array(mpc_dat.q)
        max_q_diff = np.max(np.abs(q_diff))
        if max_q_diff > self.angle_reset_threshold and self.mpc_enabled:  # Reset JEP to Q.
            if self.verbose:
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
                if self.verbose:
                    print np.max(mpc_dat.f_l)
                rospy.signal_shutdown("Haptic MPC: MAXIMUM ALLOWABLE FORCE EXCEEDED.  SHUTTING DOWN!")

        # Postural term input
        delta_theta_des = self.goalDeltaPosture(mpc_dat.goal_posture, mpc_dat.q, self.max_theta_step, self.theta_step_scale)
        if len(delta_theta_des) > n_joints:  # Trim to the number of DOFs being used for this - not necessarily the number of joints present, eg Cody 5DOF
            delta_theta_des = delta_theta_des[0:n_joints]

        (cost_quadratic_matrices,
         cost_linear_matrices,
         constraint_matrices,
         constraint_vectors,
         lb, ub) = esm.convert_to_qp_posture(J_h,  # 6xDOF end effector jacobian
                                             mpc_dat.Jc_l,  # list of contacts
                                             mpc_dat.K_j,  # joint_stiffnesses (DOFxDOF)
                                             mpc_dat.Kc_l,  # list of 3x3s
                                             mpc_dat.Rc_l,  # list of 3x3s
                                             mpc_dat.delta_f_min,  # num contacts * 1
                                             mpc_dat.delta_f_max,
                                             mpc_dat.phi_curr,  # DOF
                                             delta_x_g, mpc_dat.f_n, mpc_dat.q,
                                             self.joint_limits_min,  # joint limits (used to be kinematics object)
                                             self.joint_limits_max,
                                             mpc_dat.jerk_opt_weight,  # between 0.000000001 and .0000000001, cool things happened
                                             mpc_dat.max_force_mag,
                                             delta_theta_des,
                                             mpc_dat.posture_weight,
                                             mpc_dat.position_weight,
                                             mpc_dat.orient_weight,
                                             mpc_dat.force_weight,
                                             mpc_dat.force_reduction_goal,
                                             self.angle_constraint_threshold)
        lb = lb[0:n_joints]
        ub = ub[0:n_joints]  # Trim the lower/upper bounds to the number of joints being used (may be less than DOF - eg Cody).
        delta_phi_opt, opt_error, feasible = esm.solve_qp(cost_quadratic_matrices,
                                                          cost_linear_matrices,
                                                          constraint_matrices,
                                                          constraint_vectors,
                                                          lb, ub,
                                                          debug_qp=False,
                                                          verbose=self.verbose)

        # Updated joint positions.
        delta_phi = np.zeros(len(self.joint_names))
        delta_phi[0:n_joints] = delta_phi_opt.T

        # warn if JEP goes beyond joint limits
        # if self.robot_kinematics.within_joint_limits(mpc_dat.jep) == False:
        #  rospy.logwarn('Outside joint limits. They will be clamped later...')
        #  rospy.logwarn("Limits: %s" % str(self.robot_kinematics.joint_lim_dict))
        #  rospy.logwarn("Current JEP: %s" % str(mpc_dat.jep))

        return delta_phi  # Return a joint position delta
        # return mpc_dat.jep # Return the an absolute joint position - old implementation

    # Computes the contact stiffness matrix, K_ci.
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

    # Calculates the force transformation matrix from a list of force normals
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
        Rc_l = []
        for n_ci in n_l:
            # n_ci is a unit vector that is normal to the surface of
            # the robot and points away from the surface of the robot.
            R_ci = np.matrix(np.zeros((1, 3)))
            R_ci[:] = n_ci[:].T
            Rc_l.append(R_ci)
        return Rc_l

    # Compute bounds for delta_f
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
        f_max = max_pushing_force * np.matrix(np.ones((n, 1)))
        delta_f_max = f_max - f_n
        # Also incorporate constraint on the expected increase in the
        # contact force for each contact. This limits the rate of
        # increase in pushing force.
        delta_f_max = np.minimum(delta_f_max, max_pushing_force_increase * np.matrix(np.ones((n, 1))))

        # Set delta_f_min. The change to the normal components of
        # the contact forces must be greater than these
        # values. delta_f_min limits the magnitude of the force
        # with which the robot can pull on the environment at each
        # of its contact locations.
        #
        # f_min is size n x 1
        #
        f_min = -max_pulling_force * np.matrix(np.ones((n, 1)))
        delta_f_min = f_min - f_n
        # Also incorporate constraint on the expected change of
        # the contact force for each contact
        delta_f_min = np.maximum(delta_f_min, -max_pushing_force_decrease * np.matrix(np.ones((n, 1))))

        # # Setting negative values of delta_f_min to large negative
        # # numbers so that adhesive forces are not a binding constraint
        delta_f_min[np.where(delta_f_min <= 0)] = -10000

        # If a force has exceeded the maximum use special constraints.
        over_max = f_n > max_pushing_force
        if over_max.any():
            # at least one of the contact forces is over the maximum allowed
            delta_f_max[over_max] = -min_decrease_when_over_max_force
            delta_f_min[over_max] = -max_decrease_when_over_max_force

        return delta_f_min, delta_f_max

    # Publishes a list of Deltas for the desired joint positions.
    # @param ctrl_data List of desired joint position deltas to be published
    def publishDeltaControlValues(self, ctrl_data):
        msg = hrl_msgs.msg.FloatArrayBare()
        msg.data = ctrl_data
        self.delta_q_des_pub.publish(msg)

    # Publish a desired joint position list directly (rather than deltas)
    # @param ctrl_data List of desired joint positions to be published (not deltas).
    def publishDesiredJointAngles(self, ctrl_data):
        msg = hrl_msgs.msg.FloatArrayBare()
        msg.data = ctrl_data
        self.q_des_pub.publish(msg)

    # Main control function. Builds the control data structure and passes it to the control calculation routine
    def updateController(self):
        # Check for haptic state msg timeout
        # with self.state_lock:
        #     time_since_last_msg = rospy.Time.now() - self.last_msg_time  # will always exist as we block on startup until a haptic state msg is heard.

        # Timeout based on time since the last heard message.
        # TODO: Replace this with a freshness parameter on the skin data. More flexible than a timeout.
        # if time_since_last_msg.to_sec() > self.timeout:
        #     if self.waiting_to_resume == False:
        #         rospy.logwarn("MPC hasn't heard a haptic state messages for %s s. Stopping control effort." % str(time_since_last_msg.to_sec()))
        #         self.waiting_to_resume = True
        #         #return from updateController without actually doing anything
        #         return
        # else:
        #     if self.waiting_to_resume == True:
        #         self.waiting_to_resume = False
        #         rospy.logwarn("MPC resumed control - haptic state messages received again.")

        # Listen to a state topic from the monitor node and respond to errors.
        with self.monitor_lock:
            mpc_state = copy.copy(self.mpc_state)
            mpc_error = copy.copy(self.mpc_error)

        if mpc_error is not None and len(mpc_error) > 0:
            if not self.waiting_for_no_errors:
                rospy.logwarn("Haptic MPC: MPC monitor detected error conditions. Stopping control effort.\nState:\n%s\nErrors:\n%s", mpc_state, mpc_error)
                self.waiting_for_no_errors = True
            return
        else:
            if self.waiting_for_no_errors:
                self.waiting_for_no_errors = False
                rospy.logwarn("Haptic MPC: MPC resumed control - errors cleared.")

        # If good, run controller.
        mpc_data = self.initMPCData()
        desired_joint_pos = self.deltaQpJepGen(mpc_data)  # deltas
        if desired_joint_pos is None:
            if self.verbose:
                rospy.loginfo("Failed to calculate joint angle updates. Re-sending current position.")
            desired_joint_pos = [0.0] * len(self.joint_angles)

        if not self.mpc_enabled:
            if self.verbose:
                rospy.loginfo("Haptic MPC Disabled.  Not sending joint commands")
            return
        self.publishDeltaControlValues(desired_joint_pos)

    # Initialise the ROS communications - init node, subscribe to the robot state and goal pos, publish JEPs
    def initComms(self, node_name):
        rospy.init_node(node_name)
        self.robot_state_sub = rospy.Subscriber("haptic_mpc/robot_state", haptic_msgs.RobotHapticState, self.robotStateCallback)
        self.goal_pose_sub = rospy.Subscriber("haptic_mpc/traj_pose", geometry_msgs.msg.PoseStamped, self.goalPoseCallback)
        self.goal_posture_sub = rospy.Subscriber("haptic_mpc/goal_posture", hrl_msgs.msg.FloatArray, self.goalPostureCallback)

        self.delta_q_des_pub = rospy.Publisher("haptic_mpc/delta_q_des", hrl_msgs.msg.FloatArrayBare, queue_size=QUEUE_SIZE)
        self.q_des_pub = rospy.Publisher("haptic_mpc/q_des", hrl_msgs.msg.FloatArrayBare, queue_size=QUEUE_SIZE)

        self.mpc_monitor_sub = rospy.Subscriber("haptic_mpc/mpc_state", haptic_msgs.HapticMpcState, self.mpcMonitorCallback)

        self.mpc_weights_sub = rospy.Subscriber("haptic_mpc/_set_weights_internal", haptic_msgs.HapticMpcWeights, self.updateWeightsCallback)
        self.mpc_weights_pub = rospy.Publisher("haptic_mpc/current_weights", haptic_msgs.HapticMpcWeights, queue_size=QUEUE_SIZE, latch=True)
        self.mpc_deadzone_pub = rospy.Publisher("haptic_mpc/in_deadzone", std_msgs.msg.Bool, queue_size=QUEUE_SIZE, latch=True)

        self.enable_mpc_srv = rospy.Service("haptic_mpc/enable_mpc", haptic_srvs.EnableHapticMPC, self.enableHapticMPC)

    # Enable Haptic MPC service handler (default is enabled).
    def enableHapticMPC(self, req):
        if req.new_state == "enabled":
            rospy.loginfo("[%s]: Enabling MPC, resetting joint angles to current position", rospy.get_name())
            with self.goal_lock:
                self.goal_pos = None
                self.goal_orient_quat = None
            with self.posture_lock:
                self.goal_posture = None
            self.publishDesiredJointAngles(self.getJointAngles())
            rospy.sleep(0.1)  # Help make sure the joint angle reset gets applied ...
            rospy.loginfo("[%s] Goal reset to current position. Enabling Controller.", rospy.get_name())
            self.mpc_enabled = True
        else:
            self.mpc_enabled = False

        return haptic_srvs.EnableHapticMPCResponse(self.mpc_state)

    # Handler for Ctrl-C signals. Some of the ROS spin loops don't respond well to Ctrl-C without this.
    def signal_handler(self, signal, frame):
        print 'Ctrl+C pressed - exiting'
        sys.exit(0)

    # Start the control loop once the controller is initialised.
    def start(self):
        signal.signal(signal.SIGINT, self.signal_handler)  # Catch Ctrl-Cs

        self.initRobot()
        self.initComms("haptic_mpc")
        rospy.sleep(1.0)  # Delay to allow the ROS node to initialise properly.
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
#       lasttime = rospy.Time.now()
        while not rospy.is_shutdown():
            self.updateController()
            # rospy.spin() # For debug - run once
            r.sleep()
#        now = rospy.Time.now()
#        print "Rate: ", 1.0/((now - lasttime).to_sec())
#        lasttime = now


def main():
    import optparse
    p = optparse.OptionParser()
    haptic_mpc_util.initialiseOptParser(p)
    opt = haptic_mpc_util.getValidInput(p)

    mpc_controller = HapticMPC(opt, "haptic_mpc")
    mpc_controller.start()
