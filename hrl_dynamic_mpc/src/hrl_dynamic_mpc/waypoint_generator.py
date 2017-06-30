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

## @package hrl_dynamic_mpc
#
# @author Daehyung Park
# @version 0.1
# @copyright Apache 2.0

import sys, os, time
import numpy, math
import copy, threading
import collections

import roslib; roslib.load_manifest('hrl_dynamic_mpc')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('hrl_haptic_mpc')
import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_lib.circular_buffer as cb

import trajectory_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import hrl_msgs.msg
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
import arm_navigation_msgs.msg
from sensor_msgs.msg import JointState

from hrl_haptic_mpc import haptic_mpc_util
import hrl_common_code_darpa_m3.visualization.draw_scene as ds

## @class WaypointGenerator Node which takes either a goal pose or a trajectory and passes
# local goals to the MPC controller to try to make the controller follow it.
class WaypointGenerator():
  node_name = None # ROS node name
  opt = None  # Options - should be specified by the optparser when run from the console.
  rate = 100 #25 #25.0 # Publish rate. By default, 25Hz
  msg_seq = 0 # Sequence counter

  # ROS Parameter paths - robot dependent.
  param_path = ""
  base_path = "haptic_mpc"
  pr2_param_path = "/pr2"
  sim_param_path = "/sim3"
  darci_sim_param_path = "/darci_sim"

  # ROS Topics. TODO: Move these to param server to allow easy changes across all modules.
  current_pose_topic = "haptic_mpc/robot_state" # For PR2, Sim3 only
  goal_pose_topic    = "haptic_mpc/goal_pose"

  # Member variables
  robot       = None # Robot object
  joint_names = None

  # Trajectory pose parameters.
  # Max Step size set very large.  Passes out received goal pose unchainged.
  # (except trajectories)
  max_pos_step = 5.0 # 50mm steps at largest #default=0.05
  #max_pos_step = 0.0 # changed
  max_ang_step = numpy.radians(361)# Slightly over 1 degree #default=1
  at_waypoint_threshold = 0.02 # tolerance for being at the goal and moving to the next waypoint.default = 0.02
  ## at_waypoint_ang_threshold = numpy.radians(5.0) #default=5.0 deg

  ## max_posture_step = numpy.radians(5.0)
  ## at_posture_threshold = numpy.radians(2.0) ## This controls the posture waypoint following. for sim
  at_posture_threshold = numpy.radians(2.0) ## This controls the posture waypoint following. for darci

  mode = "none" # Set to "posture", "pose", or potentially "both"

  # canonical time system
  tau   = 0.0
  tau_d = 0.0
  tau_alpha = 10.0

  current_trajectory_duration = 0.0 # trajectory duration
  current_trajectory_deque = collections.deque()
  ee_pose   = None
  goal_pose = None
  current_ee_waypoint = None
  joint_states = None
  base_frame_id_ = None

  ## self.n_jts = 7 #temp
  ## self.ee_pos_buf = cb.CircularBuffer(self.hist_size, (3,))
  ## self.q_buf = cb.CircularBuffer(self.hist_size, (self.n_jts,))

  ps_traj_msg = geometry_msgs.msg.PoseArray()
  jt_traj_msg = trajectory_msgs.msg.JointTrajectory()
  jt_disp_msg = arm_navigation_msgs.msg.DisplayTrajectory()

  traj_lock  = threading.RLock()
  goal_lock  = threading.RLock()
  state_lock = threading.RLock()
  jstate_lock = threading.RLock() ## Haptic state lock

  ## Constructor. Calls functions to initialise robot specific parameters, then initialises all publishers/subscribers.
  def __init__(self, node_name, opt):
    rospy.loginfo("Initialising trajectory generator for Haptic MPC")
    self.opt = opt
    self.node_name = node_name
    rospy.init_node(node_name)

    # Set up the relevant robot parameters
    if (self.opt.robot == "pr2"):
      self.initPR2()
    elif(self.opt.robot == "sim3" or self.opt.robot == "sim3_nolim"):
      self.initSim3()
    elif(self.opt.robot == "darci"):
      self.initDarci()
    elif(self.opt.robot == "darci_sim"):
      self.initDarciSim()
    else:
      rospy.logerr("Invalid Robot type: %s" % robot)

    # Set up the publishers/subscribers and their callbacks.
    self.initComms()

    ## # Temporal offset
    ## self.at_waypoint_threshold = rospy.get_param('haptic_mpc/controller/offset')
    ## print "waypoint threshold: ", self.at_waypoint_threshold

    return

  ## Initialise Darci Simulation kinematics. NB: Only used for joint limits, will eventually be removed once these are passed with the robot state.
  def initDarciSim(self):
    from pykdl_utils.kdl_kinematics import create_kdl_kin

    # set param
    at_waypoint_threshold = 0.01 # tolerance for being at the goal and moving to the next waypoint.default = 0.02
    at_posture_threshold = numpy.radians(1.0) ## This controls the posture waypoint following. for darci

    rospy.loginfo("Trajectory generator for: Darci Simulator")
    if not self.opt.arm:
      rospy.logerr('Arm not specified for Darci Simulator')
      sys.exit()

    if self.opt.arm == 'l':
      self.robot_kinematics = create_kdl_kin('torso_lift_link', 'end_effector_LEFT')
    else:
      self.robot_kinematics = create_kdl_kin('torso_lift_link', 'end_effector_RIGHT')

    self.tf_listener = tf.TransformListener()

    ## import darci_sim_client
    ## self.robot_client = darci_sim_client.DarciSimClient()

    if self.opt.arm == None:
        rospy.logerr('Need to specify --arm.\nExiting...')
        sys.exit()

  ## Initialise Darci kinematics. NB: Only used for joint limits, will eventually be removed once these are passed with the robot state.
  def initDarci(self):
    from pykdl_utils.kdl_kinematics import create_kdl_kin

    # set param
    at_waypoint_threshold = 0.1 # tolerance for being at the goal and moving to the next waypoint.default = 0.02
    at_posture_threshold = numpy.radians(2.0) ## This controls the posture waypoint following. for darci

    rospy.loginfo("Trajectory generator for: Darci ")
    if not self.opt.arm:
      rospy.logerr('Arm not specified for Darci ')
      sys.exit()

    if self.opt.arm == 'l':
      self.robot_kinematics = create_kdl_kin('torso_lift_link', 'end_effector_LEFT')
    else:
      self.robot_kinematics = create_kdl_kin('torso_lift_link', 'end_effector_RIGHT')
    #if self.opt.arm == 'l':
    #  self.robot_kinematics = create_kdl_kin('zlift_link', 'end_effector_LEFT')
    #else:
    #  self.robot_kinematics = create_kdl_kin('zlift_link', 'end_effector_RIGHT')

    self.tf_listener = tf.TransformListener()

    ## import darci_sim_client
    ## self.robot_client = darci_sim_client.DarciSimClient()

    if self.opt.arm == None:
        rospy.logerr('Need to specify --arm.\nExiting...')
        sys.exit()

  ## Initialise PR2 kinematics. NB: Only used for joint limits, will eventually be removed once these are passed with the robot state.
  def initPR2(self):
    from pykdl_utils.kdl_kinematics import create_kdl_kin

    rospy.loginfo("Trajectory generator for: PR2")
    if not self.opt.arm:
      rospy.logerr('Arm not specified for PR2')
      sys.exit()

    self.robot_kinematics = create_kdl_kin('torso_lift_link', self.opt.arm+'_gripper_tool_frame')
    self.tf_listener = tf.TransformListener()

    if self.opt.arm == None:
        rospy.logerr('Need to specify --arm.\nExiting...')
        sys.exit()

  ## Initialise 3DOF Sim kinematics. NB: This doesn't actually do anything and is added mostly for structural consistency.
  def initSim3(self):
    rospy.loginfo("Trajectory generator for: Simulation 3DOF")
    # Nothing to initialise for this.

  ## Initialise all publishers/subscribers used by the waypoint generator.
  def initComms(self):
    # Publish to a waypoint pose topic
    self.goal_pose_pub      = rospy.Publisher("haptic_mpc/goal_pose", geometry_msgs.msg.PoseStamped) # Pose waypoint publishing
    self.goal_posture_pub   = rospy.Publisher("haptic_mpc/goal_posture", hrl_msgs.msg.FloatArrayBare)
    self.mpc_weights_pub    = rospy.Publisher("haptic_mpc/goal_weight", haptic_msgs.HapticMpcWeights)
    self.traj_tau_pub       = rospy.Publisher("haptic_mpc/tau",std_msgs.msg.Float64)
    self.tracking_error_pub = rospy.Publisher("haptic_mpc/track_err", std_msgs.msg.Float64)

    self.ps_cmd_viz_pub = rospy.Publisher("haptic_mpc/ps_cmd_viz", geometry_msgs.msg.PoseArray)
    self.jt_cmd_viz_pub = rospy.Publisher("haptic_mpc/jt_cmd_viz", arm_navigation_msgs.msg.DisplayTrajectory)

    # Subscribe a trajectory from planner
    rospy.Subscriber('haptic_mpc/joint_trajectory', trajectory_msgs.msg.JointTrajectory, self.jointTrajectoryCallback)
    rospy.Subscriber('haptic_mpc/pose_trajectory', geometry_msgs.msg.PoseArray, self.poseTrajectoryCallback)

    # Subscribe info
    rospy.Subscriber("/joint_states", JointState, self.jointStatesCallback)

    if (self.opt.robot == "pr2" or self.opt.robot == "sim3" or self.opt.robot == "sim3_nolim"):
      # Subscribe to the current robot state
      rospy.Subscriber(self.current_pose_topic, haptic_msgs.RobotHapticState, self.robotStateCallback)
    elif(self.opt.robot == "darci_sim"):
      self.joint_state_sub = rospy.Subscriber('/'+self.opt.arm+'_arm_controller/state', JointTrajectoryControllerState, self.jointControllerStateCallback)
    elif(self.opt.robot == "darci"):
    ##   self.joint_state_sub = rospy.Subscriber('/'+self.opt.arm+'_arm_controller/state', JointTrajectoryControllerState, self.jointControllerStateCallback)
      print "darci"
    else:
      rospy.logerr("Invalid Robot type: %s" % robot)

    # visualization
    self.draw_ee_path   = ds.SceneDraw("haptic_mpc/viz/ee_path", "/torso_lift_link")


    ##
    # Callback for /joint_states topic. Updates current joint
    # angles and efforts for the arms constantly
    # @param data JointState message recieved from the /joint_states topic
  def jointStatesCallback(self, msg):
    with self.jstate_lock:
        self.joint_states  = msg

        if self.opt.robot == "darci":
          self.joint_angles = copy.copy(self.joint_states.position[11:18]) #temp
          self.joint_names  = ['left_arm_j0', 'left_arm_j1', 'left_arm_j2', 'left_arm_j3', 'left_arm_j4', 'left_arm_j5', 'left_arm_j6'] #temp
          end_effector_position, end_effector_orient_quat = self.robot_kinematics.FK(self.joint_angles)

          pose = geometry_msgs.msg.Pose()
          ee_pos = end_effector_position.A1
          pose.position.x = ee_pos[0]
          pose.position.y = ee_pos[1]
          pose.position.z = ee_pos[2]
          ## pose.orientation.x = end_effector_orient_quat[0]
          ## pose.orientation.y = end_effector_orient_quat[1]
          ## pose.orientation.z = end_effector_orient_quat[2]
          ## pose.orientation.w = end_effector_orient_quat[3]
          self.ee_pose = pose #end_effector_position # correct type?
          ## print self.ee_pose



  ## Update goal pose.
  ## Store the current pose from the haptic state publisher
  # @param msg RobotHapticState messge object
  def robotStateCallback(self, msg):
    with self.state_lock:
      self.ee_pose = msg.hand_pose
      self.joint_angles = msg.joint_angles

  def jointControllerStateCallback(self, msg):
    with self.state_lock:
      if self.joint_names is None:
          self.joint_names = copy.copy(msg.joint_names)
      self.joint_angles = copy.copy(msg.actual.positions)
      ## self.joint_velocities = copy.copy(msg.actual.velocities)
      ## self.time = msg.header.stamp.secs + msg.header.stamp.nsecs*(1e-9)
      end_effector_position, end_effector_orient_quat = self.robot_kinematics.FK(self.joint_angles)

      pose = geometry_msgs.msg.Pose()
      ee_pos = end_effector_position.A1
      pose.position.x = ee_pos[0]
      pose.position.y = ee_pos[1]
      pose.position.z = ee_pos[2]
      ## pose.orientation.x = end_effector_orient_quat[0]
      ## pose.orientation.y = end_effector_orient_quat[1]
      ## pose.orientation.z = end_effector_orient_quat[2]
      ## pose.orientation.w = end_effector_orient_quat[3]
      self.ee_pose = pose #end_effector_position # correct type?


  ## Store a trajectory of poses in the deque. Converts it to the 'torso_frame' if required.
  # @param msg A geometry_msgs.msg.PoseArray object
  def poseTrajectoryCallback(self, msg):
      rospy.loginfo("Waypoint Gen: Got new pose trajectory")
      self.goal_pose = None
      with self.traj_lock:
          self.current_trajectory_msg = msg
          self.current_trajectory_deque.clear()
          # if we have an empty array, clear the deque and do nothing else.
          if len(msg.poses) == 0:
              rospy.logwarn("Received empty pose array. Clearing trajectory buffer")
              return

          self.base_frame_id_ = msg.header.frame_id

          #Check if pose array is in torso_lift_link.  If not, transform.
          if not 'torso_lift_link' in msg.header.frame_id:
              print "msg.header.frame_id"
              print msg.header.frame_id
              try:
                  self.tf_listener.waitForTransform(msg.header.frame_id, '/torso_lift_link',
                                            msg.header.stamp, rospy.Duration(5.0))
              except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException) as e:
                  rospy.logerr('[arm_trajectory_generator]: TF Exception: %s' %e)

          pose_array = []
          for pose in msg.poses:
              ps = geometry_msgs.msg.PoseStamped(msg.header, pose)
              ps.header.stamp = rospy.Time(0)
              new_pose_stamped = self.tf_listener.transformPose('/torso_lift_link', ps)
              ## pose_array.append(new_pose_stamped.pose)
              self.current_trajectory_deque.append(new_pose_stamped.pose)

          with self.goal_lock: # Invalidate previous goal poses.
              self.goal_pose = None


  ## Store a joint angle trajectory in the deque. Performs forward kinematics to convert it to end effector poses in the torso frame.
  # @param msg A trajectory_msgs.msg.JointTrajectory object.
  def jointTrajectoryCallback(self, msg):
    with self.traj_lock:
      self.current_trajectory_msg = msg
      self.current_trajectory_deque.clear()
      # if we have an empty array, clear the deque and do nothing else.
      if len(msg.points) == 0:
        rospy.logwarn("Received empty joint array. Clearing trajectory buffer")
        return

      rospy.loginfo("Got new non-empty joint trajectory")

      for point in msg.points:
        self.current_trajectory_deque.append(point) # Check type of current value when pulling from the deque

        self.tau   = 0.0
        self.tau_d = 0.0

    # Clear goal pose so the waypoints come from the trajectory
    with self.goal_lock:
      self.goal_pose = None


  ## Update the weights used by the MPC.
  def setControllerWeights(self, position_weight, orient_weight, posture_weight):
    weights_msg = haptic_msgs.HapticMpcWeights()
    weights_msg.header.stamp = rospy.Time.now()
    weights_msg.position_weight = position_weight
    weights_msg.orient_weight = orient_weight
    weights_msg.posture_weight = posture_weight
    self.mpc_weights_pub.publish(weights_msg) # Enable position tracking only - disable orientation by setting the weight to 0


  ## Returns the next waypoint along a straight line trajectory from the current ee pose to the goal pose.
  # The step size towards the goal is configurable through the parameters passed in.
  # @param current_pose geometry_msgs.msg.Pose
  # @param goal_pose geometry_msgs.msg.Pose
  # @param max_pos_step = scalar float for position step size to take (metres)
  # @param max_ang_step = scalar float for orientation step size to take (radians)
  # @return A geometry_msgs.msg.Pose to send to the MPC
  def straightLineTrajectory(self, current_pose, goal_pose, max_pos_step, max_ang_step):
    desired_pose = geometry_msgs.msg.Pose() # Create the Pose for the desired waypoint

    current_pos_vector = numpy.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    goal_pos_vector = numpy.array([goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])

    position_waypoint = self.getPositionStep(current_pos_vector, goal_pos_vector, max_pos_step)

    desired_pose.position.x = position_waypoint[0]
    desired_pose.position.y = position_waypoint[1]
    desired_pose.position.z = position_waypoint[2]

    # Calculate the new orientation. Use slerp - spherical interpolation on quaternions
    current_orientation = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
    goal_orientation = [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w]

    orientation_waypoint = goal_orientation#self.getOrientationStep(current_orientation, goal_orientation, max_ang_step)

    desired_pose.orientation.x = orientation_waypoint[0]
    desired_pose.orientation.y = orientation_waypoint[1]
    desired_pose.orientation.z = orientation_waypoint[2]
    desired_pose.orientation.w = orientation_waypoint[3]

    # Return completed pose data structure
    return desired_pose

  ## Returns a linearly interpolated step towards the goal pos from the current pos
  # @param current_pos Current position as a numpy array (assumed to be [x,y,z])
  # @param goal_pos Goal position as a numpy array (assumed to be [x,y,z])
  # @param max_pos_step A scalar max step size for position (in metres).
  # @return An interpolated step in position towards the goal.
  def getPositionStep(self, current_pos, goal_pos, max_pos_step):
    difference_to_goal = goal_pos - current_pos
    dist_to_goal = numpy.sqrt(numpy.vdot(difference_to_goal, difference_to_goal))
    if dist_to_goal > max_pos_step:
      step_vector = difference_to_goal / dist_to_goal * max_pos_step # Generate a linear step towards the goal position.
    else:
      step_vector = difference_to_goal # The distance remaining to the goal is less than the step size used.
    desired_position = current_pos + step_vector
    return desired_position

  ## Returns a linearly interpolated step towards the goal orientation from the current orientation (using SLERP)
  # @param q_h_orient Current hand orientation as a quaternion in numpy array form (assumed to be [x,y,z,w])
  # @param q_g_orient Current goal orientation as a quaternion in numpy array form (assumed to be [x,y,z,w])
  # @param max_ang_step A scalar max step size for orientation (in radians).
  # @return An interpolated step in orientation towards the goal.
  def getOrientationStep(self, q_h_orient, q_g_orient, max_ang_step):
    ang = ut.quat_angle(q_h_orient, q_g_orient)
    ang_mag = abs(ang)
    step_fraction = 0.001
    if step_fraction * ang_mag > max_ang_step:
      # this is pretty much always true, can clean up the code.
      step_fraction = max_ang_step / ang_mag
    interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_fraction)
    return interp_q_goal

  ## Returns a message header (std_msgs object), populated with the time, frame_id and sequence number
  def getMessageHeader(self):
    header = std_msgs.msg.Header()
    header.seq = self.msg_seq
    self.msg_seq += 1
    header.stamp = rospy.get_rostime()
    #header.frame_id = "/torso_lift_link"
    header.frame_id = "/base_link"
    return header

  ## Max absolute distance between joints postures.
  def distanceBetweenPostures(self, postureA, postureB, num = 0):
    max_dist = 0.0
    if num == 0 : num = len(postureA)
    for i in range(1, num):
      dist = abs(postureA[i] - postureB[i])
      if dist > max_dist:
        max_dist = dist
    return max_dist

  ## Euclidian distance between two poses. Ignores differences in orientation.
  # @param poseA geometry_msgs.msg.Pose
  # @param poseB geometry_msgs.msg.Pose
  # @return The euclidian distance between the two pose positions.
  def distanceBetweenPoses(self, poseA, poseB):
    xdiff = poseA.position.x - poseB.position.x
    ydiff = poseA.position.y - poseB.position.y
    zdiff = poseA.position.z - poseB.position.z
    return numpy.sqrt(xdiff**2 + ydiff **2 + zdiff**2)

  ## Return the angle between two quaternions (axis-angle representation)
  # @param poseA geometry_msgs.msg.Pose
  # @param poseB geometry_msgs.msg.Pose
  # @return The angular distance between the two pose orientations.
  def angularDistanceBetweenPoses(self, poseA, poseB):
    quatA = [poseA.orientation.x, poseA.orientation.y, poseA.orientation.z, poseA.orientation.w]
    quatB = [poseB.orientation.x, poseB.orientation.y, poseB.orientation.z, poseB.orientation.w]
    ang_diff = ut.quat_angle(quatA, quatB)
    return ang_diff

  ## Pull postures from the trajectory deque
  def getPostureFromTrajectory(self):
    with self.state_lock:
      curr_posture = copy.copy(self.joint_angles)
      ## self.q_buf.append(curr_posture)

    with self.traj_lock:

      # If we have a current trajectory, represented as a sequence of Pose OR JointTrajectoryPoint objects in a deque
      if len(self.current_trajectory_deque) > 0:

        if type(self.current_trajectory_deque[0]) == trajectory_msgs.msg.JointTrajectoryPoint:

          # Check if we need to trim the list
          if len(self.current_trajectory_deque) > 1:
            # Return the next point closest along the trajectory if we're close enough to it (eg within 5mm of it)
            # Trim the trajectory so that the current waypoint is a reasonable distance away - too fine steps make the controller unhappy.
            # Discard trajectory points that are within the min distance unless this the last point in the trajectory.
            # Adjust this by increasing or decreasing max_pos_step
            ## while self.distanceBetweenPostures(curr_posture, self.current_trajectory_deque[0].positions) < self.at_posture_threshold and len(self.current_trajectory_deque) > 1:
            ##   print "Trimming trajectory - dist: %s, len(deque): %s" %  (self.distanceBetweenPostures(curr_posture, self.current_trajectory_deque[0].positions), len(self.current_trajectory_deque))

            if int(len(self.current_trajectory_deque)/5) > 2 and False:
              # find closest points
              min_dist = 1000.0
              min_idx  = 0

              # check only first 33% of trajectories
              for i in xrange(int(len(self.current_trajectory_deque)/5)):
                dist = self.distanceBetweenPostures(curr_posture, \
                                                    self.current_trajectory_deque[i].positions, 5)

                if dist < min_dist:
                  min_idx = i
                  min_dist = dist

              for i in xrange(min_idx+1):
                if len(self.current_trajectory_deque) >= 2:
                  print "Moving to next waypoint. ", len(self.current_trajectory_deque), self.distanceBetweenPostures(curr_posture, \
                                                    self.current_trajectory_deque[0].positions, 5)
                  self.current_trajectory_deque.popleft()

            else:
              if self.distanceBetweenPostures(curr_posture, self.current_trajectory_deque[0].positions, 5) < \
                self.at_posture_threshold:
                self.current_trajectory_deque.popleft()

                ## print "Moving to next waypoint. ", len(self.current_trajectory_deque)
                ## print numpy.degrees(self.current_trajectory_deque[0].positions)

          desired_posture = self.current_trajectory_deque[0]#self.straightLineTrajectory(curr_ee_pose, self.current_trajectory_deque[0], self.max_pos_step, self.max_ang_step)
          return desired_posture

        else:
          self.current_trajectory_deque.clear()
          return None


      else: # We haven't got a valid trajectory. Return None.
        return None


  ## Try to get a waypoint pose from the trajectory deque.
  #
  # Also trims the trajectory to reduce the number of waypoint.
  # Scans through the trajectory to find the next pose that is at least max_pos_step away (or the last pose in the deque).
  # @return A Pose object if there is a currently stored trajectory, otherwise None.
  def getWaypointFromTrajectory(self):
    with self.state_lock:
      curr_ee_pose = copy.copy(self.ee_pose)

    with self.traj_lock:
      # If we have a current trajectory, represented as a sequence of Pose objects in a deque
      if len(self.current_trajectory_deque) > 0:

        if type(self.current_trajectory_deque[0]) == geometry_msgs.msg.Pose:

          # Check if we need to trim the list
          if len(self.current_trajectory_deque) > 1:

            if int(len(self.current_trajectory_deque)/5) > 2 and False:

              min_dist = 1000.0
              min_idx  = 0

              # check only first 33% of trajectories
              for i in xrange(int(len(self.current_trajectory_deque)/5)):
                dist = self.distanceBetweenPoses(curr_ee_pose, self.current_trajectory_deque[i])

                if dist < min_dist or dist < self.at_waypoint_threshold:
                  min_idx = i
                  min_dist = dist

              for i in xrange(min_idx+1):
                if len(self.current_trajectory_deque) >= 2:
                  print "Moving to next waypoint. ", len(self.current_trajectory_deque), \
                    self.distanceBetweenPoses(curr_ee_pose, self.current_trajectory_deque[0])
                  self.current_trajectory_deque.popleft()
            else:
              if self.distanceBetweenPoses(curr_ee_pose, self.current_trajectory_deque[0]) < self.at_waypoint_threshold:
                self.current_trajectory_deque.popleft()

          ## if len(self.current_trajectory_deque) > 4:

          ##   while not (self.distanceBetweenPoses(curr_ee_pose, self.current_trajectory_deque[0])
          ##              < self.at_waypoint_threshold and
          ##              self.distanceBetweenPoses(curr_ee_pose, self.current_trajectory_deque[1])
          ##              >= self.at_waypoint_threshold):
          ##     if len(self.current_trajectory_deque) == 2:
          ##       return None
          ##     else:
          ##       self.current_trajectory_deque.popleft()


          ##   ## while self.distanceBetweenPoses(curr_ee_pose, self.current_trajectory_deque[0]) < self.at_waypoint_threshold or self.distanceBetweenPoses(curr_ee_pose, self.current_trajectory_deque[2]) < self.at_waypoint_threshold:
          ##   ##   ## print "Trimming trajectory - dist: %s, len(deque): %s" %  (self.distanceBetweenPoses(self.ee_pose, self.current_trajectory_deque[0]), len(self.current_trajectory_deque))
          ##   ##   self.current_trajectory_deque.popleft()
          ## elif len(self.current_trajectory_deque) > 1:
          ##   # Return the next point closest along the trajectory if we're close enough to it (eg within 5mm of it)
          ##   if self.distanceBetweenPoses(curr_ee_pose, self.current_trajectory_deque[0]) < self.at_waypoint_threshold:
          ##     self.current_trajectory_deque.popleft()


          desired_pose = self.current_trajectory_deque[0]#self.straightLineTrajectory(curr_gripper_pose, self.current_trajectory_deque[0], self.max_pos_step, self.max_ang_step)
          return desired_pose

        else:
          self.current_trajectory_deque.clear()
          return None

      else: # We haven't got a valid trajectory. Return None.
        return None



  ## Publishes the next waypoint for the control to try to achieve.
  #
  # If we have a valid trajectory, get the next waypoint from that.
  # If not and we have a valid end goal pose instead, move in a straight line towards it.
  # If we have neither, don't publish anything.
  def generateWaypoint(self):
    # TODO: Work out which way to generate the next waypoint. Currently always do a straight line
    # update waypoint if we've achieved this waypoint, else keep current waypoint.
    with self.state_lock:
      tmp_curr_pose = copy.copy(self.ee_pose)
    with self.goal_lock:
      tmp_goal_pose = copy.copy(self.goal_pose)

    if tmp_curr_pose == None: # If we haven't heard robot state yet, don't act.
      return

    # Logic flow changed so that the deque can store either geometry_msgs.msg.Pose or trajectory_msgs.msg.JointTrajectoryPoint
    if tmp_goal_pose != None:
      desired_pose = self.straightLineTrajectory(tmp_curr_pose, tmp_goal_pose, self.max_pos_step, self.max_ang_step)
      waypoint_msg = geometry_msgs.msg.PoseStamped()
      waypoint_msg.header = self.getMessageHeader()
      waypoint_msg.pose = desired_pose
      self.pose_waypoint_pub.publish(waypoint_msg)
      self.viz_traj([])

    elif len(self.current_trajectory_deque) > 0: # We have some valid trajectory.

      try:
        traj_type = type(self.current_trajectory_deque[0])
      except:
        rospy.loginfo("empty traj queue")
        return

      # If the deque has joint postures rather than poses.
      if traj_type == trajectory_msgs.msg.JointTrajectoryPoint:
        if self.mode != "posture":
          print "pose => posture"
          self.setControllerWeights(0.0, 0.0, 1.0)
          ## self.q_buf.clear()
          time.sleep(0.5)
        self.mode = "posture"

        desired_posture = self.getPostureFromTrajectory() # If we have a trajectory, return a valid posture
        if desired_posture == None:
          rospy.loginfo("No desired_posture")
          return

        waypoint_msg = hrl_msgs.msg.FloatArrayBare()
        #print desired_posture.positions
        waypoint_msg.data = list(desired_posture.positions)

        # publish time factor
        ## msg = std_msgs.msg.Float64()
        ## msg.data = self.tau
        ## self.traj_tau_pub.publish(msg)
        self.goal_posture_pub.publish(waypoint_msg)

        with self.state_lock:
          tmp_curr_posture = copy.copy(self.joint_angles)

        # publish posture error
        self.tracking_error_pub.publish(self.distanceBetweenPostures(tmp_curr_posture, desired_posture.positions, 5))

      elif traj_type == geometry_msgs.msg.Pose:
        if self.mode != "pose":
          print "posture => pose"
          self.setControllerWeights(1.0, 0.0, 0.0)
          ## self.ee_pos_buf.clear()
          time.sleep(0.5)
        self.mode = "pose"

        desired_pose = self.getWaypointFromTrajectory() # If we have a trajectory, return a valid pose (else None)
        if type(desired_pose) != geometry_msgs.msg.Pose: return

        ## if desired_pose == None and tmp_goal_pose != None: # If we have a teleop goal but don't have a trajectory, go in a straight line to the goal
        ##   desired_pose = self.straightLineTrajectory(tmp_curr_pose, tmp_goal_pose, self.max_pos_step, self.max_ang_step)

        # Don't publish invalid waypoints. If we still didn't get a good pose from the straight line interpolation, something is wrong.
        if desired_pose == None:
          rospy.loginfo("No desired_pose")
          return

        # Publish a waypoint every cycle.
        waypoint_msg = geometry_msgs.msg.PoseStamped()
        waypoint_msg.header = self.getMessageHeader()
        waypoint_msg.pose = desired_pose
        self.goal_pose_pub.publish(waypoint_msg)

        with self.state_lock:
          tmp_curr_pose = copy.copy(self.ee_pose)

        # publish pose error
        self.tracking_error_pub.publish(self.distanceBetweenPoses(tmp_curr_pose, desired_pose))

      else:
        rospy.loginfo("Object in the waypoint deque is neither geometry_msgs.msg.Pose or trajectory_msgs.msg.JointTrajectoryPoint. Who broke it?!")

      self.viz_traj(self.current_trajectory_deque)

    else:
      desired_posture = False
      self.viz_traj([])


  ## Start the waypoint generator publishing waypoints.
  def start(self):
    rate = rospy.Rate(self.rate) # 25Hz, nominally.
    rospy.loginfo("Beginning publishing waypoints")
    while not rospy.is_shutdown():
      self.generateWaypoint()
      #print rospy.Time()
      rate.sleep()


  def viz_traj(self, desired_traj):

    if desired_traj == []: return

    # init joint msg
    self.jt_traj_msg.header.frame_id = "world"
    self.jt_traj_msg.header.stamp = rospy.Time.now()
    self.jt_traj_msg.joint_names = self.joint_names

    # init posearray msg
    if self.base_frame_id_ == None: self.base_frame_id_ = 'torso_lift_link'
    self.ps_traj_msg.header.frame_id = self.base_frame_id_
    self.ps_traj_msg.header.stamp = rospy.Time.now()

    # init pose rviz linelist msg
    lPos = []

    if type(desired_traj[0]) == trajectory_msgs.msg.JointTrajectoryPoint:

        # joint
        self.jt_traj_msg.points = [desired_traj]

        # pose
        desired_pose = geometry_msgs.msg.Pose()
        desired_pose.position.x = 0.0
        desired_pose.position.y = 0.0
        desired_pose.position.z = 0.0
        self.ps_traj_msg.poses = [desired_pose]

    elif type(desired_traj[0]) == geometry_msgs.msg.Pose:

        # joint
        desired_posture = trajectory_msgs.msg.JointTrajectoryPoint() #[0.0] * len(self.joint_names)
        desired_posture.positions = [0.0] * len(self.joint_names)
        self.jt_traj_msg.points = [desired_posture]

        # pose
        self.ps_traj_msg.poses = [desired_traj]

        # visualization
        if len(desired_traj) > 1:
            for i in xrange(len(desired_traj)):
                mPos = numpy.matrix([desired_traj[i].position.x,
                                     desired_traj[i].position.y,
                                     desired_traj[i].position.z]).T
                lPos.append(mPos)

    else:

        # joint
        desired_posture = trajectory_msgs.msg.JointTrajectoryPoint() #[0.0] * len(self.joint_names)
        desired_posture.positions = [0.0] * len(self.joint_names)
        self.jt_traj_msg.points = [desired_posture]

        # pose
        desired_pose = geometry_msgs.msg.Pose()
        desired_pose.position.x = 0.0
        desired_pose.position.y = 0.0
        desired_pose.position.z = 0.0
        self.ps_traj_msg.poses = [desired_pose]

    with self.jstate_lock:
        if self.joint_states == None: return
        self.jt_disp_msg.robot_state.joint_state = self.joint_states

    self.jt_disp_msg.trajectory.joint_trajectory = self.jt_traj_msg
    self.jt_cmd_viz_pub.publish(self.jt_disp_msg)
    self.ps_cmd_viz_pub.publish(self.ps_traj_msg)

    ee_traj_color = [0.2,0.2,1.0,0.7]
    self.draw_ee_path.pub_segment(lPos, ee_traj_color, 600)



if __name__ == '__main__':
    # Set up input arg parser
    import optparse
    p = optparse.OptionParser()

    haptic_mpc_util.initialiseOptParser(p)
    opt = haptic_mpc_util.getValidInput(p)

    # Create and start the trajectory manager module.
    traj_mgr = WaypointGenerator('mpc_traj_gen', opt) # loads all parameter sets on init
    traj_mgr.start()




      ## current_trajectory_size = len(self.current_trajectory_deque)
      ## if current_trajectory_size > 0:

      ##   if self.tau == 0.0:
      ##     self.current_trajectory_duration = 1.0/float(self.rate)*float(current_trajectory_size)

      ##   traj_index = int(self.tau * current_trajectory_size)
      ##   #print "tau: ", self.tau, self.tau_d, "traj_index: ", traj_index, current_trajectory_size, "duration: ", self.current_trajectory_duration
      ##   if traj_index >= current_trajectory_size: return self.current_trajectory_deque[-1]

      ##   desired_posture = self.current_trajectory_deque[traj_index]

      ##   dist = self.distanceBetweenPostures(curr_posture, desired_posture.positions)
      ##   self.tau_d = 1.0/float(self.rate) * 1.0/(self.current_trajectory_duration) / (1.0 + self.tau_alpha*dist**2)
      ##   self.tau += self.tau_d

      ##   ## if traj_index < current_trajectory_size:
      ##   ##   if dist < self.at_posture_threshold:

      ##   ##     traj_index = (int)(self.tau * current_trajectory_size)
      ##   ##     desired_posture = self.current_trajectory_deque[traj_index]

      ##   ##     self.tau_d = 1.0/(1.0/(float)self.rate*(float)current_trajectory_size) / (1.0 + tau_alpha*dist**2)
      ##   ##     self.tau += tau_d
      ##   return desired_posture

      ## else: # We haven't got a valid trajectory. Return None.
      ##   self.tau   = 0.0
      ##   self.tau_d = 0.0
      ##   print "Reset trajectory variables!"
      ##   return None

