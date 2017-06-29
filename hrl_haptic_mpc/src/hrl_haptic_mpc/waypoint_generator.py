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
#
# @author Jeff Hawke
# @version 0.1
# @copyright Apache 2.0

import sys
import numpy
import copy
import threading
import collections

import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.transforms as tr

import trajectory_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import hrl_msgs.msg
import moveit_msgs.msg

from hrl_haptic_mpc import haptic_mpc_util

QUEUE_SIZE = 10

# @class WaypointGenerator Node which takes either a goal pose or a trajectory and passes
# local goals to the MPC controller to try to make the controller follow it.


class WaypointGenerator(object):
    node_name = None  # ROS node name
    # Options - should be specified by the optparser when run from the console.
    opt = None
    rate = 25  # 25.0 # Publish rate. By default, 25Hz
    msg_seq = 0  # Sequence counter

    # ROS Parameter paths - robot dependent.
    param_path = ""
    base_path = rospy.get_namespace() + "haptic_mpc"
    control_path = '/control_params'

    # ROS Topics.
    current_pose_topic = "haptic_mpc/robot_state"
    goal_pose_topic = "haptic_mpc/goal_pose"
    goal_pose_array_topic = "haptic_mpc/goal_pose_array"
    waypoint_pose_topic = "haptic_mpc/traj_pose"
    posture_threshold_topic = "haptic_mpc/at_posture_threshold"

    # Member variables
    robot = None  # Robot object
    robot_name = None  # Robot name
    sensor = None  # Sensor object
    eq_gen_type = None
    default_eq_gen_type = 'mpc_qs_1'
    joint_names = None

    skin_topic_list = None
    scl = None  # skin client
    epcon = None

    # Trajectory pose parameters.
    max_pos_step = rospy.get_param('~waypoint_step_dist', 0.10)  # 1cm steps at largest #default=0.05
    max_ang_step = numpy.radians(rospy.get_param('~waypoint_step_angle', 15))  # Slightly over 1 degree #default=1
    # tolerance for being at the goal and moving to the next waypoint.default
    # = 0.02
    at_waypoint_threshold = rospy.get_param('~waypoint_dist_thresh', 0.03)
    at_waypoint_ang_threshold = numpy.radians(rospy.get_param('~waypoint_angle_thresh', 3.0))  # default=5.0 deg

    max_posture_step = numpy.radians(rospy.get_param('~waypoint_step_posture', 5.0))
    # This controls the posture waypoint following.
    at_posture_threshold = numpy.radians(rospy.get_param('~waypoint_posture_thresh', 10.0))

    # To properly set weight, it need to be None in the begining
    # Mode will be set in generateWaypoint function
    mode = None  # "pose"  # Set to "posture" or "pose"

    # canonical time system
    tau = 0.0
    tau_d = 0.0
    tau_alpha = 10.0

    current_trajectory_duration = 0.0  # trajectory duration
    current_trajectory_deque = collections.deque()
    gripper_pose = None
    goal_pose = None
    current_gripper_waypoint = None
    base_frame_id_ = None

    ps_traj_msg = geometry_msgs.msg.PoseArray()
    jt_traj_msg = trajectory_msgs.msg.JointTrajectory()
    jt_disp_msg = moveit_msgs.msg.DisplayTrajectory()

    traj_lock = threading.RLock()
    goal_lock = threading.RLock()
    state_lock = threading.RLock()

    # Constructor. Calls functions to initialise robot specific parameters,
    # then initialises all publishers/subscribers.
    def __init__(self, node_name, opt):
        rospy.loginfo("[%s] Initialising trajectory generator for Haptic MPC.", rospy.get_name())
        self.opt = opt
        self.node_name = node_name
        rospy.init_node(node_name)

        # Set up the relevant robot parameters
        if self.opt.robot == "cody":
            self.initCody()
        elif self.opt.robot == "pr2":
            self.initPR2()
        elif self.opt.robot == "sim3" or self.opt.robot == "sim3_nolim":
            self.initSim3()
        elif self.opt.robot == "simcody":
            self.initSimCody()
        elif self.opt.robot == "crona":
            self.initCrona()
        elif self.opt.robot == "darci_sim":
            self.initDarciSim()
        else:
            rospy.logerr("[%s] Invalid Robot type: %s", rospy.get_name(), self.opt.robot)

        orient_weight = rospy.get_param(self.base_path + self.control_path + '/orientation_weight')
        pos_weight = rospy.get_param(self.base_path + self.control_path + '/position_weight')
        posture_weight = rospy.get_param(self.base_path + self.control_path + '/posture_weight')
        self.pose_weights = (pos_weight, orient_weight, 0.0)
        self.posture_weights = (0.0, 0.0, posture_weight)

        # Set up the publishers/subscribers and their callbacks.
        self.initComms()
        self.setControllerWeights(*self.pose_weights)

    def initCody(self):
        rospy.loginfo("[%s] Trajectory generator for: Cody.", rospy.get_name())
        # TODO:
        # sys.exit()

    # Initialise Darci Simulation kinematics. NB: Only used for joint limits,
    # will eventually be removed once these are passed with the robot state.
    def initDarciSim(self):
        from pykdl_utils.kdl_kinematics import create_kdl_kin

        rospy.loginfo("[%s] Trajectory generator for: Darci Simulator.", rospy.get_name())
        if not self.opt.arm:
            rospy.logerr("[%s] Need to specify --arm ('r' or 'l').\nExiting...", rospy.get_name())
            sys.exit()

        if self.opt.arm == 'l':
            self.robot_kinematics = create_kdl_kin(
                'torso_lift_link', 'end_effector_LEFT')
        else:
            self.robot_kinematics = create_kdl_kin(
                'torso_lift_link', 'end_effector_RIGHT')

        self.tf_listener = tf.TransformListener()

        if self.opt.arm is None:
            rospy.logerr("[%s] Need to specify --arm ('r' or 'l').\nExiting...", rospy.get_name())
            sys.exit()

    # Initialise PR2 kinematics. NB: Only used for joint limits, will
    # eventually be removed once these are passed with the robot state.
    def initPR2(self):
        from pykdl_utils.kdl_kinematics import create_kdl_kin
        robot_path = '/pr2'

        rospy.loginfo("[%s] Trajectory generator for: PR2.", rospy.get_name())

        self.torso_frame = rospy.get_param(self.base_path +
                                           robot_path +
                                           '/torso_frame')
        self.end_effector_frame = rospy.get_param(self.base_path +
                                                  robot_path +
                                                  '/end_effector_frame')
        self.robot_kinematics = create_kdl_kin(self.torso_frame,
                                               self.end_effector_frame,
                                               description_param="/robot_description")
        self.tf_listener = tf.TransformListener()

        if self.opt.arm is None:
            rospy.logerr("[%s] Need to specify --arm ('r' or 'l').\nExiting...", rospy.get_name())
            sys.exit()

    # Initialise 3DOF Sim kinematics. NB: This doesn't actually do anything
    # and is added mostly for structural consistency.
    def initSim3(self):
        rospy.loginfo("[%s] Trajectory generator for: Simulation 3DOF.", rospy.get_name())
        # Nothing to initialise for this.

    def initCrona(self):
        from pykdl_utils.kdl_kinematics import create_kdl_kin

        rospy.loginfo("[%s] Trajectory generator for: cRoNA.", rospy.get_name())
        if self.opt.arm is None:
            rospy.logerr("[%s] Need to specify --arm ('r' or 'l').\nExiting...", rospy.get_name())
            sys.exit()

        # self.robot_kinematics = create_kdl_kin('torso_chest_link', self.opt.arm+'_hand_link')
        self.robot_kinematics = create_kdl_kin(
            'base_link', self.opt.arm + '_hand_link')  # testing
        self.tf_listener = tf.TransformListener()

    # Initialise all publishers/subscribers used by the waypoint generator.
    def initComms(self):
        # Publish to a waypoint pose topic
        self.pose_waypoint_pub = rospy.Publisher(self.waypoint_pose_topic,
                                                 geometry_msgs.msg.PoseStamped,
                                                 queue_size=QUEUE_SIZE)
        self.goal_posture_pub = rospy.Publisher("haptic_mpc/goal_posture",
                                                hrl_msgs.msg.FloatArray,
                                                queue_size=QUEUE_SIZE)
        self.mpc_weights_pub = rospy.Publisher("haptic_mpc/_set_weights_internal",
                                               haptic_msgs.HapticMpcWeights,
                                               queue_size=QUEUE_SIZE)
        self.traj_tau_pub = rospy.Publisher("haptic_mpc/tau",
                                            std_msgs.msg.Float64,
                                            queue_size=QUEUE_SIZE)
        self.pose_traj_viz_pub = rospy.Publisher('haptic_mpc/current_pose_traj',
                                                 geometry_msgs.msg.PoseArray,
                                                 queue_size=QUEUE_SIZE,
                                                 latch=True)
        self.joint_traj_viz_pub = rospy.Publisher("haptic_mpc/current_joint_traj",
                                                  moveit_msgs.msg.DisplayTrajectory,
                                                  queue_size=QUEUE_SIZE)

        # Subscribe to the a goal pose topic.
        rospy.Subscriber(self.goal_pose_topic, geometry_msgs.msg.PoseStamped, self.goalPoseCallback)
        # Subscribe to the current robot state
        rospy.Subscriber(self.current_pose_topic, haptic_msgs.RobotHapticState, self.robotStateCallback)
        # Subscribe to the goal pose array topic.
        rospy.Subscriber(self.goal_pose_array_topic, geometry_msgs.msg.PoseArray, self.poseTrajectoryCallback)
        # Planner
        rospy.Subscriber('haptic_mpc/joint_trajectory', trajectory_msgs.msg.JointTrajectory, self.jointTrajectoryCallback)
        # Subscribe to the posture threshold topic.
        rospy.Subscriber(self.posture_threshold_topic, std_msgs.msg.Float64, self.postureThresholdCallback)
        # Subscribe to the controller weights
        rospy.Subscriber("haptic_mpc/weights", haptic_msgs.HapticMpcWeights, self.updateWeightsCallback)

    # Update goal pose.
    # @param msg A geometry_msgs.msg.PoseStamped object.
    def goalPoseCallback(self, msg):
        # If no frame specified, clear current pose goal
        if not msg.header.frame_id:
            with self.goal_lock:
                self.goal_pose = None
            rospy.logwarn("[%s] Received Pose Goal with Empty FrameID -- Clearing Pose Goal", rospy.get_name())
            return
        rospy.loginfo("[%s] Got new goal pose in %s frame.", rospy.get_name(), msg.header.frame_id)
        if not ('torso_lift_link' in msg.header.frame_id or
                'torso_chest_link' in msg.header.frame_id or
                'torso_link' in msg.header.frame_id):
            try:
                self.tf_listener.waitForTransform(
                    msg.header.frame_id, '/torso_lift_link', msg.header.stamp, rospy.Duration(5.0))
            except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException) as e:
                rospy.logerr(
                    '[%s]: TF Exception: %s', rospy.get_name(), e)
            ps = geometry_msgs.msg.PoseStamped(msg.header, msg.pose)
            ps.header.stamp = rospy.Time(0)
            new_pose_stamped = self.tf_listener.transformPose(
                '/torso_lift_link', ps)
            msg = new_pose_stamped

        with self.goal_lock:
            self.goal_pose = msg.pose

        with self.traj_lock:
            self.current_trajectory_deque.clear()

    # Store the current pose from the haptic state publisher
    # @param msg RobotHapticState messge object
    def robotStateCallback(self, msg):
        with self.state_lock:
            self.gripper_pose = msg.hand_pose
            self.joint_angles = msg.joint_angles
            self.joint_names = msg.joint_names

    # Store a trajectory of poses in the deque. Converts it to the 'torso_frame' if required.
    # @param msg A geometry_msgs.msg.PoseArray object
    def poseTrajectoryCallback(self, msg):
        rospy.loginfo("[%s] Got new pose trajectory in %s frame.", rospy.get_name(), msg.header.frame_id)
        with self.goal_lock:
            self.goal_pose = None
        with self.traj_lock:
            self.current_trajectory_msg = msg
            self.current_trajectory_deque.clear()
            # if we have an empty array, clear the deque and do nothing else.
            if len(msg.poses) == 0:
                rospy.logwarn(
                    "[%s] Received empty pose array. Clearing trajectory buffer.", rospy.get_name())
                return

            self.base_frame_id_ = msg.header.frame_id

            # Check if pose array is in torso_lift_link.  If not, transform.
            if 'torso_lift_link' not in msg.header.frame_id:
                try:
                    self.tf_listener.waitForTransform(msg.header.frame_id, '/torso_lift_link',
                                                      msg.header.stamp, rospy.Duration(5.0))
                except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException) as e:
                    rospy.logerr(
                        '[%s]: TF Exception: %s', rospy.get_name(), e)

            pose_array = []
            for pose in msg.poses:
                ps = geometry_msgs.msg.PoseStamped(msg.header, pose)
                ps.header.stamp = rospy.Time(0)
                new_pose_stamped = self.tf_listener.transformPose(
                    '/torso_lift_link', ps)
                pose_array.append(new_pose_stamped.pose)

            # visualisation message
            # pose_array_msg = geometry_msgs.msg.PoseArray()
            # pose_array_msg.header.frame_id = "/torso_lift_link"
            # pose_array_msg.poses = pose_array
            # self.pose_traj_viz_pub.publish(pose_array_msg)
            # Append the list of poses to the recently cleared deque
            self.current_trajectory_deque.extend(pose_array)

            with self.goal_lock:  # Invalidate previous goal poses
                self.goal_pose = None

    # Store a joint angle trajectory in the deque. Performs forward kinematics to convert it to end effector poses in the torso frame.
    # @param msg A trajectory_msgs.msg.JointTrajectory object.
    def jointTrajectoryCallback(self, msg):
        rospy.loginfo("[%s] Got new joint trajectory.", rospy.get_name())
        with self.traj_lock:
            self.current_trajectory_msg = msg
            self.current_trajectory_deque.clear()
            # if we have an empty array, clear the deque and do nothing else.
            if len(msg.points) == 0:
                rospy.logwarn(
                    "[%s] Received empty joint array. Clearing trajectory buffer.", rospy.get_name())
                return

            for point in msg.points:
                # Calculate pose for this point using FK
                # Append pose to deque
                #        joint_angles = point.positions
                #        end_effector_position, end_effector_orient_cart = self.robot_kinematics.FK(joint_angles, len(joint_angles))
                #        end_effector_orient_quat = tr.matrix_to_quaternion(end_effector_orient_cart)
                #
                #        pose = geometry_msgs.msg.Pose()
                #        ee_pos = end_effector_position.A1
                #        pose.position.x = ee_pos[0]
                #        pose.position.y = ee_pos[1]
                #        pose.position.z = ee_pos[2]
                #        pose.orientation.x = end_effector_orient_quat[0]
                #        pose.orientation.y = end_effector_orient_quat[1]
                #        pose.orientation.z = end_effector_orient_quat[2]
                #        pose.orientation.w = end_effector_orient_quat[3]
                # Check type of current value when pulling from the deque
                self.current_trajectory_deque.append(point)

                self.tau = 0.0
                self.tau_d = 0.0

        # Clear goal pose so the waypoints come from the trajectory
        with self.goal_lock:
            self.goal_pose = None

    # Update posture threshold to adjust posture trajectory control speed
    def postureThresholdCallback(self, msg):
        self.at_posture_threshold = msg.data

    # Update default weights (values when active).
    def updateWeightsCallback(self, msg):
        self.pose_weights = (msg.position_weight, msg.orient_weight, 0.0)
        self.posture_weights = (0.0, 0.0, msg.posture_weight)
        self.setControllerWeights(msg.position_weight, msg.orient_weight, msg.posture_weight)

    # Update the weights used by the MPC.
    def setControllerWeights(self, position_weight, orient_weight, posture_weight):
        print "Waypoint Generator setting weights: %s, %s, %s" % (position_weight, orient_weight, posture_weight)
        weights_msg = haptic_msgs.HapticMpcWeights()
        weights_msg.header.stamp = rospy.Time.now()
        weights_msg.position_weight = position_weight
        weights_msg.orient_weight = orient_weight
        weights_msg.posture_weight = posture_weight
        # Enable position tracking only - disable orientation by setting the
        # weight to 0
        self.mpc_weights_pub.publish(weights_msg)

    # Returns the next waypoint along a straight line trajectory from the current gripper pose to the goal pose.
    # The step size towards the goal is configurable through the parameters passed in.
    # @param current_pose geometry_msgs.msg.Pose
    # @param goal_pose geometry_msgs.msg.Pose
    # @param max_pos_step = scalar float for position step size to take (metres)
    # @param max_ang_step = scalar float for orientation step size to take (radians)
    # @return A geometry_msgs.msg.Pose to send to the MPC
    def straightLineTrajectory(self, current_pose, goal_pose, max_pos_step, max_ang_step):
        # Create the Pose for the desired waypoint
        desired_pose = geometry_msgs.msg.Pose()

        current_pos_vector = numpy.array(
            [current_pose.position.x, current_pose.position.y, current_pose.position.z])
        goal_pos_vector = numpy.array(
            [goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])

        position_waypoint = self.getPositionStep(
            current_pos_vector, goal_pos_vector, max_pos_step)

        desired_pose.position = geometry_msgs.msg.Point(*position_waypoint)

        # Calculate the new orientation. Use slerp - spherical interpolation on
        # quaternions
        current_orientation = [current_pose.orientation.x, current_pose.orientation.y,
                               current_pose.orientation.z, current_pose.orientation.w]
        goal_orientation = [goal_pose.orientation.x, goal_pose.orientation.y,
                            goal_pose.orientation.z, goal_pose.orientation.w]
        orientation_waypoint = self.getOrientationStep(current_orientation, goal_orientation, max_ang_step)
#        orientation_waypoint = goal_orientation

        desired_pose.orientation = geometry_msgs.msg.Quaternion(*orientation_waypoint)

        # Return completed pose data structure
        return desired_pose

    # Returns a linearly interpolated step towards the goal pos from the current pos
    # @param current_pos Current position as a numpy array (assumed to be [x,y,z])
    # @param goal_pos Goal position as a numpy array (assumed to be [x,y,z])
    # @param max_pos_step A scalar max step size for position (in metres).
    # @return An interpolated step in position towards the goal.
    def getPositionStep(self, current_pos, goal_pos, max_pos_step):
        difference_to_goal = goal_pos - current_pos
        dist_to_goal = numpy.linalg.norm(difference_to_goal)
        if dist_to_goal > max_pos_step:
            # Generate a linear step towards the goal position.
            step_vector = difference_to_goal / dist_to_goal * max_pos_step
        else:
            # The distance remaining to the goal is less than the step size
            # used.
            step_vector = difference_to_goal
        desired_position = current_pos + step_vector
        return desired_position

    # Returns a linearly interpolated step towards the goal orientation from the current orientation (using SLERP)
    # @param q_h_orient Current hand orientation as a quaternion in numpy array form (assumed to be [x,y,z,w])
    # @param q_g_orient Current goal orientation as a quaternion in numpy array form (assumed to be [x,y,z,w])
    # @param max_ang_step A scalar max step size for orientation (in radians).
    # @return An interpolated step in orientation towards the goal.
    def getOrientationStep(self, q_h_orient, q_g_orient, max_ang_step):
        ang = ut.quat_angle(q_h_orient, q_g_orient)
        step_fraction = 1 if abs(ang) <= max_ang_step else max_ang_step / abs(ang)
        interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_fraction)
        return interp_q_goal

    # Returns a message header (std_msgs object), populated with the time,
    # frame_id and sequence number
    def getMessageHeader(self):
        header = std_msgs.msg.Header()
        header.seq = self.msg_seq
        self.msg_seq += 1
        header.stamp = rospy.get_rostime()
        header.frame_id = "/torso_lift_link"
        # header.frame_id = "/base_link"
        return header

    # Max absolute distance between joints postures.
    def distanceBetweenPostures(self, postureA, postureB):
        max_dist = 0.0
        for i in range(1, len(postureA)):
            dist = abs(postureA[i] - postureB[i])
            if dist > max_dist:
                max_dist = dist
        return max_dist

    # Euclidian distance between two poses. Ignores differences in orientation.
    # @param poseA geometry_msgs.msg.Pose
    # @param poseB geometry_msgs.msg.Pose
    # @return The euclidian distance between the two pose positions.
    def distanceBetweenPoses(self, poseA, poseB):
        xdiff = poseA.position.x - poseB.position.x
        ydiff = poseA.position.y - poseB.position.y
        zdiff = poseA.position.z - poseB.position.z
        return numpy.sqrt(xdiff**2 + ydiff ** 2 + zdiff**2)

    # Return the angle between two quaternions (axis-angle representation)
    # @param poseA geometry_msgs.msg.Pose
    # @param poseB geometry_msgs.msg.Pose
    # @return The angular distance between the two pose orientations.
    def angularDistanceBetweenPoses(self, poseA, poseB):
        quatA = [poseA.orientation.x, poseA.orientation.y,
                 poseA.orientation.z, poseA.orientation.w]
        quatB = [poseB.orientation.x, poseB.orientation.y,
                 poseB.orientation.z, poseB.orientation.w]
        ang_diff = ut.quat_angle(quatA, quatB)
        return ang_diff

    # Try to get a waypoint pose from the trajectory deque.
    #
    # Also trims the trajectory to reduce the number of waypoint.
    # Scans through the trajectory to find the next pose that is at least max_pos_step away (or the last pose in the deque).
    # @return A Pose object if there is a currently stored trajectory, otherwise None.
    def getWaypointFromTrajectory(self):
        with self.state_lock:
            curr_gripper_pose = copy.copy(self.gripper_pose)
        with self.traj_lock:
            # If we have a current trajectory, represented as a sequence of
            # Pose objects in a deque
            if len(self.current_trajectory_deque) > 0:
                # Check if we need to trim the list
                if len(self.current_trajectory_deque) > 1:
                    # Return the next point closest along the trajectory if
                    # we're close enough to it (eg within 5mm of it)
                    if self.distanceBetweenPoses(curr_gripper_pose, self.current_trajectory_deque[0]) < self.at_waypoint_threshold:  # \
                        #            and self.angularDistanceBetweenPoses(curr_gripper_pose, self.current_trajectory_deque[0]) < self.at_waypoint_ang_threshold:
                        # Trim the trajectory so that the current waypoint is a reasonable distance away - too fine steps make the controller unhappy.
                        # Discard trajectory points that are within the min distance unless this the last point in the trajectory.
                        # Adjust this by increasing or decreasing max_pos_step
                        while self.distanceBetweenPoses(curr_gripper_pose, self.current_trajectory_deque[0]) < self.max_pos_step and len(self.current_trajectory_deque) > 1:
                            print "Trimming trajectory - dist: %s, len(deque): %s" % (self.distanceBetweenPoses(self.gripper_pose, self.current_trajectory_deque[0]), len(self.current_trajectory_deque))
                            self.current_trajectory_deque.popleft()

                # self.straightLineTrajectory(curr_gripper_pose, self.current_trajectory_deque[0], self.max_pos_step, self.max_ang_step)
                desired_pose = self.current_trajectory_deque[0]
                return desired_pose

            else:  # We haven't got a valid trajectory. Return None.
                return None

    # Pull postures from the trajectory deque
    def getPostureFromTrajectory(self):
        with self.state_lock:
            curr_posture = copy.copy(self.joint_angles)
        with self.traj_lock:

            # Check if we need to trim the list
            if len(self.current_trajectory_deque) > 1:
                max_length = len(self.current_trajectory_deque)
                min_idx = -1

                for i in xrange(len(self.current_trajectory_deque)):
                    dist = self.distanceBetweenPostures(curr_posture, self.current_trajectory_deque[max_length-1-i].positions)
                    if dist < self.at_posture_threshold:
                        min_idx = max_length-1-i
                        break

                for i in xrange(min_idx+1):
                    if len(self.current_trajectory_deque) >= 2:
                        self.current_trajectory_deque.popleft()

            desired_posture = self.current_trajectory_deque[0]  # self.straightLineTrajectory(curr_ee_pose, self.current_trajectory_deque[0], self.max_pos_step, self.max_ang_step)
            return desired_posture

            # current_trajectory_size = len(self.current_trajectory_deque)
            # if current_trajectory_size > 0:

            #     if self.tau == 0.0:
            #         self.current_trajectory_duration = 1.0 / \
            #             float(self.rate) * float(current_trajectory_size)

            #     traj_index = int(self.tau * current_trajectory_size)
            #     # print "tau: ", self.tau, self.tau_d, "traj_index: ",
            #     # traj_index, current_trajectory_size, "duration: ",
            #     # self.current_trajectory_duration
            #     if traj_index >= current_trajectory_size:
            #         return self.current_trajectory_deque[-1]

            #     desired_posture = self.current_trajectory_deque[traj_index]

            #     dist = self.distanceBetweenPostures(
            #         curr_posture, desired_posture.positions)
            #     self.tau_d = 1.0 / \
            #         float(self.rate) * 1.0 / (self.current_trajectory_duration) / \
            #         (1.0 + self.tau_alpha * dist**2)
            #     self.tau += self.tau_d

            #     # if traj_index < current_trajectory_size:
            #     # if dist < self.at_posture_threshold:

            #     #     traj_index = (int)(self.tau * current_trajectory_size)
            #     #     desired_posture = self.current_trajectory_deque[traj_index]

            #     # self.tau_d = 1.0/(1.0/(float)self.rate*(float)current_trajectory_size) / (1.0 + tau_alpha*dist**2)
            #     #     self.tau += tau_d
            #     return desired_posture

            # else:  # We haven't got a valid trajectory. Return None.
            #     self.tau = 0.0
            #     self.tau_d = 0.0
            #     print "Reset trajectory variables!"
            #     return None

# If we have a current trajectory, represented as a sequence of Pose OR JointTrajectoryPoint objects in a deque
# if len(self.current_trajectory_deque) > 0:
# Check if we need to trim the list
# if len(self.current_trajectory_deque) > 1:
# Return the next point closest along the trajectory if we're close enough to it (eg within 5mm of it)
# if self.distanceBetweenPostures(curr_posture, self.current_trajectory_deque[0].positions) < self.at_posture_threshold:# \
# Trim the trajectory so that the current waypoint is a reasonable distance away - too fine steps make the controller unhappy.
# Discard trajectory points that are within the min distance unless this the last point in the trajectory.
# Adjust this by increasing or decreasing max_pos_step
# while self.distanceBetweenPostures(curr_posture, self.current_trajectory_deque[0].positions) < self.at_posture_threshold and len(self.current_trajectory_deque) > 1:
# print "Trimming trajectory - dist: %s, len(deque): %s" %  (self.distanceBetweenPostures(curr_posture, self.current_trajectory_deque[0].positions), len(self.current_trajectory_deque))
# self.current_trajectory_deque.popleft()
# print "Moving to next waypoint."
# print numpy.degrees(self.current_trajectory_deque[0].positions)

# desired_posture = self.current_trajectory_deque[0]#self.straightLineTrajectory(curr_gripper_pose, self.current_trajectory_deque[0], self.max_pos_step, self.max_ang_step)
# return desired_posture

# else: # We haven't got a valid trajectory. Return None.
# return None

    # Publishes the next waypoint for the control to try to achieve.
    #
    # If we have a valid trajectory, get the next waypoint from that.
    # If not and we have a valid end goal pose instead, move in a straight line towards it.
    # If we have neither, don't publish anything.
    def generateWaypoint(self):
        # TODO: Work out which way to generate the next waypoint. Currently always do a straight line
        # update waypoint if we've achieved this waypoint, else keep current waypoint.
        with self.state_lock:
            tmp_curr_pose = copy.copy(self.gripper_pose)
        with self.goal_lock:
            tmp_goal_pose = copy.copy(self.goal_pose)

        # If we haven't heard robot state yet, don't act.
        if tmp_curr_pose is None:
            return

        # Logic flow changed so that the deque can store either
        # geometry_msgs.msg.Pose or trajectory_msgs.msg.JointTrajectoryPoint
        if tmp_goal_pose is not None:
            if self.mode != "pose":
                self.setControllerWeights(*self.pose_weights)
            self.mode = "pose"
            desired_pose = self.straightLineTrajectory(
                tmp_curr_pose, tmp_goal_pose, self.max_pos_step, self.max_ang_step)

            # pose viz
#            self.viz_traj(desired_pose)

            waypoint_msg = geometry_msgs.msg.PoseStamped()
            waypoint_msg.header = self.getMessageHeader()
            waypoint_msg.pose = desired_pose
            self.pose_waypoint_pub.publish(waypoint_msg)

        # We have some valid trajectory.
        elif len(self.current_trajectory_deque) > 0:
            # If the deque has poses
            if type(self.current_trajectory_deque[0]) == geometry_msgs.msg.Pose:
                if self.mode != "pose":
                    self.setControllerWeights(*self.pose_weights)
                self.mode = "pose"
                # If we have a trajectory, return a valid pose (else None)
                desired_pose = self.getWaypointFromTrajectory()
                # If we have a teleop goal but don't have a trajectory, go in a
                # straight line to the goal
                if desired_pose is None and tmp_goal_pose is not None:
                    desired_pose = self.straightLineTrajectory(
                        tmp_curr_pose, tmp_goal_pose, self.max_pos_step, self.max_ang_step)

                # Don't publish invalid waypoints. If we still didn't get a
                # good pose from the straight line interpolation, something is
                # wrong.
                if desired_pose is None:
                    return

                # pose viz
#                self.viz_traj(desired_pose)

                # Publish a waypoint every cycle.
                waypoint_msg = geometry_msgs.msg.PoseStamped()
                waypoint_msg.header = self.getMessageHeader()
                waypoint_msg.pose = desired_pose
                self.pose_waypoint_pub.publish(waypoint_msg)
            # If the deque has joint postures rather than poses.
            elif type(self.current_trajectory_deque[0]) == trajectory_msgs.msg.JointTrajectoryPoint:
                if self.mode != "posture":
                    self.setControllerWeights(*self.posture_weights)
                self.mode = "posture"
                # If we have a trajectory, return a valid posture
                desired_posture = self.getPostureFromTrajectory()
                if desired_posture is None:
                    rospy.loginfo("[%s] No desired_posture set.", rospy.get_name())
                    return

#                self.viz_traj(desired_posture)

                waypoint_msg = hrl_msgs.msg.FloatArray()
                # print desired_posture.positions
                waypoint_msg.data = list(desired_posture.positions)

                # publish time factor
                msg = std_msgs.msg.Float64()
                msg.data = self.tau
                self.traj_tau_pub.publish(msg)

                self.goal_posture_pub.publish(waypoint_msg)
            else:
                rospy.logwarn(
                    "[%s] Object in the waypoint deque is neither geometry_msgs.msg.Pose or trajectory_msgs.msg.JointTrajectoryPoint. Who broke it?!", rospy.get_name())

    # Start the waypoint generator publishing waypoints.
    def start(self):
        rate = rospy.Rate(self.rate)  # 25Hz, nominally.
        rospy.loginfo("[%s] Beginning publishing waypoints.", rospy.get_name())
        while not rospy.is_shutdown():
            self.generateWaypoint()
            # print rospy.Time()
            rate.sleep()

#    def viz_traj(self, desired_traj):
#   TODO: Not updated to Hydro because not immediately needed, and DisplayTrajectory msg
#       moved from arm_navigation_msgs to moveit_msgs, and the fields are non-trivially changed
#       between the two. --Phil, 12AUG2015
#
#        # init joint msg
#        self.jt_traj_msg.header.frame_id = "world"
#        self.jt_traj_msg.header.stamp = rospy.Time.now()
#        self.jt_traj_msg.joint_names = self.joint_names
#
#        # init posearray msg
#        if self.base_frame_id_ is None:
#            self.base_frame_id_ = 'torso_lift_link'
#        self.ps_traj_msg.header.frame_id = self.base_frame_id_
#        self.ps_traj_msg.header.stamp = rospy.Time.now()
#
#        if type(desired_traj) == trajectory_msgs.msg.JointTrajectoryPoint:
#
#                # joint
#            self.jt_traj_msg.points = [desired_traj]
#
#            # pose
#            desired_pose = geometry_msgs.msg.Pose()
#            desired_pose.position.x = 0.0
#            desired_pose.position.y = 0.0
#            desired_pose.position.z = 0.0
#            self.ps_traj_msg.poses = [desired_pose]
#
#        elif type(desired_traj) == geometry_msgs.msg.Pose:
#
#            # joint
#            # [0.0] * len(self.joint_names)
#            desired_posture = trajectory_msgs.msg.JointTrajectoryPoint()
#            desired_posture.positions = [0.0] * len(self.joint_names)
#            self.jt_traj_msg.points = [desired_posture]
#
#            # pose traj
#            self.ps_traj_msg.poses = [desired_traj]
#
#        else:
#
#            # joint
#            # [0.0] * len(self.joint_names)
#            desired_posture = trajectory_msgs.msg.JointTrajectoryPoint()
#            desired_posture.positions = [0.0] * len(self.joint_names)
#            self.jt_traj_msg.points = [desired_posture]
#
#            # pose
#            desired_pose = geometry_msgs.msg.Pose()
#            desired_pose.position.x = 0.0
#            desired_pose.position.y = 0.0
#            desired_pose.position.z = 0.0
#            self.ps_traj_msg.poses = [desired_pose]
#
#        with self.state_lock:
#            if self.joint_angles is None:
#                return
#            self.jt_disp_msg.robot_state.joint_state = self.joint_angles
#
#        self.jt_disp_msg.trajectory.joint_trajectory = self.jt_traj_msg
#
#        self.pose_traj_viz_pub.publish(self.ps_traj_msg)
#        self.joint_traj_viz_pub.publish(self.jt_disp_msg)


def main():
    # Set up input arg parser
    import optparse
    p = optparse.OptionParser()

    haptic_mpc_util.initialiseOptParser(p)
    opt = haptic_mpc_util.getValidInput(p)

    # Create and start the trajectory manager module.
    # loads all parameter sets on init
    traj_mgr = WaypointGenerator('mpc_traj_gen', opt)
    traj_mgr.start()
