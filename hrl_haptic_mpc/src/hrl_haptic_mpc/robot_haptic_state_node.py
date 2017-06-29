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
# @copyright Apache Licence


import threading
import itertools as it
import sys
import signal

import rospy
import geometry_msgs.msg as geom_msgs
from tf import TransformListener
from std_msgs.msg import Header

from hrl_lib import transforms as tr
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
from hrl_haptic_mpc import multiarray_to_matrix, haptic_mpc_util, skin_client as sc

QUEUE_SIZE = 1000


class RobotHapticStateServer(object):
    """Haptic state publisher: publishes all relevant haptic state information on a common interface independent of robot type."""
    def __init__(self, opt, node_name=None):
        """Constructor for robot haptic state server."""
        self.opt = opt
        # Set up all ros comms to start with
        self.node_name = node_name
        self.tf_listener = None
        self.state_pub = None
        self.rate = 100.0  # 100 Hz.
        self.msg_seq = 0  # Sequence counter

        # ROS Param server paths.
        self.base_path = rospy.get_namespace()+"haptic_mpc"
        self.robot_path = '/'+self.opt.robot

        # Skin data
        self.skin_topic_list = []  # List of topic names
        self.skin_client = None

        # Robot object. Contains all the subscribers and robot specific
        # kinematics, etc
        self.robot = None

        # Joint data
        self.joint_names = []
        self.joint_angles = []
        self.desired_joint_angles = []
        self.joint_velocities = []
        self.joint_stiffness = []
        self.joint_damping = []
        self.joint_data_lock = threading.RLock()
        self.joint_names = []

        # End effector pose
        self.end_effector_position = None
        self.end_effector_orient_cart = None
        self.end_effector_orient_quat = None

        self.torso_pose = geom_msgs.Pose()

        # Jacobian storage
        self.Jc = None  # Contact jacobians
        self.Je = None  # End effector jacobian
        self.trim_threshold = 1.0  # this is 1.0 for forces

        # Jacobian MultiArray to Matrix converter
        self.ma_to_m = multiarray_to_matrix.MultiArrayConverter()

        # Initialise various parameters.
        self.initComms()
        self.initRobot(self.opt.robot)

    def initRobot(self, robot_type="pr2"):
        """Initialise all robot specific parameters (skin topics, robot interfaces, etc). Calls the appropriate init function."""
        if robot_type == "pr2":
            self.initPR2()
        elif robot_type == "cody":
            self.initCody()
        elif robot_type == "cody5dof":
            self.initCody(5)
        elif robot_type == "sim3" or robot_type == "sim3_nolim" or robot_type == "sim_equal_links_1":
            self.initSim(robot_type)
        elif robot_type == "crona":
            self.initCrona()
        elif robot_type == "darci":
            self.initDarci()
        elif robot_type == "darci_sim":
            self.initDarciSim()
        else:
            rospy.logerr("RobotHapticState: Invalid robot type specified")
            sys.exit()

    def initDarci(self):
        print "DARCI actual robot not implemented in Quasi-Static MPC"
        sys.exit(1)

    def initDarciSim(self):
        # Robot kinematic classes and skin clients. These are specific to each robot
        from hrl_haptic_mpc import urdf_arm_darpa_m3 as urdf_arm

        # Load parameters from ROS Param server
        self.skin_topic_list = rospy.get_param(self.base_path + self.robot_path + '/skin_list/' + self.opt.sensor)
        self.torso_frame = rospy.get_param(self.base_path + self.robot_path + '/torso_frame')
        self.inertial_frame = rospy.get_param(self.base_path + self.robot_path + '/inertial_frame')
        self.end_effector_frame = rospy.get_param(self.base_path + self.robot_path + '/end_effector_frame')
        rospy.loginfo("RobotHapticState: Initialising PR2 haptic state publisher with the following skin topics: \n%s", self.skin_topic_list)
        self.skin_client = sc.TaxelArrayClient(self.skin_topic_list, self.torso_frame, self.tf_listener)
        self.skin_client.setTrimThreshold(self.trim_threshold)
        rospy.loginfo("RobotHapticState: Initialising robot interface")
        if not self.opt.arm:
            rospy.logerr("RobotHapticState: No arm specified for Darci Sim")
            sys.exit()

        # Create the robot object. Provides interfaces to the robot arm and
        # various kinematic functions.
        self.robot = urdf_arm.URDFArm(self.opt.arm,
                                      self.tf_listener,
                                      self.torso_frame,
                                      self.end_effector_frame)

    # Initialise parameters for the state publisher when used on the PR2.
    def initPR2(self):
        # Robot kinematic classes and skin clients. These are specific to each robot
        from hrl_haptic_mpc import urdf_arm_darpa_m3 as urdf_arm

        # Load parameters from ROS Param server
        self.skin_topic_list = rospy.get_param(self.base_path + self.robot_path + '/skin_list/' + self.opt.sensor)
        self.torso_frame = rospy.get_param(self.base_path + self.robot_path + '/torso_frame')
        self.inertial_frame = rospy.get_param(self.base_path + self.robot_path + '/inertial_frame')
        self.end_effector_frame = rospy.get_param(self.base_path + self.robot_path + '/end_effector_frame')
        rospy.loginfo("RobotHapticState: Initialising PR2 haptic state publisher with the following skin topics: \n%s", self.skin_topic_list)
        self.skin_client = sc.TaxelArrayClient(self.skin_topic_list, self.torso_frame, self.tf_listener)
        self.skin_client.setTrimThreshold(self.trim_threshold)
        rospy.loginfo("RobotHapticState: Initialising robot interface")
        if not self.opt.arm:
            rospy.logerr("RobotHapticState: No arm specified for PR2")
            sys.exit()

        # Create the robot object. Provides interfaces to the robot arm and
        # various kinematic functions.
        self.robot = urdf_arm.URDFArm(self.opt.arm,
                                      self.tf_listener,
                                      self.torso_frame,
                                      self.end_effector_frame)

    # Initialise parameters for the state publisher when used on Cody.
    def initCody(self, num_of_joints=7):
        import hrl_cody_arms.cody_arm_client

        # Load the skin list from the param server
        self.skin_topic_list = rospy.get_param(self.base_path + self.robot_path + '/skin_list/' + self.opt.sensor)
        self.torso_frame = rospy.get_param(self.base_path + self.robot_path + '/torso_frame')
        self.inertial_frame = rospy.get_param(self.base_path + self.robot_path + '/inertial_frame')
        rospy.loginfo("RobotHapticState: Initialising Cody haptic state publisher with the following skin topics: \n%s", self.skin_topic_list)
        self.skin_client = sc.TaxelArrayClient(self.skin_topic_list, self.torso_frame, self.tf_listener)
        self.skin_client.setTrimThreshold(self.trim_threshold)
        rospy.loginfo("RobotHapticState: Initialising robot interface")
        if not self.opt.arm:
            rospy.logerr("RobotHapticState: No arm specified for Cody")
            sys.exit()

        if num_of_joints == 7:
            self.robot = hrl_cody_arms.cody_arm_client.CodyArmClient_7DOF(self.opt.arm)
        elif num_of_joints == 5:
            self.robot = hrl_cody_arms.cody_arm_client.CodyArmClient_5DOF(self.opt.arm)
        else:
            rospy.logerr("RobotHapticState: Incorrect number of joints specified for Cody.  Must be 5 or 7")
            sys.exit()

    # Initialise parameters for the state publisher when used in simulation
    # with the 3DOF arm.
    def initSim(self, robot_type='sim3'):
        from hrl_haptic_mpc import gen_sim_arms as sim_robot

        # Load the skin list from the param server
        if robot_type == 'sim3':
            import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as sim_robot_config
        elif robot_type == 'sim3_nolim':
            import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule_nolim as sim_robot_config
        elif robot_type == 'sim_equal_links_1':
            import hrl_common_code_darpa_m3.robot_config.multi_link_one_planar as sim_robot_config

        self.skin_topic_list = rospy.get_param(self.base_path + self.robot_path + '/skin_list/' + self.opt.sensor)
        self.torso_frame = rospy.get_param(self.base_path + self.robot_path + '/torso_frame')
        self.inertial_frame = rospy.get_param(self.base_path + self.robot_path + '/inertial_frame')
        rospy.loginfo("RobotHapticState: Initialising Sim haptic state publisher" +
                      "with the following skin topics: \n%s"
                      % str(self.skin_topic_list))

        self.skin_client = sc.TaxelArrayClient(self.skin_topic_list, self.torso_frame, self.tf_listener)
        self.skin_client.setTrimThreshold(self.trim_threshold)

        # TODO: Add config switching here.
        rospy.loginfo("RobotHapticState: Initialising Sim robot interface")
        sim_config = sim_robot_config
        self.robot = sim_robot.ODESimArm(sim_config)

    # Initialise parameters for the state publisher when used in simulation
    # with the 7DOF cody arm.
    def initSimCody(self):
        import cody_arm_darpa_m3 as cody_arm

        # Load the skin list from the param server
        self.skin_topic_list = rospy.get_param(self.base_path + self.robot_path + '/skin_list')
        self.torso_frame = rospy.get_param(self.base_path + self.robot_path + '/torso_frame')
        self.inertial_frame = rospy.get_param(self.base_path + self.robot_path + '/inertial_frame')
        rospy.loginfo("RobotHapticState: Initialising Sim haptic state publisher" +
                      "with the following skin topics: \n%s"
                      % str(self.skin_topic_list))
        self.skin_client = sc.TaxelArrayClient(self.skin_topic_list, self.torso_frame, self.tf_listener)
        self.skin_client.setTrimThreshold(self.trim_threshold)

        # TODO: Add config switching here.
        rospy.loginfo("RobotHapticState: Initialising robot interface")
        if not self.opt.arm:
            rospy.logerr("RobotHapticState: No arm specified for Sim Cody")
            sys.exit()

        self.robot = cody_arm.CodyArmClient(self.opt.arm)

    def initCrona(self):
        from hrl_haptic_mpc import urdf_arm_darpa_m3 as urdf_arm

        # self.skin_topic_list = rospy.get_param(self.base_path + '/skin_list/' + self.opt.sensor)
        self.skin_topic_list = None
        self.torso_frame = rospy.get_param(self.base_path + self.robot_path + '/torso_frame')
        self.end_effector_frame = rospy.get_param(self.base_path + self.robot_path + '/end_effector_frame')
        self.inertial_frame = rospy.get_param(self.base_path + self.robot_path + '/inertial_frame')

        self.skin_client = sc.TaxelArrayClient([], self.torso_frame, self.tf_listener)
        rospy.loginfo("RobotHapticState: Initialising CRONA haptic state publisher with the following skin topics: \n%s", self.skin_topic_list)

        rospy.loginfo("RobotHapticState: Initialising robot interface")
        if not self.opt.arm:
            rospy.logerr("RobotHapticState: No arm specified for CRONA")
            sys.exit()
        self.robot = urdf_arm.URDFArm(self.opt.arm,
                                      self.tf_listener,
                                      base_link=self.torso_frame,
                                      end_link=self.end_effector_frame)
        self.skins = []
        self.Jc = []

    # Initialise publishers for the robot haptic state,
    # the current gripper pose, and a TF listener.
    # NB: The skin client and robot clients will have their own
    # publishers/subscribers specific to them.
    def initComms(self):
        if self.node_name is not None:
            rospy.init_node(self.node_name)
        self.tf_listener = TransformListener()
        self.state_pub = rospy.Publisher('haptic_mpc/robot_state', haptic_msgs.RobotHapticState, queue_size=QUEUE_SIZE)
        self.gripper_pose_pub = rospy.Publisher('haptic_mpc/gripper_pose', geom_msgs.PoseStamped, queue_size=QUEUE_SIZE)

    # Returns a header type with the current timestamp.
    # Does not set the frame_id
    def getMessageHeader(self):
        header = Header()
        header.stamp = rospy.get_rostime()
        return header

    # Updates the stored end effector Jacobian from the current joint angles
    # and end effector position
    def updateEndEffectorJacobian(self):
        # self.Je = [self.robot.kinematics.jacobian(self.joint_angles)]
        self.Je = [self.robot.kinematics.jacobian(self.joint_angles, self.end_effector_position)]
        # pos = self.robot.kinematics.forward(self.joint_angles,end_link=self.opt.arm+'_forearm_roll_link')
        # self.Je = [self.robot.kinematics.jacobian(self.joint_angles, pos[:3,3])]

    # Compute contact Jacobians based on the provided taxel array dictionary
    # @param skin_data Dictionary containing taxel array messages indexed by topic name
    def updateContactJacobians(self, skin_data):
        # loc_l = list of taxel locations relative the "torso_lift_link" frame.
        # jt_l = list of joints beyond which the jacobian columns are zero.
        # loc_l. jt_l from skin client.
        Jc_l = []
        loc_l, jt_l = self.getTaxelLocationAndJointList(skin_data)

        if len(loc_l) != len(jt_l):
            rospy.logfatal("Haptic State Publisher: Dimensions don't match. %s, %s", len(loc_l), len(jt_l))
            sys.exit()

        for jt_li, loc_li in it.izip(jt_l, loc_l):
            Jc = self.robot.kinematics.jacobian(self.joint_angles, loc_li)
            Jc[:, jt_li + 1:] = 0.0
            # trim the jacobian to suit the number of DOFs.
            Jc = Jc[0:3, 0:len(self.joint_stiffness)]
            Jc_l.append(Jc)
        self.Jc = Jc_l

    # Returns a Pose object for the torso pose in the stated inertial frame
    def updateTorsoPose(self):
        # Get the transformation from the desired frame to current frame
        self.tf_listener.waitForTransform(self.inertial_frame, self.torso_frame, rospy.Time(), rospy.Duration(10.0))
        t1, q1 = self.tf_listener.lookupTransform(self.inertial_frame, self.torso_frame, rospy.Time(0))
        torso_pose = geom_msgs.Pose()
        torso_pose.position = geom_msgs.Point(*t1)
        torso_pose.orientation = geom_msgs.Quaternion(*q1)
        return torso_pose

    # Store latest joint states from the specified robot class
    # @var joint_names: Joint names
    # @var joint_angles: Joint angles
    # @var joint_velocities: Joint velocities
    # @var joint_stiffness: Joint stiffness
    # @var joint_damping: Joint damping
    # @var q_des: Desired joint angles
    def updateJointStates(self):
        self.joint_names = self.robot.get_joint_names()
        self.joint_angles = self.robot.get_joint_angles()
        self.joint_stiffness, self.joint_damping = self.robot.get_joint_impedance()
        self.joint_velocities = self.robot.get_joint_velocities()
        q_des = self.robot.get_ep()
        if q_des is not None:
            self.desired_joint_angles = q_des

    # Compute and store the end effector position, orientation, and jacobian
    # from the current joint angles.
    def updateEndEffectorPose(self):
        pos, rot = self.robot.kinematics.FK(self.joint_angles)
        self.end_effector_position = pos
        self.end_effector_orient_cart = rot
        quat = tr.matrix_to_quaternion(rot)
        # Quaternions +/-q are equivalent rotations, may flip arbitrarily upon conversion from rotation matrix
        # Normalize the sign (arbitrarily) to remove sign flipping which causes erratic behavior due the the T_quat below
        if self.end_effector_orient_quat is None:
            self.end_effector_orient_quat = quat
        if tr.np.dot(quat, self.end_effector_orient_quat) < 0:
            quat = [-q_i for q_i in quat]
        self.end_effector_orient_quat = quat

    # Returns a list of taxel locations and a list of joint numbers after which the
    # joint torque will have no effect on the contact force
    # @param skin_data Dictionary of TaxelArrays indexed by topic
    # @retval locations List of taxel locations where a force is present
    # @retval joint_nums List of joints after which the joint torque will have no effect on the contact force
    # @return These arrays will both be the same length (as the joint number corresponds
    def getTaxelLocationAndJointList(self, skin_data):
        locations = []
        joint_nums = []

        for ta_msg in skin_data.values():
            # Get points list
            ta_locs = self.skin_client.getContactLocationsFromTaxelArray(ta_msg)
            # Create list of joints beyond which the joint torque has no effect
            # on contact force
            ta_jts = []
            for contact_index in range(len(ta_msg.centers_x)):
                # Index of last joint, 0 indexed.
                jt_num = len(self.joint_angles) - 1

                # Verify we have the same number of link names as taxel
                # contacts If not, make no assumptions about joints.
                if len(ta_msg.link_names) >= len(ta_msg.centers_x):
                    link_name = ta_msg.link_names[contact_index]

                    # Iterate over the known joint names looking for the link this is associated with.
                    # NB: links should be named based on their joint.
                    # NB:
                    for idx, joint_name in enumerate(self.joint_names):
                        if joint_name in link_name:
                            jt_num = idx
                            break

                ta_jts.append(jt_num)

            # Attach these lists to the end of the global list (incorporating
            # multiple taxel arrays)
            locations.extend(ta_locs)
            joint_nums.extend(ta_jts)

        return locations, joint_nums

    # Modify taxel data for PR2 specific situations
    # TODO: Survy to implement selective taxel ignoring.
    # @param skin_data Dictionary containing taxel array messages indexed by topic name
    def modifyPR2Taxels(self, skin_data):
        # print "modifyPR2Taxels"
        return skin_data

    # Modifies data from the taxel array based on robot specific configurations.
    # An example of this is ignoring the PR2 wrist taxels when the wrist
    # is near its joint limit as the wrist itself will trigger the taxel.
    # @param skin_data Dict containing taxel array messages indexed by topic name
    # @return skin_data Modified dictionary containing taxel array messages
    def modifyRobotSpecificTaxels(self, skin_data):
        if self.opt.robot == 'pr2':
            return self.modifyPR2Taxels(skin_data)
        # If this is running on a different robot, don't modify the data.
        return skin_data

    # Calls all the sub-component updates
    def updateHapticState(self):
        self.updateJointStates()
        # self.torso_pose = self.updateTorsoPose()
        self.updateEndEffectorPose()
        self.updateEndEffectorJacobian()
        # Skin sensor calculations.
        # Get the latest skin data from the skin client
        skin_data = self.skin_client.getTrimmedSkinData()
        full_skin_data = self.skin_client.getSkinData()
        # Trim skin_data based on specific robot state (eg wrist
        # configuration).
        skin_data = self.modifyRobotSpecificTaxels(skin_data)
        self.updateContactJacobians(skin_data)
        # Add the list of  TaxelArray messages to the message
        self.skins = skin_data.values()
        self.full_skins = full_skin_data.values()

    # Build the haptic state message data structure
    # @return haptic_state_msg Haptic State message object containing relevant data
    def getHapticStateMessage(self):
        # Update all haptic state data (jacobians etc)
        self.updateHapticState()

        msg = haptic_msgs.RobotHapticState()

        msg.header = self.getMessageHeader()
        msg.header.frame_id = self.torso_frame

        # TODO Locking on data? - check these are copies.
        # Joint states
#    self.updateJointStates()
        msg.joint_names = self.joint_names
        msg.joint_angles = self.joint_angles
        msg.desired_joint_angles = self.desired_joint_angles
        msg.joint_velocities = self.joint_velocities
        msg.joint_stiffness = self.joint_stiffness
        msg.joint_damping = self.joint_damping

        msg.torso_pose = self.torso_pose  # self.updateTorsoPose()

        # End effector calculations
#    self.updateEndEffectorPose()
        msg.hand_pose.position = geom_msgs.Point(*(self.end_effector_position.A1))
        msg.hand_pose.orientation = geom_msgs.Quaternion(*self.end_effector_orient_quat)

#    self.updateEndEffectorJacobian()
        msg.end_effector_jacobian = self.ma_to_m.matrixListToMultiarray(
            self.Je)

#    # Skin sensor calculations.
#    # Get the latest skin data from the skin client
#    skin_data = self.skin_client.getTrimmedSkinData()
#    # Trim skin_data based on specific robot state (eg wrist configuration).
#    skin_data = self.modifyRobotSpecificTaxels(skin_data)
#    # Add the list of  TaxelArray messages to the message
#    msg.skins = skin_data.values()
#    self.updateContactJacobians(skin_data)
# Add the list of  TaxelArray messages to the message
        msg.skins = self.skins
        msg.contact_jacobians = self.ma_to_m.matrixListToMultiarray(self.Jc)

        return msg

    # Build and publish the haptic state message.
    def publishRobotState(self):
        msg = self.getHapticStateMessage()

        # Publish the newly formed state message
        for i in range(len(msg.joint_names)):
            msg.joint_names[i] = str(msg.joint_names[i])  # testing
        self.state_pub.publish(msg)

        # Publish gripper pose for debug purposes
        ps_msg = geom_msgs.PoseStamped()
        ps_msg.header = self.getMessageHeader()
        ps_msg.header.frame_id = self.torso_frame

        ps_msg.pose.position = geom_msgs.Point(
            *(self.end_effector_position.A1))
        ps_msg.pose.orientation = geom_msgs.Quaternion(
            *self.end_effector_orient_quat)

        self.gripper_pose_pub.publish(ps_msg)

    # Handler for Ctrl-C signals. Some of the ROS spin loops don't respond well to
    # Ctrl-C without this.
    def signal_handler(self, signal, frame):
        print 'Ctrl+C pressed - exiting'
        sys.exit(0)

    # Start the state publisher
    def start(self):
        signal.signal(signal.SIGINT, self.signal_handler)  # Catch Ctrl-Cs

        rospy.loginfo(
            "RobotHapticState: Starting Robot Haptic State publisher")
        rate = rospy.Rate(self.rate)  # 100Hz, nominally.

        # Blocking sleep to prevent the node publishing until joint states
        # are read by the robot client.
        rospy.loginfo("RobotHapticState: Waiting for robot state")
        joint_stiffness, joint_damping = self.robot.get_joint_impedance()
        while (self.robot.get_joint_angles() is None or
               self.robot.get_joint_velocities() is None or
               joint_stiffness is None):
            joint_stiffness, joint_damping = self.robot.get_joint_impedance()
            rate.sleep()

        rospy.loginfo("RobotHapticState: Got robot state")

        while self.robot.get_ep() is None:
            rospy.sleep(0.001)
            rospy.loginfo(
                "RobotHapticState: Setting desired joint angles to current joint_angles")
            self.robot.set_ep(self.robot.get_joint_angles())

        rospy.loginfo("RobotHapticState: Starting publishing")
        while not rospy.is_shutdown():
            self.publishRobotState()
#      rospy.spin() # Blocking spin for debug/dev purposes
            rate.sleep()


def main():
    # Parse an options list specifying robot type
    import optparse
    p = optparse.OptionParser()
    haptic_mpc_util.initialiseOptParser(p)
    opt = haptic_mpc_util.getValidInput(p)
    if not opt.robot or not opt.sensor or not opt.arm:
        p.error("Robot haptic state publisher requires a specified robot, sensor, AND arm to use.")
    robot_state = RobotHapticStateServer(opt, "robot_haptic_state_server")
    robot_state.start()
