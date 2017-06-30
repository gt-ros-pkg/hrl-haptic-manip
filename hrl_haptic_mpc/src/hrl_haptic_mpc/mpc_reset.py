#!/usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

from hrl_haptic_manipulation_in_clutter_srvs.srv import EnableHapticMPC
from hrl_haptic_manipulation_in_clutter_msgs.msg import HapticMpcWeights


RESET_ANGLES = [0.795, 0.298, 1.641, -2.191, 3.2344, -0.879, 2.00]


class HapticMPCReset(object):

    def __init__(self):
        self.joint_names = None
        self.joint_angles = RESET_ANGLES
        self.ee_pose = None
        self.enable_mpc_client = rospy.ServiceProxy(
            "/haptic_mpc/enable_mpc", EnableHapticMPC)
        self.joint_controller = rospy.get_param(
            "~controller", "l_arm_controller")
        self.traj_pub = rospy.Publisher(
            'haptic_mpc/joint_trajectory', JointTrajectory)
        self.feedback_pub = rospy.Publisher("fdbk_out", String)

        self.joint_state_sub = rospy.Subscriber(
            "%s/state" % self.joint_controller, JointTrajectoryControllerState, self.joint_state_cb)
        self.ee_pose_sub = rospy.Subscriber(
            "haptic_mpc/gripper_pose", PoseStamped, self.ee_pose_cb)
        self.goal_pose_pub = rospy.Publisher(
            "haptic_mpc/goal_pose", PoseStamped)
        self.reset_sub = rospy.Subscriber("reset", Bool, self.reset_cb)
        self.mpc_weights_pub = rospy.Publisher(
            "haptic_mpc/weights", HapticMpcWeights)

    def joint_state_cb(self, msg):
        self.joint_angles = msg.actual.positions
        if self.joint_names is None:
            self.joint_names = msg.joint_names

    def ee_pose_cb(self, msg):
        self.ee_pose = msg

    def send_feedback(self, msg):
        rospy.loginfo("[%s] %s" % (rospy.get_name(), msg))
        self.feedback_pub.publish(msg)

    def setControllerWeights(self, position_weight, orient_weight, posture_weight):
        weights_msg = HapticMpcWeights()
        weights_msg.header.stamp = rospy.Time.now()
        weights_msg.position_weight = position_weight
        weights_msg.orient_weight = orient_weight
        weights_msg.posture_weight = posture_weight
        # Enable position tracking only - disable orientation by setting the
        # weight to 0
        self.mpc_weights_pub.publish(weights_msg)

    def reset_cb(self, msg):
        while self.joint_names is None:
            rospy.logerr(
                "[%s] Waiting for joint state information" % rospy.get_name())
            rospy.sleep(0.5)
        while self.ee_pose is None:
            rospy.logerr(
                "[%s] Waiting for gripper pose information" % rospy.get_name())
            rospy.sleep(0.5)
        if msg.data:

            # Stop Skin Controller
            # self.send_feedback("Stopping skin controller to reset arm.")
            # req = EnableHapticMPCRequest()
            # req.new_state = "disabled"
            # resp = self.enable_mpc_client.call(req)
            # self.goal_pose_pub.publish(PoseStamped())
            self.setControllerWeights(0.0, 0.0, 1.0)

            # Send Joint Goal to reset arm
            jtp = JointTrajectoryPoint()
            jtp.positions = RESET_ANGLES
            err = np.max(np.subtract(RESET_ANGLES, self.joint_angles))
            jtp.time_from_start = rospy.Duration(err / (np.pi / 8.))
            jt = JointTrajectory()
            jt.joint_names = self.joint_names
            jt.points.append(jtp)
            self.traj_pub.publish(jt)
            rospy.sleep(jtp.time_from_start + rospy.Duration(1.0))
            # self.goal_pose_pub.publish(self.ee_pose)
            rospy.sleep(5.0)
            self.setControllerWeights(5.0, 4.0, 0.0)
            self.send_feedback("Arm reset successfully.")

            # Restart skin controller
            # req.new_state = "enabled"
            # resp = self.enable_mpc_client.call(req)


def main():
    rospy.init_node('haptic_mpc_reset')
    mpc_reset = HapticMPCReset()
    rospy.spin()
