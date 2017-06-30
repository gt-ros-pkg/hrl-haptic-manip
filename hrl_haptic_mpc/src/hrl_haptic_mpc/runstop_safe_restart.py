#!/usr/bin/env python

import sys

import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped

from hrl_haptic_manipulation_in_clutter_srvs.srv import EnableHapticMPC, EnableHapticMPCRequest


class RunStopMonitor(object):
    def __init__(self, side):
        self.side = side[0]
        rospy.wait_for_service('haptic_mpc/enable_mpc')
        self.enable_mpc_client = rospy.ServiceProxy('haptic_mpc/enable_mpc', EnableHapticMPC, persistent=True)
        self.pose_goal_pub = rospy.Publisher('haptic_mpc/goal_pose', PoseStamped, queue_size=1)
        self.joint_goal_pub = rospy.Publisher('haptic_mpc/joint_trajectory', JointTrajectory, queue_size=1)
        self.current_motor_status = None
        self.enabled = None
        self.motors_halted_sub = rospy.Subscriber('/pr2_ethercat/motors_halted', Bool, self.motor_status_cb)
        while self.current_motor_status is None:
            rospy.loginfo('[%s] Waiting for Motor Status', rospy.get_name())
            rospy.sleep(1)

    def motor_status_cb(self, status):
        self.current_motor_status = not status.data
        self.update_overall_status()

    def update_overall_status(self):
        if self.current_motor_status is None:
            return

        if self.enabled and not self.current_motor_status:
            self.disable_mpc()
        elif not self.enabled and self.current_motor_status:
            self.enable_mpc()
        self.enabled = self.current_motor_status

    def disable_mpc(self):
        rospy.loginfo("[%s] Disabling MPC on halt", rospy.get_name())
        self.enable_mpc_client.call(EnableHapticMPCRequest('disabled'))
        # Send empty msg to clear waypoint generator's deque
        self.joint_goal_pub.publish(JointTrajectory())
        self.pose_goal_pub.publish(PoseStamped())

    def enable_mpc(self):
            rospy.loginfo("[%s] Enabling MPC on restart", rospy.get_name())
            self.joint_goal_pub.publish(JointTrajectory())
            self.pose_goal_pub.publish(PoseStamped())
            rospy.sleep(0.4)
            self.enable_mpc_client.call(EnableHapticMPCRequest('enabled'))


def main():
    rospy.init_node('haptic_mpc_runstop_monitor')
    args = rospy.myargv(sys.argv)
    rsm = RunStopMonitor(args[1])
    rospy.spin()

if __name__ == '__main__':
    main()
