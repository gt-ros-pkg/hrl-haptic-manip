#!/usr/bin/python
#
#
# Copyright (c) 2013, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \authors: Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
# \adviser: Charles Kemp (Healthcare Robotics Lab, Georgia Tech.)

import sys, time, copy
import numpy as np
from threading import RLock

import roslib; roslib.load_manifest('hrl_dynamic_mpc')
roslib.load_manifest('pr2_controllers_msgs')
import rospy
from geometry_msgs.msg import Wrench, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from hrl_lib import transforms as tr

import darci_arm_kinematics as dak

##
#Class DarciSimClient()
#gives interface in python for controller to be similar to other MPC stuff we've done on Cody,
#also exposes kinematics, IK, joint limits and the ability to recorde robot data.
#

class DarciSimClient():

    def __init__(self, arm = 'l', record_data = False):
        self.arm = arm
        if arm == 'r':
            print "using right arm on Darci"
            self.RIGHT_ARM = 0  # i don't think this was used anywhere
        else:
            self.LEFT_ARM = 1
        #     print "Left and both arms not implemented in this client yet ... \n"
        #     print "will require writing different function calls since joint data for left and right arm come in together from the meka server"
        #     sys.exit()

        self.kinematics = dak.DarciArmKinematics(arm)
        self.joint_pub = rospy.Publisher('/'+self.arm+'_arm_controller/command', JointTrajectory)
        self.joint_names = None
        self.lock = RLock()
        self.joint_angles = None
        self.joint_velocities = None
        self.J_h = None
        self.time = None
        self.desired_joint_angles = None
        self.stiffness_percent = 0.75
        self.ee_force = None
        self.ee_torque = None
        
        self.skins = None #will need code for all of these skin interfaces
        self.Jc_l = []
        self.n_l = []
        self.values_l = []

        #values from m3gtt repository in robot_config folder
        # These could be read in from a yaml file like this (import yaml; stream = open("FILE_NAME_HERE", 'r'); data = yaml.load(stream))
        # However, not clear if absolute path to yaml file (possibly on another computer) is better then just defining it here
        # The downside is obviously if someone tunes gains differently on the robot.
        self.joint_stiffness = (np.array([1, 1, 1, 1, 0.06, 0.08, 0.08])*180/np.pi*self.stiffness_percent).tolist()
        self.joint_damping = (np.array([0.06, 0.1, 0.015, 0.015, 0.0015, 0.002, 0.002])*180/np.pi*self.stiffness_percent).tolist()
        
        self.record_data = record_data

        if self.record_data:
            from collections import deque
            self.q_record = deque()
            self.qd_record = deque()
            self.times = deque()

        self.state_sub = rospy.Subscriber('/'+self.arm+'_arm_controller/state', JointTrajectoryControllerState, self.robotStateCallback)
        rospy.sleep(1.0)

        while self.joint_angles is None:
            rospy.sleep(0.05)
        self.desired_joint_angles = copy.copy(self.joint_angles)

        self.joint_cmd = JointTrajectory()
        self.joint_cmd.header.stamp = rospy.Time.now()
        self.joint_cmd.header.frame_id = '/torso_lift_link'
        self.joint_cmd.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = self.desired_joint_angles
        jtp.velocities = [1.]*len(self.joint_names)
        self.joint_cmd.points = [jtp]
        self.joint_pub.publish(self.joint_cmd)

    def robotStateCallback(self, msg):
        self.lock.acquire()
        if self.joint_names is None:
            self.joint_names = copy.copy(msg.joint_names)
        self.joint_angles = copy.copy(msg.actual.positions)
        self.joint_velocities = copy.copy(msg.actual.velocities)
        self.time = msg.header.stamp.secs + msg.header.stamp.nsecs*(1e-9)
        self.lock.release()

    def updateHapticState(self):
        pos, rot = self.kinematics.FK(self.joint_angles)
        self.end_effector_position = pos
        self.end_effector_orient_quat = tr.matrix_to_quaternion(rot)
        self.J_h = self.kinematics.Jacobian(self.joint_angles)

    def updateSendCmd(self):
        self.joint_cmd.points[0].positions = np.array(self.desired_joint_angles, dtype=np.float32).tolist()
        self.joint_pub.publish(self.joint_cmd)

    def addDeltaToDesiredJointAngles(self, delta_angles):
        self.desired_joint_angles = (np.array(self.desired_joint_angles) + np.array(delta_angles)).tolist()

    def setDesiredJointAngles(self, angles):
        self.desired_joint_angles = angles

    def getDesiredJointAngles(self):
        return self.desired_joint_angles

    def getJointAngles(self):
        return self.joint_angles

    def get_joint_angles(self):
        return self.getJointAngles()

    def getJointVelocities(self):
        return self.joint_velocities

    def recordCurData(self):
        if self.record_data:
            self.q_record.append(copy.copy(self.joint_angles))
            self.qd_record.append(copy.copy(self.joint_velocities))
            self.times.append(copy.copy(self.time))
        else:
            print "you didn't pass in the right flags to record robot data ... exiting"
            sys.exit()

    def getRecordedData(self):
        return self.q_record, self.qd_record, self.times

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.updateHapticState()
            self.updateSendCmd()

            if self.record_data == True:
                self.q_record.append(copy.copy(self.joint_angles))
                self.qd_record.append(copy.copy(self.joint_velocities))
                self.times.append(copy.copy(self.time))
            rate.sleep()


        if self.record_data:
            import scipy.io as io
            data = {'q':self.q_record,
                    'qd':self.qd_record,
                    'times':self.times}
            io.savemat('./darci_dynamics_data.mat', data)

if __name__ == '__main__':

    # this was for testing the values from the joints
    # however, it gives a good example of the different data and functions available to command the arm.
    rospy.init_node( 'move_arm_node', anonymous = True )
    darci = DarciSimClient(arm='l')
    rospy.sleep(5)
    inp = None
    while inp != 'q':
        #sending simple command to joints
        angles = [0.0]*7
        #angles[3] = np.pi/2

        # uploading command to arms through ROS
        darci.setDesiredJointAngles(angles)
        darci.updateSendCmd()

        # updating end effector position and orientation
        darci.updateHapticState()

        joint_angles = darci.joint_angles

        inp = raw_input('q for quit, otherwise continue: \n')

        #getting joint info after test
        darci.updateHapticState()
        joint_angles_new = darci.joint_angles

    # going back to home position before quitting
    darci.setDesiredJointAngles([0]*7)
    darci.updateSendCmd()
