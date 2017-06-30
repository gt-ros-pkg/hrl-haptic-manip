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


import roslib; roslib.load_manifest('hrl_dynamic_mpc')
roslib.load_manifest('m3ctrl_msgs')
import rospy
import sys, time, os
import numpy as np
import time
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from m3ctrl_msgs.msg import M3JointCmd
from sensor_msgs.msg import JointState
import threading
import copy
from hrl_lib import transforms as tr
from collections import *
import darci_arm_kinematics as dak

##
#Class DarciClient()
#gives interface in python for controller to be similar to other MPC stuff we've done on Cody,
#also exposes kinematics, IK, joint limits and the ability to recorde robot data.
#

class DarciClient():
   
    def __init__(self, arm = 'l', record_data = False):
        self.arm = arm
        if arm == 'r':
            print "using right arm on Darci"
            self.RIGHT_ARM = 0  # i don't think this was used anywhere
        elif arm == 'l':
            self.LEFT_ARM = 1
        #     print "Left and both arms not implemented in this client yet ... \n"
        #     print "will require writing different function calls since joint data for left and right arm come in together from the meka server"
        #     assert(False)

        self.kinematics = dak.DarciArmKinematics(arm)
        self.JOINT_MODE_ROS_THETA_GC = 2
        self.SMOOTHING_MODE_SLEW = 1
        self.humanoid_pub = rospy.Publisher('/humanoid_command', M3JointCmd)
        self.lock = threading.RLock()
        self.joint_angles = None
        self.joint_velocities = None
        self.torque = None
        self.J_h = None
        self.time = None
        self.desired_joint_angles = None
        self.stiffness_percent = 0.7
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

        if record_data == True:
            self.q_record = deque()
            self.qd_record = deque()
            self.torque_record = deque()
            self.times = deque()

        rospy.sleep(1.0)
        self.state_sub = rospy.Subscriber('/humanoid_state', JointState, self.robotStateCallback)

        while self.joint_angles == None:
            rospy.sleep(0.01)
        self.desired_joint_angles = copy.copy(self.joint_angles)

        self.joint_cmd = M3JointCmd()
        self.joint_cmd.header.stamp = rospy.Time.now()
        self.joint_cmd.chain_idx = np.arange(7,dtype=np.int16)
        self.joint_cmd.control_mode = [self.JOINT_MODE_ROS_THETA_GC]*7  
        self.joint_cmd.stiffness = np.array([self.stiffness_percent]*7,dtype=np.float32) 
        self.joint_cmd.velocity = np.array([1.0]*7,dtype=np.float32) 
        self.joint_cmd.position = np.array(self.joint_angles,dtype=np.float32)
        self.joint_cmd.smoothing_mode = [0]*7

        if self.arm == 'l':
            self.joint_cmd.header.frame_id = 'humanoid_cmd_left'
            self.joint_cmd.chain = [self.LEFT_ARM]*7
        elif self.arm == 'r':
            self.joint_cmd.header.frame_id = 'humanoid_cmd_right'
            self.joint_cmd.chain = [self.RIGHT_ARM]*7
            
        self.humanoid_pub.publish(self.joint_cmd)

        if arm == 'l':
            self.ft_sub = rospy.Subscriber('/loadx6_left_state', Wrench, self.ftStateCallback)
        elif arm == 'r':
            self.ft_sub = rospy.Subscriber('/loadx6_right_state', Wrench, self.ftStateCallback)


    def ftStateCallback(self,msg):
        self.lock.acquire()
        pos, rot = self.kinematics.FK(self.joint_angles)
        self.ee_force = (rot*np.matrix([msg.force.x, msg.force.y, msg.force.z]).reshape(3,1)).A1.tolist()
        self.ee_torque = (rot*np.matrix([msg.torque.x, msg.torque.y, msg.torque.z]).reshape(3,1)).A1.tolist()
        self.lock.release()

    def robotStateCallback(self, msg):
        self.lock.acquire()
        if self.arm == 'l':
            self.joint_angles = copy.copy(msg.position[7:])
            self.joint_velocities = copy.copy(msg.velocity[7:])
            self.torque = copy.copy(msg.effort[7:])
        elif self.arm == 'r':
            self.joint_angles = copy.copy(msg.position[0:7])
            self.joint_velocities = copy.copy(msg.velocity[0:7])
            self.torque = copy.copy(msg.effort[0:7])
        self.time = msg.header.stamp.secs + msg.header.stamp.nsecs*(1e-9)
        self.lock.release()

    def updateHapticState(self):
        pos, rot = self.kinematics.FK(self.joint_angles)
        self.end_effector_position = pos
        self.end_effector_orient_quat = tr.matrix_to_quaternion(rot)
        self.J_h = self.kinematics.Jacobian(self.joint_angles)

    def updateSendCmd(self):
        self.joint_cmd.position = np.array(self.desired_joint_angles, dtype=np.float32)
        self.humanoid_pub.publish(self.joint_cmd)

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
        if self.record_data == True:
            self.q_record.append(copy.copy(self.joint_angles))
            self.qd_record.append(copy.copy(self.joint_velocities))
            self.torque_record.append(copy.copy(self.torque))
            self.times.append(copy.copy(self.time))
        else:
            print "you didn't pass in the right flags to record robot data ... exiting"
            assert(False)

    def getRecordedData(self):
        return self.q_record, self.qd_record, self.torque_record, self.times

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.updateHapticState()
            self.updateSendCmd()

            if self.record_data == True:
                self.q_record.append(copy.copy(self.joint_angles))
                self.qd_record.append(copy.copy(self.joint_velocities))
                self.torque_record.append(copy.copy(self.torque))
                self.times.append(copy.copy(self.time))
            rate.sleep()


        if self.record_data == True:
            import scipy.io as io
            data = {'q':self.q_record, 
                    'qd':self.qd_record,
                    'torque':self.torque_record,
                    'times':self.times}
            io.savemat('./darci_dynamics_data.mat', data)



if __name__ == '__main__':

    # this was for testing the torque values from the joints (it does not incorporate torque due to gravity)
    # however, it gives a good example of the different data and functions available to command the arm.
    rospy.init_node( 'move_arm_node', anonymous = True )
    darci = DarciClient(arm='l')
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

        #getting joint, torque and ft data before test
        joint_angles = darci.joint_angles
        torque = darci.torque
        ee_torque = darci.ee_torque
        ee_force = darci.ee_force

        inp = raw_input('q for quit, otherwise will check diff in last torque value: \n')

        #getting joint, torque, end effector and ft data after test
        darci.updateHapticState()
        joint_angles_new = darci.joint_angles
        torque_new = darci.torque
        ee_torque_new = darci.ee_torque
        ee_force_new = darci.ee_force

        #comparing predicted torque change using stiffness (quasi-static, slow motions)
        predicted_change = np.matrix(np.diag(darci.joint_stiffness))*(np.matrix(joint_angles) - np.matrix(joint_angles_new)).reshape(7,1)
        actual_change = (np.matrix(torque_new) - np.matrix(torque)).reshape(7,1)
        print "predicted change in torque :\n", predicted_change
        print "actual change in torque :\n", actual_change

        #testing if transformation of force-torque data using FK is working
        diff_real_torque = np.array(ee_torque_new) - np.array(ee_torque)
        diff_real_force = np.array(ee_force_new) - np.array(ee_force)
        print "diff real torque is :\n", diff_real_torque/1000.
        print "diff real force is :\n", diff_real_force/1000.

        # checking what joint torques would be with no gravity given force at end effector
        # we can put the arm in configurations where certain joints are not affected by gravity
        # to use this.
        J_h_T = np.matrix(darci.J_h).T
        tau = J_h_T*np.matrix(np.hstack((diff_real_force, diff_real_torque))).reshape(6,1)/1000.
        print "torque at joints due to end effector force :\n", tau
        
    # going back to home position before quitting
    darci.setDesiredJointAngles([0]*7)
    darci.updateSendCmd()
