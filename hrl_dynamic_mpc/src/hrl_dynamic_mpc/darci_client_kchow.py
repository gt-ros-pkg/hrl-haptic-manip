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
from geometry_msgs.msg import Twist, PoseStamped
from m3ctrl_msgs.msg import M3JointCmd
from sensor_msgs.msg import JointState
import threading
import copy
from hrl_lib import transforms as tr
from collections import *
import darci_arm_kinematics as dak
from tf import TransformListener

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
            self.kinematics = dak.DarciArmKinematics(arm)
        elif arm == 'l':
            self.LEFT_ARM = 1
            self.kinematics = dak.DarciArmKinematics(arm)
        elif arm == 'lr':
            self.LEFT_ARM = 1
            self.RIGHT_ARM = 0
            self.kinematics = dak.DarciArmKinematics('l')
            self.kinematics2 = dak.DarciArmKinematics('r')
        #     print "Left and both arms not implemented in this client yet ... \n"
        #     print "will require writing different function calls since joint data for left and right arm come in together from the meka server"
        #     assert(False)

        self.JOINT_MODE_ROS_THETA_GC = 2
        self.SMOOTHING_MODE_SLEW = 1
        self.humanoid_pub = rospy.Publisher('/humanoid_command', M3JointCmd)
        self.lock = threading.RLock()
        self.joint_angles = None
        self.joint_angles2 = None
        self.joint_velocities = None
        self.joint_velocities2 = None
        self.torque = None
        self.J_h = None
        self.time = None
        self.desired_joint_angles = None
        self.desired_joint_angles2 = None
        self.stiffness_percent = 1.0
        self.ee_force = None
        self.ee_torque = None
	self.bias_values = None
	self.bias_values2 = None
        
        self.skins = None #will need code for all of these skin interfaces
        self.Jc_l = []
        self.n_l = []
        self.values_l = []

	self.tf_listener = TransformListener()

        #values from m3gtt repository in robot_config folder
        # These could be read in from a yaml file like this (import yaml; stream = open("FILE_NAME_HERE", 'r'); data = yaml.load(stream))
        # However, not clear if absolute path to yaml file (possibly on another computer) is better then just defining it here
        # The downside is obviously if someone tunes gains differently on the robot.
        self.joint_stiffness = (np.array([1, 1, 1, 1, 0.06, 0.08, 0.08])*180/np.pi*self.stiffness_percent).tolist()
        print "joint_stiffness_percent: ",self.stiffness_percent
        self.joint_damping = (np.array([0.06, 0.1, 0.015, 0.015, 0.0015, 0.002, 0.002])*180/np.pi*self.stiffness_percent).tolist()
        
        self.record_data = record_data

        if record_data == True:
            self.q_record = deque()
            self.qd_record = deque()
            self.torque_record = deque()
            self.times = deque()

        rospy.sleep(0.1)
        self.state_sub = rospy.Subscriber('/humanoid_state', JointState, self.robotStateCallback)

        while self.joint_angles == None:
            rospy.sleep(0.01)

        self.joint_cmd = M3JointCmd()
        self.joint_cmd.header.stamp = rospy.Time.now()
        self.joint_cmd.chain_idx = np.arange(7,dtype=np.int16)
        self.joint_cmd.control_mode = [self.JOINT_MODE_ROS_THETA_GC]*7  
        self.joint_cmd.stiffness = np.array([self.stiffness_percent]*7,dtype=np.float32) 
        self.joint_cmd.velocity = np.array([1.0]*7,dtype=np.float32) 
        self.joint_cmd.position = np.array(self.joint_angles,dtype=np.float32)
        self.joint_cmd.smoothing_mode = [0]*7

        if self.arm == 'l':
            self.desired_joint_angles = copy.copy(self.joint_angles)
            self.joint_cmd.header.frame_id = 'humanoid_cmd_left'
            self.joint_cmd.chain = [self.LEFT_ARM]*7
            self.humanoid_pub.publish(self.joint_cmd)
        elif self.arm == 'r':
            self.desired_joint_angles = copy.copy(self.joint_angles)
            self.joint_cmd.header.frame_id = 'humanoid_cmd_right'
            self.joint_cmd.chain = [self.RIGHT_ARM]*7
            self.humanoid_pub.publish(self.joint_cmd)
        elif self.arm == 'lr':
            self.desired_joint_angles = copy.copy(self.joint_angles)
            self.desired_joint_angles2 = copy.copy(self.joint_angles2)
            self.joint_cmd.header.frame_id = 'humanoid_cmd_left'
            self.joint_cmd.chain = [self.LEFT_ARM]*7
            self.joint_cmd2 = copy.deepcopy(self.joint_cmd)
	    self.joint_cmd2.position = np.array(self.joint_angles2,dtype=np.float32)
            self.joint_cmd2.header.frame_id = 'humanoid_cmd_right'
            self.joint_cmd2.chain = [self.RIGHT_ARM]*7
            self.humanoid_pub.publish(self.joint_cmd)
            self.humanoid_pub.publish(self.joint_cmd2)
            

        if arm == 'l':
            self.ft_sub = rospy.Subscriber('/loadx6_left_state', Wrench, self.ftStateCallback)
        elif arm == 'r':
            self.ft_sub = rospy.Subscriber('/loadx6_right_state', Wrench, self.ftStateCallback)
        elif arm == 'lr':
            self.ft_sub = rospy.Subscriber('/loadx6_left_state', Wrench, self.ftStateCallback)
            self.ft_sub2 = rospy.Subscriber('/loadx6_right_state', Wrench, self.ftStateCallback2)

    def zeroftbias(self):
	self.bias_values = None

    def zeroftbias2(self):
	self.bias_values2 = None

    def ftStateCallback(self,msg):
	# bias the force torque sensor (usually set elbow joint to 90 degrees)
	# maybe eventually trying taking the average values over a data-gathering period of time
	if self.bias_values == None:
	    self.bias_values = [msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]

        self.lock.acquire()
        pos, rot = self.kinematics.FK(self.joint_angles)
        self.ee_force = (rot*np.matrix([msg.force.x-self.bias_values[0], msg.force.y-self.bias_values[1], msg.force.z-self.bias_values[2]]).reshape(3,1)).A1.tolist()
        self.ee_torque = (rot*np.matrix([msg.torque.x-self.bias_values[3], msg.torque.y-self.bias_values[4], msg.torque.z-self.bias_values[5]]).reshape(3,1)).A1.tolist()
        self.lock.release()

    def ftStateCallback2(self,msg):
	# bias the force torque sensor (usually set elbow joint to 90 degrees)
	if self.bias_values2 == None:
	    self.bias_values2 = [msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z]

        self.lock.acquire()
        pos, rot = self.kinematics.FK(self.joint_angles2)
        self.ee_force2 = (rot*np.matrix([msg.force.x-self.bias_values2[0], msg.force.y-self.bias_values2[1], msg.force.z-self.bias_values2[2]]).reshape(3,1)).A1.tolist()
        self.ee_torque2 = (rot*np.matrix([msg.torque.x-self.bias_values2[3], msg.torque.y-self.bias_values2[4], msg.torque.z-self.bias_values2[5]]).reshape(3,1)).A1.tolist()
        self.lock.release()

    def robotStateCallback(self, msg):
        self.lock.acquire()
        if self.arm == 'l':
	    self.joint_names = copy.copy(msg.name[7:])
            self.joint_angles = copy.copy(msg.position[7:])
            self.joint_velocities = copy.copy(msg.velocity[7:])
            self.torque = copy.copy(msg.effort[7:])
        elif self.arm == 'r':
	    self.joint_names = copy.copy(msg.name[0:7])
            self.joint_angles = copy.copy(msg.position[0:7])
            self.joint_velocities = copy.copy(msg.velocity[0:7])
            self.torque = copy.copy(msg.effort[0:7])
        elif self.arm == 'lr':
	    self.joint_names = copy.copy(msg.name[7:])
            self.joint_angles = copy.copy(msg.position[7:])
            self.joint_velocities = copy.copy(msg.velocity[7:])
            self.torque = copy.copy(msg.effort[7:])
	    self.joint_names2 = copy.copy(msg.name[0:7])
            self.joint_angles2 = copy.copy(msg.position[0:7])
            self.joint_velocities2 = copy.copy(msg.velocity[0:7])
            self.torque2 = copy.copy(msg.effort[0:7])
        self.time = msg.header.stamp.secs + msg.header.stamp.nsecs*(1e-9)
        self.lock.release()

    def updateHapticState(self):
        pos, rot = self.kinematics.FK(self.joint_angles)
        self.end_effector_position = pos
        self.end_effector_orient_quat = tr.matrix_to_quaternion(rot)
        self.J_h = [self.kinematics.Jacobian(self.joint_angles)]

	if self.arm == 'lr':
	    self.larm_ee_p = PoseStamped()
            self.larm_ee_p.header.frame_id = 'torso_lift_link'
            self.larm_ee_p.pose.position.x = self.end_effector_position[0,0]
            self.larm_ee_p.pose.position.y = self.end_effector_position[1,0]
            self.larm_ee_p.pose.position.z = self.end_effector_position[2,0]
            self.larm_ee_p.pose.orientation.x = self.end_effector_orient_quat[0]
            self.larm_ee_p.pose.orientation.y = self.end_effector_orient_quat[1]
            self.larm_ee_p.pose.orientation.z = self.end_effector_orient_quat[2]
            self.larm_ee_p.pose.orientation.w = self.end_effector_orient_quat[3]

	    pos, rot = self.kinematics2.FK(self.joint_angles2)
            self.end_effector_position2 = pos
            self.end_effector_orient_quat2 = tr.matrix_to_quaternion(rot)
	    self.rarm_ee_p = PoseStamped()
	    self.rarm_ee_p.header.frame_id = 'torso_lift_link'
	    self.rarm_ee_p.pose.position.x = self.end_effector_position2[0]
	    self.rarm_ee_p.pose.position.y = self.end_effector_position2[1]
	    self.rarm_ee_p.pose.position.z = self.end_effector_position2[2]
	    self.rarm_ee_p.pose.orientation.x = self.end_effector_orient_quat2[0]
	    self.rarm_ee_p.pose.orientation.y = self.end_effector_orient_quat2[1]
	    self.rarm_ee_p.pose.orientation.z = self.end_effector_orient_quat2[2]
	    self.rarm_ee_p.pose.orientation.w = self.end_effector_orient_quat2[3]
            self.J_h2 = [self.kinematics2.Jacobian(self.joint_angles2)]

   	    # find Jacobian at middle of forearm
	    
	    ps = PoseStamped()
    	    #ps.header.frame_id = 'wrist_LEFT'
    	    ps.header.frame_id = 'elbowclevis_LEFT'
            #ps.pose.position.z = -0.138735
            ps.pose.position.z = -(4*0.138735-(0.0864*2-0.08128))/2.
	    self.tf_listener.waitForTransform('torso_lift_link','elbowclevis_LEFT',rospy.Time(),rospy.Duration(5))
            self.larm_f_p = self.tf_listener.transformPose('torso_lift_link',ps)
            larm_jacobian_pos = np.matrix([self.larm_f_p.pose.position.x,self.larm_f_p.pose.position.y,self.larm_f_p.pose.position.z]).reshape(3,1)
            self.lforearm_J = [self.kinematics.jacobian(self.joint_angles, larm_jacobian_pos)]
            self.lforearm_J[0][:,5] = 0
            self.lforearm_J[0][:,6] = 0


	    ps = PoseStamped()
            #ps.header.frame_id = 'wrist_RIGHT'
            ps.header.frame_id = 'elbowclevis_RIGHT'
            #ps.pose.position.z = -0.138735
            ps.pose.position.z = -(4*0.138735-(0.0864*2-0.08128))/2.
	    self.tf_listener.waitForTransform('torso_lift_link','elbowclevis_RIGHT',rospy.Time(),rospy.Duration(5))
            self.rarm_f_p = self.tf_listener.transformPose('torso_lift_link',ps)
            rarm_jacobian_pos = np.matrix([self.rarm_f_p.pose.position.x,self.rarm_f_p.pose.position.y,self.rarm_f_p.pose.position.z]).reshape(3,1)
            self.rforearm_J = [self.kinematics2.jacobian(self.joint_angles2, larm_jacobian_pos)]
            self.rforearm_J[0][:,5] = 0
            self.rforearm_J[0][:,6] = 0
	    		

    def updateSendCmd(self):
        self.joint_cmd.position = np.array(self.desired_joint_angles, dtype=np.float32)
        self.humanoid_pub.publish(self.joint_cmd)
	if self.arm == 'lr':
	    self.joint_cmd2.position = np.array(self.desired_joint_angles2, dtype=np.float32)
            self.humanoid_pub.publish(self.joint_cmd2)
	

    def addDeltaToDesiredJointAngles(self, delta_angles, delta_angles2 = [0,0,0,0,0,0,0]):
        self.desired_joint_angles = (np.array(self.desired_joint_angles) + np.array(delta_angles)).tolist()
	if self.arm == 'lr':
            self.desired_joint_angles2 = (np.array(self.desired_joint_angles2) + np.array(delta_angles2)).tolist()

    def setDesiredJointAngles(self, angles, angles2 = [0,0,0,0,0,0,0]):
        self.desired_joint_angles = angles
	if self.arm == 'lr':
	    self.desired_joint_angles2 = angles2

    def getDesiredJointAngles(self):
        return self.desired_joint_angles

    def getDesiredJointAngles2(self):
        return self.desired_joint_angles2

    def getJointAngles(self):
        return self.joint_angles

    def get_joint_angles(self):
        return self.getJointAngles()

    def getJointAngles2(self):
	return self.joint_angles2

    def getJointVelocities(self):
        return self.joint_velocities

    def getJointVelocities2(Self):
	return self.joint_velocities2

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
