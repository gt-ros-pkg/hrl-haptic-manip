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
# Authors: Kevin Chow
# Healthcare Robotics Laboratory


#######################################################
# 
# cRoNA sensor
# 
########################################################

import roslib
roslib.load_manifest('LMPC')
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool
from sttr_msgs.msg import CronaState, RagdollObjectArray
from pykdl_utils.kdl_kinematics import create_kdl_kin
from LMPC_msgs.msg import RobotHapticState, TaxelArray
import urdf_crona2
from geometry_msgs.msg import PoseStamped, Twist, Pose
import csv
from tf import TransformListener
import copy


class CronaSensor(object):
    def __init__(self):
        rospy.init_node('crona_sensor')
	rospy.Subscriber('crona/sim_state',CronaState,self.update_state)
	rospy.Subscriber('crona/clear_jt_feedforward',Bool,self.check_clear)
	## only in simulation
	rospy.Subscriber('gazebo/objectcog',RagdollObjectArray,self.update_ragdoll_state)
	
	self.larm_sensor_pub = rospy.Publisher('/l_arm/crona/est_force',TaxelArray)
	self.rarm_sensor_pub = rospy.Publisher('/r_arm/crona/est_force',TaxelArray)
	self.jt_ff_pub = rospy.Publisher('/crona/jt_feedforward',Float64MultiArray)
	self.larm_force_values = [np.zeros(20).tolist(),np.zeros(20).tolist(),np.zeros(20).tolist()]
	self.rarm_force_values = [np.zeros(20).tolist(),np.zeros(20).tolist(),np.zeros(20).tolist()]
	self.larm_jt = [0 for i in range(6)]
	self.rarm_jt = [0 for i in range(6)]
	self.old_larm_force_z = 0
	self.old_rarm_force_z = 0
	self.clear = 0
	self.larm_kinematics = create_kdl_kin('base_link', 'l_hand_link')
	self.rarm_kinematics = create_kdl_kin('base_link', 'r_hand_link')
	self.tf_listener = TransformListener()
	
	#self.larm_force_values = [np.zeros(50).tolist(),np.zeros(50).tolist(),np.zeros(50).tolist()]
	#self.rarm_force_values = [np.zeros(50).tolist(),np.zeros(50).tolist(),np.zeros(50).tolist()]

	self.torso_pose = Pose()

    def update_ragdoll_state(self,msg):
	world_ragdoll_ps = msg
	ps = PoseStamped()
	ps.header.frame_id = 'world'
	if 'forearm_roll' not in world_ragdoll_ps.frame_names[-1]:
            ps.pose.position.x = world_ragdoll_ps.centers_x[-1]
            ps.pose.position.y = world_ragdoll_ps.centers_y[-1]
            ps.pose.position.z = world_ragdoll_ps.centers_z[-1]
    	else:
            ps.pose.position.x = (world_ragdoll_ps.centers_x[0]+world_ragdoll_ps.centers_x[1])/2
            ps.pose.position.y = 0
            ps.pose.position.z = (world_ragdoll_ps.centers_z[0]+world_ragdoll_ps.centers_z[1])/2
	self.ragdoll_ps = self.tf_listener.transformPose('base_link',ps)

    def check_clear(self,msg):
	self.clear = msg.data
	if self.clear:
	    self.larm_force_values = [np.zeros(20).tolist(),np.zeros(20).tolist(),np.zeros(20).tolist()]
            self.rarm_force_values = [np.zeros(20).tolist(),np.zeros(20).tolist(),np.zeros(20).tolist()]

    def update_state(self,msg):
	self.larm_ja = [msg.actual_position[5],msg.actual_position[6],msg.actual_position[7],msg.actual_position[8],msg.actual_position[9],msg.actual_position[10]]
	self.rarm_ja = [msg.actual_position[11],msg.actual_position[12],msg.actual_position[13],msg.actual_position[14],msg.actual_position[15],msg.actual_position[16]]
	self.torso_ja = [msg.actual_position[3]]
	#self.larm_je = [msg.effort[5],msg.effort[6],msg.effort[7],msg.effort[8],msg.effort[9],msg.effort[10]]
	#self.rarm_je = [msg.effort[11],msg.effort[12],msg.effort[13],msg.effort[14],msg.effort[15],msg.effort[16]]
	#self.larm_je = (np.array(self.larm_je)+np.array(self.larm_jt)).tolist()
	#self.rarm_je = (np.array(self.rarm_je)+np.array(self.rarm_jt)).tolist()
	self.larm_je = [msg.joint_torque[5],msg.joint_torque[6],msg.joint_torque[7],msg.joint_torque[8],msg.joint_torque[9],msg.joint_torque[10]]
	self.rarm_je = [msg.joint_torque[11],msg.joint_torque[12],msg.joint_torque[13],msg.joint_torque[14],msg.joint_torque[15],msg.joint_torque[16]]
	self.torso_je = [msg.joint_torque[3]]

	try:
	    ps = PoseStamped()
	    ps.header.frame_id = 'torso_link'
	    self.torso_pose = self.tf_listener.transformPose('base_link',ps)
	    moment_arm = self.ragdoll_ps.pose.position.x-self.torso_pose.pose.position.x
	    self.estimated_weight = self.torso_je[0]/moment_arm
	except:
	    pass	    
	
    def calculate_larm_jacobian(self):
	forearm_pos = self.larm_kinematics.forward(self.torso_ja+self.larm_ja,end_link='l_forearm_roll_link',base_link='base_link')
	hand_pos = self.larm_kinematics.forward(self.torso_ja+self.larm_ja,end_link='l_hand_link',base_link='base_link')
	self.larm_avg_pos = (forearm_pos[:3,3]+hand_pos[:3,3])/2
        #print "larm_avg_pos: ",self.larm_avg_pos
    	self.larm_jacobian = [self.larm_kinematics.jacobian(self.torso_ja+self.larm_ja,self.larm_avg_pos)]
	for i in range(6):
	    #print i
	    self.larm_jacobian[0][i,6] = 0

    def calculate_rarm_jacobian(self):
        forearm_pos = self.rarm_kinematics.forward(self.torso_ja+self.rarm_ja,end_link='r_forearm_roll_link',base_link='base_link')
        hand_pos = self.rarm_kinematics.forward(self.torso_ja+self.rarm_ja,end_link='r_hand_link',base_link='base_link')
	self.rarm_avg_pos = (forearm_pos[:3,3]+hand_pos[:3,3])/2
        #print "rarm_avg_pos: ",self.rarm_avg_pos
        self.rarm_jacobian = [self.rarm_kinematics.jacobian(self.torso_ja+self.rarm_ja,self.rarm_avg_pos)]
	for i in range(6):
	    self.rarm_jacobian[0][i,6] = 0

    def calculate_larm_force(self):
	self.calculate_larm_jacobian()
	#print "self.larm_jacobian: ",self.larm_jacobian
	#print "original: ",self.larm_force_values[0]
	#self.calc_l_forearm_force = np.linalg.inv(np.transpose(np.matrix(self.larm_jacobian[0][:5,:5]).reshape(5,5)))*(np.matrix(self.larm_je[:5])*-1).reshape(5,1)
	#self.calc_l_forearm_force = np.linalg.pinv(np.transpose(np.matrix(self.larm_jacobian[0][:3,:6]).reshape(3,6)))*(-1*np.matrix((self.torso_je+self.larm_je[:5])).reshape(6,1))
	self.calc_l_forearm_force = np.linalg.pinv(np.transpose(np.matrix(self.larm_jacobian[0][:3,1:6]).reshape(3,5)))*(-1*np.matrix((self.larm_je[:5])).reshape(5,1))
	self.larm_force_values[0].append(float(self.calc_l_forearm_force[0]))
        self.larm_force_values[1].append(float(self.calc_l_forearm_force[1]))
        self.larm_force_values[2].append(float(self.calc_l_forearm_force[2]))
	del self.larm_force_values[0][0]
        del self.larm_force_values[1][0]
        del self.larm_force_values[2][0]
	self.larm_force = [sum(self.larm_force_values[0])/len(self.larm_force_values[0]),sum(self.larm_force_values[1])/len(self.larm_force_values[1]),sum(self.larm_force_values[2])/len(self.larm_force_values[2])]
	self.old_larm_force_z = copy.copy(self.larm_force[2])
	self.larm_force[2] = (self.old_larm_force_z/(abs(self.old_larm_force_z)+abs(self.old_rarm_force_z)))*self.estimated_weight
	#print "larm_force: ",self.larm_force

    def calculate_rarm_force(self):
	self.calculate_rarm_jacobian()
	#print "self.rarm_jacobian: ",self.rarm_jacobian
	#print "original: ",self.rarm_force_values[0]
	#self.calc_r_forearm_force = np.linalg.inv(np.transpose(np.matrix(self.rarm_jacobian[0][:5,:5]).reshape(5,5)))*(np.matrix(self.rarm_je[:5])*-1).reshape(5,1)	
	#self.calc_r_forearm_force = np.linalg.pinv(np.transpose(np.matrix(self.rarm_jacobian[0][:3,:6]).reshape(3,5)))*(-1*np.matrix(self.torso_je+self.rarm_je[:5])).reshape(6,1)	
	self.calc_r_forearm_force = np.linalg.pinv(np.transpose(np.matrix(self.rarm_jacobian[0][:3,1:6]).reshape(3,5)))*(-1*np.matrix(self.rarm_je[:5])).reshape(5,1)	
	self.rarm_force_values[0].append(float(self.calc_r_forearm_force[0]))
        self.rarm_force_values[1].append(float(self.calc_r_forearm_force[1]))
        self.rarm_force_values[2].append(float(self.calc_r_forearm_force[2]))
	del self.rarm_force_values[0][0]
	del self.rarm_force_values[1][0]
        del self.rarm_force_values[2][0]
	self.rarm_force = [sum(self.rarm_force_values[0])/len(self.rarm_force_values[0]),sum(self.rarm_force_values[1])/len(self.rarm_force_values[1]),sum(self.rarm_force_values[2])/len(self.rarm_force_values[2])]
	self.old_rarm_force_z = copy.copy(self.rarm_force[2])
	self.rarm_force[2] = (self.old_rarm_force_z/(abs(self.old_larm_force_z)+abs(self.old_rarm_force_z)))*self.estimated_weight
	#print "rarm_force: ",self.rarm_force


    def calculate_jt_feedforward(self):
	# make local copy
	larm_force = self.larm_force[:]
	rarm_force = self.rarm_force[:]
	larm_force += [0,0,0]
        rarm_force += [0,0,0]	
	# W = W_1+W_2
	estimated_weight = 1000
	# W = scale*(F_1+F_2)
	self.norm_larm_force = np.linalg.norm(larm_force)
	self.norm_rarm_force = np.linalg.norm(rarm_force)
	larm_within_jl, rarm_within_jl = self.check_joint_limits()
	#print "larm_within_jl: ",larm_within_jl
	#print "rarm_within_jl: ",rarm_within_jl
	# help stop drift
	print "self.larm_force[0]: ", self.larm_force[0]
	print "self.rarm_force[0]: ", self.rarm_force[0]
	if self.larm_force[0]<5 or not larm_within_jl:
	    larm_force = [0,0,0,0,0,0]
	    print "LARM!"
	if self.rarm_force[0]<5 or not rarm_within_jl:
	    rarm_force = [0,0,0,0,0,0]
	    print "RARM!"
	if (self.norm_larm_force+self.norm_rarm_force)>estimated_weight:
	#if (self.larm_force[0]+self.rarm_force[0])>estimated_weight:
	    # scale = W/(F_1+F_2)
	    scale = estimated_weight/(self.norm_larm_force+self.norm_rarm_force)
	    print "scale: ",scale
	    #print "larm_force: ",larm_force
	    #print "rarm_force: ",rarm_force
	    # scaled force
	    larm_scaled_force = [scale*larm_force[i] for i in range(len(larm_force))]
	    rarm_scaled_force = [scale*rarm_force[i] for i in range(len(rarm_force))]
	else:
	    larm_scaled_force = larm_force
            rarm_scaled_force = rarm_force
	print "larm_scaled_force: ",larm_scaled_force
	print "rarm_scaled_force: ",rarm_scaled_force
	#print "larm_scaled_force: ",larm_scaled_force
	#print "rarm_scaled_force: ",rarm_scaled_force
	larm_temp = np.transpose(np.matrix(self.larm_jacobian[0][:6,:6]).reshape(6,6))*(np.matrix(larm_scaled_force).reshape(6,1))
	self.larm_jt = [-1*float(i) for i in larm_temp]
	rarm_temp = np.transpose(np.matrix(self.rarm_jacobian[0][:6,:6]).reshape(6,6))*(np.matrix(rarm_scaled_force).reshape(6,1))
	self.rarm_jt = [-1*float(i) for i in rarm_temp]	
	#print "self.larm_jt: ",self.larm_jt
	#print "self.rarm_jt: ",self.rarm_jt
	jt_msg = Float64MultiArray()
	jt_msg.data += self.larm_jt[:]
 	jt_msg.data += self.rarm_jt[:]
	#self.jt_ff_pub.publish(jt_msg)
	
    def create_larm_messages(self):
	larm_msg = TaxelArray()
        larm_msg.header.frame_id = 'base_link'
	larm_msg.link_names = ['l_forearm_roll_link']
        larm_msg.centers_x = [self.larm_avg_pos[0]]
        larm_msg.centers_y = [self.larm_avg_pos[1]]
        larm_msg.centers_z = [self.larm_avg_pos[2]]
	larm_msg.values_x = [self.larm_force[0]]
	larm_msg.values_y = [self.larm_force[1]]
	larm_msg.values_z = [self.larm_force[2]]
	norm_larm_force = self.larm_force/np.linalg.norm(self.larm_force)
	larm_msg.normals_x = [norm_larm_force[0]]
	larm_msg.normals_y = [norm_larm_force[1]]
        larm_msg.normals_z = [norm_larm_force[2]]
	self.larm_sensor_pub.publish(larm_msg)
    
    def create_rarm_messages(self):
	rarm_msg = TaxelArray()
        rarm_msg.header.frame_id = 'base_link'
	rarm_msg.link_names = ['r_forearm_roll_link']
	rarm_msg.centers_x = [self.rarm_avg_pos[0]]
	rarm_msg.centers_y = [self.rarm_avg_pos[1]]
	rarm_msg.centers_z = [self.rarm_avg_pos[2]]
	rarm_msg.values_x = [self.rarm_force[0]]
	rarm_msg.values_y = [self.rarm_force[1]]
	rarm_msg.values_z = [self.rarm_force[2]]
	norm_rarm_force = self.rarm_force/np.linalg.norm(self.rarm_force)
	rarm_msg.normals_x = [norm_rarm_force[0]]
	rarm_msg.normals_y = [norm_rarm_force[1]]
	rarm_msg.normals_z = [norm_rarm_force[2]]
	self.rarm_sensor_pub.publish(rarm_msg)

    def check_joint_limits(self):
	# joint torques can be erratic when the arm is close to joint limits
	larm_jl_max = np.radians([110.,30.,60.,80.,45.,70.])
	larm_jl_min = np.radians([-30.,-30.,0.,0.,-45.,0.])
	rarm_jl_max = np.radians([110.,30.,0.,80.,45.,70.])
	rarm_jl_min = np.radians([-30.,-30.,-60.,0.,-45.,0.])
	larm_within_jl = 1
	rarm_within_jl = 1
	#for i in range(6):
	#    if self.larm_ja[i] > larm_jl_max[i] or self.larm_ja[i] < larm_jl_min[i]:
		#print 'larm',i, self.larm_ja[i], larm_jl_max[i], larm_jl_min[i]
		#larm_within_jl = 0
	#    if self.rarm_ja[i] > rarm_jl_max[i] or self.rarm_ja[i] < rarm_jl_min[i]:
		#print 'rarm',i, self.rarm_ja[i], rarm_jl_max[i], rarm_jl_min[i]
		#rarm_within_jl = 0
	return larm_within_jl, rarm_within_jl 

    def start(self):
	rospy.sleep(1)
	while not rospy.is_shutdown():
	    try:
		self.calculate_larm_force()
		self.calculate_rarm_force()
		#print "self.larm_force: ",self.larm_force
		#print "self.rarm_force: ",self.rarm_force
		#self.calculate_jt_feedforward()
		self.create_larm_messages()
		self.create_rarm_messages()
	        rospy.sleep(.1)
	    except KeyboardInterrupt:
                self.f.close()

if __name__ == '__main__':
    s = CronaSensor()
    s.start()
