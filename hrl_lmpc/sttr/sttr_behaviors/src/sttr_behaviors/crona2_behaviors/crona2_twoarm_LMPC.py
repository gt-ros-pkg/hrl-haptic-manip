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
# cRoNA2 Lift Script
# 
########################################################
# send goals to the LMPC 

import sys
import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
import numpy as np
from sttr_msgs.msg import CronaState, MoveJoint, RagdollObjectArray, GoalPose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from hrl_msgs.msg import *
from math import *
from LMPC_msgs.msg import *
import csv

class cRoNA2Lift(object):
    def __init__(self):
	self.circle_drawing_status = False
	rospy.Subscriber('crona/sim_state',CronaState,self.update_state)
	rospy.Subscriber('/haptic_mpc/deadzone',Bool,self.deadzone_callback)
	rospy.Subscriber('/haptic_mpc/dist_to_goal',FloatArray,self.dist_to_goal_callback)
	rospy.Subscriber('/haptic_mpc/robot_state',RobotHapticState,self.ee_position_callback)
	
	self.move = rospy.Publisher('crona/command',MoveJoint)
	self.larm_mpc_move = rospy.Publisher('/l_arm/haptic_mpc/goal_posture',FloatArray)
	self.larm_mpc_cart_move = rospy.Publisher('/l_arm/haptic_mpc/traj_pose',PoseStamped)
	self.mpc_cart_move = rospy.Publisher('/haptic_mpc/traj_pose',GoalPose)
        self.rarm_mpc_move = rospy.Publisher('/r_arm/haptic_mpc/goal_posture',FloatArray)
	self.torso_move = rospy.Publisher('/crona_torso_controller/command',JointTrajectory)
	self.base_move = rospy.Publisher('/crona_base_controller/command',JointTrajectory)
	self.goal_status_pub = rospy.Publisher('crona/goal_status',FloatArray)

	#self.dist_to_goal = open('LMPC_dtg_lower_down','w')
	#self.circle_drawing = open('LMPC_circle_drawing','w')
	#self.controller_test_data = open('slb_'+sys.argv[1]+'_'+sys.argv[2]+'_'+sys.argv[3]+'_'+sys.argv[4]+'_'+sys.argv[5], 'w')
	#self.csv_circle_drawing = csv.writer(self.circle_drawing)
	#self.csv_dist_to_goal = csv.writer(self.dist_to_goal)
	#self.csv_controller_test_data = csv.writer(self.controller_test_data)
        self.larm_dist_to_goal = 0
        self.rarm_dist_to_goal = 0
        self.base_torso_dist_to_goal = 0
	self.deadzone = False

    def ee_position_callback(self,msg):
	if self.circle_drawing_status == True:
	    self.csv_circle_drawing.writerow([msg.l_hand_pose.position.x,msg.l_hand_pose.position.y,msg.l_hand_pose.position.z,msg.r_hand_pose.position.x,msg.r_hand_pose.position.y,msg.r_hand_pose.position.z])

    def dist_to_goal_callback(self,msg):
	if self.circle_drawing_status == True:
	    self.csv_dist_to_goal.writerow(msg.data)	

    def deadzone_callback(self,msg):
	self.deadzone = msg.data

    def update_state(self,msg):
	self.base_ja = [msg.desired_position[0],msg.desired_position[1],msg.desired_position[2]]	
	self.torso_ja = [np.degrees(msg.desired_position[3]).tolist()]
	self.head_ja = [np.degrees(msg.desired_position[4]).tolist()]
	self.larm_ja = np.degrees(msg.desired_position[5:11]).tolist()
	self.rarm_ja = np.degrees(msg.desired_position[11:17]).tolist()

	self.base_je = [msg.effort[0],msg.effort[1],msg.effort[2]]
	self.torso_je = [msg.effort[3]]
	self.head_je = [msg.effort[4]]
	self.larm_je = [msg.effort[5],msg.effort[6],msg.effort[7],msg.effort[8],msg.effort[9],msg.effort[10]]
	self.rarm_je = [msg.effort[11],msg.effort[12],msg.effort[13],msg.effort[14],msg.effort[15],msg.effort[16]]
	
    def base_torso_joint_error(self,goal_trajectory):
        current_ja = self.base_ja+self.torso_ja+self.head_ja
        error = (np.array(current_ja)-np.array(goal_trajectory)).tolist()
        return error

    def initialize_msg(self):
        self.msg = MoveJoint()
        self.msg.goals = [0.,0.,0.,0.,0.]
        self.home_pose = [0.,0.,0.,0.,0.]
        self.msg.names = rospy.get_param('/crona_base_controller/joints')+rospy.get_param('/crona_torso_controller/joints')+rospy.get_param('/crona_head_controller/joints')
        self.msg.time = 0.

    def create_msg(self,goal,time):
	self.msg.goals = goal
	self.msg.time = time

    def send_goal(self,base_torso_goal,time,freq):
	jerror_threshold = 0.4 # arbitrarily chosen for now
        r = rospy.Rate(freq)
        goal_trajectories = self.linear_interpolation_goal(base_torso_goal,time,freq)
        for i in range(len(goal_trajectories)):
            self.create_msg(goal_trajectories[i],1./freq)
            self.goal_status()
            self.move.publish(self.msg)
            r.sleep()
            jerror_cond = max(self.larm_dist_to_goal,self.rarm_dist_to_goal)-self.base_torso_dist_to_goal > jerror_threshold
            #print "jerror_cond: ",max(np.linalg.norm(self.larm_joint_error),np.linalg.norm(self.rarm_joint_error))-np.linalg.norm(np.array(self.base_torso_joint_error(base_torso_goal)))
            while jerror_cond:
                self.goal_status()
                self.move.publish(self.msg)
                r.sleep()
                print 'wait for joint error small than threshold...', 'err=', max(self.larm_dist_to_goal,self.rarm_dist_to_goal)-self.base_torso_dist_to_goal
                jerror_cond = max(self.larm_dist_to_goal,self.rarm_dist_to_goal)-self.base_torso_dist_to_goal > jerror_threshold
        return True

    def linear_interpolation_goal(self,goal,time,freq):
        init = self.base_ja+self.torso_ja+self.head_ja
        goal_trajectory = []
        increment = (np.array(goal)-np.array(init))/(time*freq)
        for i in range(int(time*freq)):
            next_goal = init+(i+1)*increment
            goal_trajectory.append(next_goal.tolist())
        return goal_trajectory

    def goal_status(self,torso=1):
        self.larm_dist_to_goal = np.linalg.norm(np.array(self.larm_goal)-np.array(self.larm_ja))/np.linalg.norm(np.array(self.larm_goal)-np.array(self.larm_init))
        self.rarm_dist_to_goal = np.linalg.norm(np.array(self.rarm_goal)-np.array(self.rarm_ja))/np.linalg.norm(np.array(self.rarm_goal)-np.array(self.rarm_init))
        if torso == 1:
	    self.base_torso_dist_to_goal = np.linalg.norm(np.array(self.base_torso_goal)-np.array(self.base_ja+self.torso_ja+self.head_ja))/np.linalg.norm(np.array(self.base_torso_goal)-np.array(self.base_torso_init))
	else:
	    self.base_torso_dist_to_goal = 0
        fa = FloatArray()
        fa.data = [self.base_torso_dist_to_goal,self.larm_dist_to_goal,self.rarm_dist_to_goal]
        print "goal_status: ",fa.data
        self.goal_status_pub.publish(fa)

    def lift(self):
	self.initialize_msg()
	self.torso_goal = [0.]
	self.head_goal = [0.]
	self.larm_goal = [65.,0.,self.l_shoulder_pan,85.,self.l_forearm_roll,45.]
        self.rarm_goal = [65.,0.,self.r_shoulder_pan,85.,self.r_forearm_roll,45.]
	
	self.mpc_goal = [-45.,65.,0.,self.l_shoulder_pan,85.,self.l_forearm_roll,45.]

	self.base_torso_goal = self.base_ja+self.torso_goal+self.head_goal
        self.base_torso_init = self.base_ja+self.torso_ja+self.head_ja
        self.larm_init = self.larm_ja
        self.rarm_init = self.rarm_ja
	fa_l = FloatArray()
        fa_r = FloatArray()
        fa_l.data = np.radians(self.larm_goal).tolist()
        fa_r.data = np.radians(self.rarm_goal).tolist()
	fa = FloatArray()
	fa.data = np.radians(self.mpc_goal).tolist()

	goal_pose_msg = GoalPose()
        goal_pose_msg.header.frame_id = 'base_link'

	#object_pose
	goal_pose_msg.object_pose.position.x = 0.79
        goal_pose_msg.object_pose.position.y = 0.
        goal_pose_msg.object_pose.position.z = 0.607
	goal_pose_msg.object_pose.orientation.x = 0.
        goal_pose_msg.object_pose.orientation.y = 0.
        goal_pose_msg.object_pose.orientation.z = 0.
        goal_pose_msg.object_pose.orientation.w = 1.

  	goal_pose_msg.lforearm_pose.position.x = 0.79	
  	goal_pose_msg.lforearm_pose.position.y = 0.265	
  	goal_pose_msg.lforearm_pose.position.z = 0.607	
  	goal_pose_msg.lforearm_pose.orientation.x = 0.	
  	goal_pose_msg.lforearm_pose.orientation.y = -0.707	
  	goal_pose_msg.lforearm_pose.orientation.z = 0.	
  	goal_pose_msg.lforearm_pose.orientation.w = 0.707	

    	goal_pose_msg.rforearm_pose.position.x = 0.79
        goal_pose_msg.rforearm_pose.position.y = -0.265
        goal_pose_msg.rforearm_pose.position.z = 0.607
        goal_pose_msg.rforearm_pose.orientation.x = 0.  
        goal_pose_msg.rforearm_pose.orientation.y = -0.707
        goal_pose_msg.rforearm_pose.orientation.z = 0.  
        goal_pose_msg.rforearm_pose.orientation.w = 0.707

        #larm_ps.pose.orientation.x = 0.5
        #larm_ps.pose.orientation.y = 0.5
        #larm_ps.pose.orientation.z = 0.5
        #larm_ps.pose.orientation.w = 0.5
	#test
	#larm_ps.pose.orientation.x = 0.123
        #larm_ps.pose.orientation.y = -0.696
        #larm_ps.pose.orientation.z = -0.123
        #larm_ps.pose.orientation.w = 0.697


	## -10 degrees
        #larm_ps.pose.orientation.x = 0.
        #larm_ps.pose.orientation.y = -0.643
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.766
        ## -20 degrees
        #larm_ps.pose.orientation.x = 0.
        #larm_ps.pose.orientation.y = -0.574
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.819
        ## -30 degrees
        #larm_ps.pose.orientation.x = 0.
        #larm_ps.pose.orientation.y = -0.500
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.866
  	#larm_ps.pose.orientation.x = 0.
        #larm_ps.pose.orientation.y = -0.866
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.5
        ## -40 degrees
        #larm_ps.pose.orientation.x = 0.
        #larm_ps.pose.orientation.y = -0.423
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.906
        ## -50 degrees
        #larm_ps.pose.orientation.x = 0.
        #larm_ps.pose.orientation.y = -0.342
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.940

	# ragdoll-centered orientations
	#larm_ps.pose.orientation.x = 0.
        #larm_ps.pose.orientation.y = 0.
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 1.
	## test 1 (10 degree pitch)
	#larm_ps.pose.orientation.x = 0.
        #larm_ps.pose.orientation.y = 0.087
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.996
	## test 2 (30 degree pitch)
	#larm_ps.pose.orientation.x = 0.
        #larm_ps.pose.orientation.y = 0.259
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.966
	## test 3 (10 degree roll)
        #larm_ps.pose.orientation.x = 0.087
        #larm_ps.pose.orientation.y = 0.
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.996
        ## test 4 (30 degree roll)
        #larm_ps.pose.orientation.x = 0.259
        #larm_ps.pose.orientation.y = 0.
        #larm_ps.pose.orientation.z = 0.
        #larm_ps.pose.orientation.w = 0.966

	#rospy.sleep(5)
 	for i in range(100):
            self.mpc_cart_move.publish(goal_pose_msg)

        '''
	larm_ps = PoseStamped()
	larm_ps.header.frame_id = 'base_link'
	theta = 10
	r = 0.1
	larm_ps.pose.orientation.x = 0.
        larm_ps.pose.orientation.y = -0.924
        larm_ps.pose.orientation.z = 0.
        larm_ps.pose.orientation.w = 0.383
	# move arm in circles
	raw_input()
	for i in range(3):
	    self.circle_drawing_status = True
	    while (theta % 360) != 0:
		print "theta: ",theta
	    	x = r*sin(np.radians(theta))
		y = r*cos(np.radians(theta))
		larm_ps.pose.position.x = 0.5
        	larm_ps.pose.position.y = 0.27+x
        	larm_ps.pose.position.z = 0.46+y
		for i in range(2):
            	    self.mpc_cart_move.publish(larm_ps)
		rospy.sleep(1)
		n = 0
		while self.deadzone == False and n != 20:
		   n += 1
		   if theta == 10:
		       rospy.sleep(0.1)
		   print "n: ",n
		theta += 10
	self.circle_drawing_status = False
		
	'''	
    
    def start(self,l_shoulder_pan=0.,r_shoulder_pan=0.,l_forearm_roll=0.,r_forearm_roll=0.):
	self.l_shoulder_pan = l_shoulder_pan
	self.r_shoulder_pan = r_shoulder_pan
	self.l_forearm_roll = l_forearm_roll
	self.r_forearm_roll = r_forearm_roll
	rospy.sleep(1)
	self.lift()

class Terminate(object):
    def __init__(self):
 	self.controller_test_data = open('trial_'+sys.argv[9]+'_results', 'wb')
	self.csv_controller_test_data = csv.writer(self.controller_test_data)
	self.mpc_terminate = rospy.Publisher('/haptic_mpc/terminate',Bool)
	rospy.sleep(2)
    	rospy.Subscriber('/haptic_mpc/deadzone',Bool,self.dz_callback)
    	rospy.Subscriber('/haptic_mpc/dist_to_goal',FloatArray,self.dtg_callback)
    	rospy.Subscriber('/link1_tactile_sensor/taxels/forces',TaxelArray,self.link1_force_callback)
    	#rospy.Subscriber('/gazebo/objectcog',RagdollObjectArray,self.ragdoll_pose_callback)
	self.terminate = 0
	self.dtg = [0,0]
	t_elasped = 0
	self.max_force = 0
	start_t = rospy.get_time()
	while self.terminate == 0 and t_elasped < 45:
	    t_elasped = rospy.get_time()-start_t
	    #print "t_elasped: ",t_elasped
	    pass
	self.csv_controller_test_data.writerow([self.terminate,t_elasped,self.max_force,self.dtg[0],self.dtg[1],sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4],sys.argv[5],sys.argv[6],sys.argv[7],sys.argv[8],sys.argv[9]])
	#self.csv_controller_test_data.writerow([self.terminate,t_elasped,self.max_force,self.dtg[0],self.dtg[1],sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4],sys.argv[5],sys.argv[6],sys.argv[7],sys.argv[8],sys.argv[9],self.ragdoll_m1_x,self.ragdoll_m1_y,self.ragdoll_m1_z,self.ragdoll_m1_rot_x,self.ragdoll_m1_rot_y,self.ragdoll_m1_rot_z,self.ragdoll_m1_rot_w,self.ragdoll_m2_x,self.ragdoll_m2_y,self.ragdoll_m2_z,self.ragdoll_m2_rot_x,self.ragdoll_m2_rot_y,self.ragdoll_m2_rot_z,self.ragdoll_m2_rot_w,self.ragdoll_m3_x,self.ragdoll_m3_y,self.ragdoll_m3_z,self.ragdoll_m3_rot_x,self.ragdoll_m3_rot_y,self.ragdoll_m3_rot_z,self.ragdoll_m3_rot_w])
	#print "t_elasped: ",t_elasped
	self.controller_test_data.close()
	self.mpc_terminate.publish(1)

    def dtg_callback(self,msg):
	self.dtg = [msg.data[0],msg.data[1]] 	
 
    def dz_callback(self,msg):
	self.terminate = msg.data

    def link1_force_callback(self,msg):
	if msg.values_x[0] != []:
	    vx = []
	    vy = []
	    vz = []
	    n = []
	    for i in range(len(msg.normals_x)):
		n_new = np.array((msg.normals_x[i],msg.normals_y[i],msg.normals_z[i]))
		if i == 0:
		    vx.append(msg.values_x[i])
		    vy.append(msg.values_y[i])
		    vz.append(msg.values_z[i])
		    n.append(np.array((msg.normals_x[i],msg.normals_y[i],msg.normals_z[i])))
		else:
		    for j in range(len(n)):
			if np.dot(n_new,n[j]) > 0.8:
			    vx[j] += msg.values_x[i]
			    vy[j] += msg.values_y[i]
			    vz[j] += msg.values_z[i]
			else:
			    vx.append(msg.values_x[i])
			    vy.append(msg.values_y[i])
			    vz.append(msg.values_z[i])
			    n.append(np.array((msg.normals_x[i],msg.normals_y[i],msg.normals_z[i])))
	    for i in range(len(vx)):
		self.max_force = max(self.max_force,np.linalg.norm((vx[i],vy[i],vz[i]))) 

    def ragdoll_pose_callback(self,msg):
        for i in range(len(msg.frame_names)):
	    if msg.frame_names[i] == 'ragdoll_m1':
		self.ragdoll_m1_x = msg.centers_x[i]
		self.ragdoll_m1_y = msg.centers_y[i]
		self.ragdoll_m1_z = msg.centers_z[i]
		self.ragdoll_m1_rot_x = msg.rotation_x[i]
		self.ragdoll_m1_rot_y = msg.rotation_y[i]
		self.ragdoll_m1_rot_z = msg.rotation_z[i]
		self.ragdoll_m1_rot_w = msg.rotation_w[i]
	    elif msg.frame_names[i] == 'ragdoll_m2':
                self.ragdoll_m2_x = msg.centers_x[i]
                self.ragdoll_m2_y = msg.centers_y[i]
                self.ragdoll_m2_z = msg.centers_z[i]
                self.ragdoll_m2_rot_x = msg.rotation_x[i]
                self.ragdoll_m2_rot_y = msg.rotation_y[i]
                self.ragdoll_m2_rot_z = msg.rotation_z[i]
                self.ragdoll_m2_rot_w = msg.rotation_w[i]
            elif msg.frame_names[i] == 'ragdoll_m3':
                self.ragdoll_m3_x = msg.centers_x[i]
                self.ragdoll_m3_y = msg.centers_y[i]
                self.ragdoll_m3_z = msg.centers_z[i]
                self.ragdoll_m3_rot_x = msg.rotation_x[i]
                self.ragdoll_m3_rot_y = msg.rotation_y[i]
                self.ragdoll_m3_rot_z = msg.rotation_z[i]
                self.ragdoll_m3_rot_w = msg.rotation_w[i]

if __name__ == '__main__':
    rospy.sleep(1)
    rospy.init_node('crona2_lift')
    c2l = cRoNA2Lift()
    c2l.start()
    #t = Terminate()
    
