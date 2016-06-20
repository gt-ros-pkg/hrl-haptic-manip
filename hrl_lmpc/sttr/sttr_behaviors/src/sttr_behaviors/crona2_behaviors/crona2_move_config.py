#!/usr/bin/env python
#######################################################
# 
# cRoNA2 Move Config Script
# 
########################################################
# script that commands robot joint angles to move the robot into a moving configuration

import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
import numpy as np
import sys
from sttr_msgs.msg import CronaState, MoveJoint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class cRoNA2MoveConfig(object):
    def __init__(self):
	rospy.Subscriber('crona/sim_state',CronaState,self.update_state)
	
	self.move = rospy.Publisher('crona/command',MoveJoint)
	self.larm_mpc_move = rospy.Publisher('/l_arm/haptic_mpc/joint_trajectory',JointTrajectory)
	self.rarm_mpc_move = rospy.Publisher('/r_arm/haptic_mpc/joint_trajectory',JointTrajectory)
	self.torso_move = rospy.Publisher('/crona_torso_controller/command',JointTrajectory)
	self.base_move = rospy.Publisher('/crona_base_controller/command',JointTrajectory)

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
	
    def joint_error(self,goal_trajectory):
	current_ja = self.base_ja+self.torso_ja+self.head_ja+self.larm_ja+self.rarm_ja
	error = (np.array(current_ja)-np.array(goal_trajectory)).tolist()
	return error

    def initialize_msg(self):
	self.msg = MoveJoint()
	self.msg.goals = [0.,0.,0.,0.,0.,0.,0.,0.,-45.,0.,0.,0.,0.,0.,45.,0.,0.]
	self.home_pose = [0.,0.,0.,0.,0.,0.,0.,0.,-45.,0.,0.,0.,0.,0.,45.,0.,0.]
	self.msg.names = rospy.get_param('/crona_base_controller/joints')+rospy.get_param('/crona_torso_controller/joints')+rospy.get_param('/crona_head_controller/joints')+rospy.get_param('l_arm_controller/joints')+rospy.get_param('r_arm_controller/joints')
	self.msg.time = 0.

    def create_msg(self,goal,time):
	self.msg.goals = goal
	self.msg.time = time

    def send_goal(self,goal,time,freq):
	jerror_threshold = 2000 # arbitrarily chosen for now
	r = rospy.Rate(freq)
	goal_trajectories = self.linear_interpolation_goal(goal,time,freq)
	for i in range(len(goal_trajectories)):
	    self.create_msg(goal_trajectories[i],1./freq)
	    self.move.publish(self.msg)
	    r.sleep()
	    jerror_cond = np.linalg.norm(np.array(self.joint_error(goal_trajectories[i]))) > jerror_threshold
	    while jerror_cond:
		self.move.publish(self.msg)
		r.sleep()
		print 'wait for joint error small than threshold...', 'err=',  np.linalg.norm(np.array(self.joint_error(goal_trajectories[i])))
	        jerror_cond = np.linalg.norm(np.array(self.joint_error(goal_trajectories[i]))) > jerror_threshold
        return True

    def linear_interpolation_goal(self,goal,time,freq):
	init = self.base_ja+self.torso_ja+self.head_ja+self.larm_ja+self.rarm_ja
	goal_trajectory = []
	increment = (np.array(goal)-np.array(init))/(time*freq)
	for i in range(int(time*freq)):
	    next_goal = init+(i+1)*increment
	    goal_trajectory.append(next_goal.tolist())
	return goal_trajectory

    def move_config(self):
        self.initialize_msg()
	#self.torso_goal = [-100.]
	#self.torso_goal = [-95.]
	self.torso_goal = [-100.-self.l_shoulder_pan/10.]
	self.head_goal = [90.]
	#self.larm_goal = [115.-self.l_shoulder_pan/10.,self.l_shoulder_pan/4.,self.l_shoulder_pan,75.,self.l_forearm_roll,self.l_hand_angle]
	self.larm_goal = [105.-self.l_shoulder_pan/10.,self.l_shoulder_pan/4.,self.l_shoulder_pan,80.,self.l_forearm_roll,45.]
	#self.rarm_goal = [115.-self.l_shoulder_pan/10.,self.r_shoulder_pan/4.,self.r_shoulder_pan,75.,self.r_forearm_roll,self.r_hand_angle]
	self.rarm_goal = [105.-self.l_shoulder_pan/10.,self.r_shoulder_pan/4.,self.r_shoulder_pan,80.,self.r_forearm_roll,45.]
	self.set_pose = self.base_ja+self.torso_goal+self.head_goal+self.larm_goal+self.rarm_goal
        self.send_goal(self.set_pose,5,10)
	
    def start(self,l_shoulder_pan=0.,r_shoulder_pan=0.,l_forearm_roll=0.,r_forearm_roll=0.,l_hand_angle=0.,r_hand_angle=0.):
	self.l_shoulder_pan = l_shoulder_pan
        self.r_shoulder_pan = r_shoulder_pan
        self.l_forearm_roll = l_forearm_roll
        self.r_forearm_roll = r_forearm_roll
        self.l_hand_angle = l_hand_angle
        self.r_hand_angle = r_hand_angle
	rospy.sleep(1)
	self.lower_down()

if __name__ == '__main__':
    rospy.init_node('crona2_move_config')
    c2mc = cRoNA2MoveConfig()
    if len(sys.argv) > 1:
    	c2mc.start(float(sys.argv[1]),-float(sys.argv[1]),-float(sys.argv[1]),float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[2]))
    else:
    	c2mc.start()
