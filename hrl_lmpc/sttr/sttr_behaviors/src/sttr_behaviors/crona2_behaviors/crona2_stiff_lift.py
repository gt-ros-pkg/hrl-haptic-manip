#!/usr/bin/env python
#######################################################
# 
# cRoNA2 Lift Script
# 
########################################################
# script that commands the robot joint angles to lift a body 
# stiff lift means that the JEP are larger than the regular lift, so this will lift the ragdoll to a higher position

import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
import numpy as np
from sttr_msgs.msg import CronaState, MoveJoint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class cRoNA2Lift(object):
    def __init__(self):
	rospy.Subscriber('crona/sim_state',CronaState,self.update_state)
	
	self.move = rospy.Publisher('crona/command',MoveJoint)
	self.larm_mpc_move = rospy.Publisher('/l_arm/haptic_mpc/joint_trajectory',JointTrajectory)
	self.rarm_mpc_move = rospy.Publisher('/r_arm/haptic_mpc/joint_trajectory',JointTrajectory)
	self.torso_move = rospy.Publisher('/crona_torso_controller/command',JointTrajectory)
	self.base_move = rospy.Publisher('/crona_base_controller/command',JointTrajectory)

    def update_state(self,msg):
	self.base_ja = [msg.position[0],msg.position[1],msg.position[2]]	
	self.torso_ja = [np.degrees(msg.position[3]).tolist()]
	self.head_ja = [np.degrees(msg.position[4]).tolist()]
	self.larm_ja = np.degrees(msg.position[5:11]).tolist()
	self.rarm_ja = np.degrees(msg.position[11:17]).tolist()

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
	jerror_threshold = 30 # arbitrarily chosen for now
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

    def lift(self):
	self.initialize_msg()
	self.torso_goal = [0.]
	self.head_goal = [0.]
	self.larm_goal = [105.,0.,self.l_shoulder_pan,85.,self.l_forearm_roll,45.]
        self.rarm_goal = [105.,0.,self.r_shoulder_pan,85.,self.r_forearm_roll,45.]
        self.set_pose = self.base_ja+self.torso_ja+self.head_ja+self.larm_goal+self.rarm_goal
        self.send_goal(self.set_pose,5,10)
	self.larm_goal = [45.,0.,self.l_shoulder_pan,55.,self.l_forearm_roll,45.]
        self.rarm_goal = [45.,0.,self.r_shoulder_pan,55.,self.r_forearm_roll,45.]
	self.set_pose = self.base_ja+self.torso_goal+self.head_goal+self.larm_goal+self.rarm_goal
        self.send_goal(self.set_pose,15,10)
	
    def start(self,l_shoulder_pan=0.,r_shoulder_pan=0.,l_forearm_roll=0.,r_forearm_roll=0.):
	self.l_shoulder_pan = l_shoulder_pan
	self.r_shoulder_pan = r_shoulder_pan
	self.l_forearm_roll = l_forearm_roll
	self.r_forearm_roll = r_forearm_roll
	rospy.sleep(1)
	self.lift()

if __name__ == '__main__':
    rospy.init_node('crona2_lift')
    c2l = cRoNA2Lift()
    c2l.start()
