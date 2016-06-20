#!/usr/bin/env python
#######################################################
# 
# cRoNA2 Release Object Script
# 
########################################################
# script that commands the robot through joint control to slide its arms from underneath the ragdoll body
# specific to the crona2 final simulation 

import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
import numpy as np
from sttr_msgs.msg import CronaState, MoveJoint
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class cRoNA2ReleaseObject(object):
    def __init__(self):
	rospy.Subscriber('crona/sim_state',CronaState,self.update_state)
	
	self.move = rospy.Publisher('crona/command',MoveJoint)
	self.larm_mpc_move = rospy.Publisher('/l_arm/haptic_mpc/joint_trajectory',JointTrajectory)
	self.rarm_mpc_move = rospy.Publisher('/r_arm/haptic_mpc/joint_trajectory',JointTrajectory)
	self.torso_move = rospy.Publisher('/crona_torso_controller/command',JointTrajectory)
	self.base_move = rospy.Publisher('/crona_base_controller/command',JointTrajectory)
	self.scripted_behaviors_publishing = rospy.Publisher('haptic_mpc/scripted_behaviors',Bool)
	self.base_control_pub = rospy.Publisher('/crona2_server/base_control',Bool)

    def update_state(self,msg):
	self.base_ja = [msg.desired_position[0],msg.desired_position[1],np.degrees(msg.desired_position[2]).tolist()]	
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
	    #print "self.msg: ",self.msg
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

    def release_object(self):
        self.initialize_msg()
	self.base_goal = [self.base_ja[0],self.base_ja[1],self.base_ja[2]]
	#self.base_goal = [-0.6,0.8,90]
	#self.torso_goal = [-91.]
	self.torso_goal = [-97.]
	self.head_goal = [90.]
	self.larm_goal = [105.,0.,0.,75.,0.,-20.]
	self.rarm_goal = [105.,0.,0.,75.,0.,-20.]
	self.set_pose = self.base_goal+self.torso_goal+self.head_goal+self.larm_goal+self.rarm_goal
        self.send_goal(self.set_pose,5,100)
	
	self.base_goal = [self.base_ja[0],self.base_ja[1],self.base_ja[2]]
        self.torso_goal = [-70.]
        self.head_goal = [90.]
        #self.larm_goal = [105.,0.,0.,55.,0.,-20.]
        #self.rarm_goal = [105.,0.,0.,55.,0.,-20.]
        self.larm_goal = [95.,0.,0.,55.,0.,-20.]
        self.rarm_goal = [95.,0.,0.,55.,0.,-20.]
        self.set_pose = self.base_goal+self.torso_goal+self.head_goal+self.larm_goal+self.rarm_goal
        self.send_goal(self.set_pose,2,100)
        self.base_goal = [self.base_ja[0],self.base_ja[1]-0.6,self.base_ja[2]]
        #self.torso_goal = [-85.]
        self.torso_goal = [-97.]
        self.head_goal = [90.]
        self.larm_goal = [105.,0.,0.,65.,0.,-20.]
        self.rarm_goal = [105.,0.,0.,65.,0.,-20.]
        self.set_pose = self.base_goal+self.torso_goal+self.head_goal+self.larm_goal+self.rarm_goal
        self.send_goal(self.set_pose,2,100)
	
    def start(self):
	rospy.sleep(1)
	for i in range(5):
            self.scripted_behaviors_publishing.publish(1)
	    self.base_control_pub.publish(1)
	self.release_object()
	for i in range(5):
            self.scripted_behaviors_publishing.publish(0)
	    self.base_control_pub.publish(0)

if __name__ == '__main__':
    rospy.init_node('crona2_move_towards_object')
    c2ro = cRoNA2ReleaseObject()
    c2ro.start()
