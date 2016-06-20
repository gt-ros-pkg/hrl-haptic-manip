#!/usr/bin/env python
#######################################################
# 
# cRoNA2 Raise Arm Script
# 
########################################################
# script that commands the robot joint angles to raise the arm of the ragdoll so that the robot can slide underneath
# this script is specific to a certain configuration of the ragdoll wrt to the robot
# and to a particular ragdoll size

import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
import numpy as np
import sys
from sttr_msgs.msg import CronaState, MoveJoint
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class cRoNA2RaiseArm(object):
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

    def send_goal(self,goal,time,freq,previous_goal=None):
        jerror_threshold = 50 # arbitrarily chosen for now
        r = rospy.Rate(freq)
        goal_trajectories = self.linear_interpolation_goal(goal,time,freq,previous_goal)
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

    def linear_interpolation_goal(self,goal,time,freq,previous_goal=None):
        if previous_goal == None:
            init = self.base_ja+self.torso_ja+self.head_ja+self.larm_ja+self.rarm_ja
        else:
            init = previous_goal
        goal_trajectory = []
        increment = (np.array(goal)-np.array(init))/(time*freq)
        for i in range(int(time*freq)):
            next_goal = init+(i+1)*increment
            goal_trajectory.append(next_goal.tolist())
        return goal_trajectory

    def raise_arm(self,theta,object_x,object_y):
	print "object_x: ",object_x
	print "object_y: ",object_y
        self.initialize_msg()
	# Move towards ragdoll so that right arm is does not run into hand
	offset_x = -1.4
	offset_y = -0.025
	### Offset for new ragdoll
	#offset_x = -1.35
	#offset_y = -0.125
	###
	world_offset_x = math.cos(theta)*offset_x-math.sin(theta)*offset_y
        world_offset_y = math.sin(theta)*offset_x+math.cos(theta)*offset_y
	self.base_goal = [object_x+world_offset_x,object_y+world_offset_y,self.base_ja[2]]
	#self.base_goal = [self.base_ja[0]+self.base_goal[0],self.base_ja[1]+self.base_goal[1],np.degrees(theta)]
	print "base_goal: ",self.base_goal
	print "object_y: ",object_y
	#self.base_goal = [3.6, -0.2, self.base_ja[2]]
        self.set_pose = self.base_goal+self.torso_ja+self.head_ja+self.larm_ja+self.rarm_ja
        self.send_goal(self.set_pose,2,100)
	# Raise ragdoll's arm so that the right arm can slide under
	self.larm_goal = [105.-self.l_shoulder_pan/10.,self.l_shoulder_pan/4.,self.l_shoulder_pan,75.,self.l_forearm_roll,25.]
	self.rarm_goal = [105.-self.l_shoulder_pan/10.,self.r_shoulder_pan/4.,self.r_shoulder_pan,75.,self.r_forearm_roll,-20.]
	self.set_pose = self.base_ja+self.torso_ja+self.head_ja+self.larm_goal+self.rarm_goal
        self.send_goal(self.set_pose,2,100)
	# Move crona sideways so that right arm is under the ragdoll arm
	offset_x = -1.4
	offset_y = 0.175
	### Offset for new ragdoll
	#offset_x = -1.35
	#offset_y = 0.075
	###
	world_offset_x = math.cos(theta)*offset_x-math.sin(theta)*offset_y
        world_offset_y = math.sin(theta)*offset_x+math.cos(theta)*offset_y
        self.base_goal = [object_x+world_offset_x,object_y+world_offset_y,self.base_ja[2]]
	#self.base_goal = [self.base_ja[0]+self.base_goal[0],self.base_ja[1]+self.base_goal[1],np.degrees(theta)]
	#self.base_goal = [3.6, 0., self.base_ja[2]]
        self.set_pose = self.base_goal+self.torso_ja+self.head_ja+self.larm_ja+self.rarm_ja
        self.send_goal(self.set_pose,2,100)
	print "base_goal: ",self.base_goal
	# Lower ragdoll's arm
	self.larm_goal = [105.-self.l_shoulder_pan/10.,self.l_shoulder_pan/4.,self.l_shoulder_pan,75.,self.l_forearm_roll,-20.]
        self.rarm_goal = [105.-self.l_shoulder_pan/10.,self.r_shoulder_pan/4.,self.r_shoulder_pan,75.,self.r_forearm_roll,-20.]
        self.set_pose = self.base_ja+self.torso_ja+self.head_ja+self.larm_goal+self.rarm_goal
        self.send_goal(self.set_pose,2,100)
	
    def start(self,l_shoulder_pan=0.,r_shoulder_pan=0.,l_forearm_roll=0.,r_forearm_roll=0.,l_hand_angle=0.,r_hand_angle=0.,theta=0.,object_x=5.,object_y=-0.175):
    ### Offset for new ragdoll
    #def start(self,l_shoulder_pan=0.,r_shoulder_pan=0.,l_forearm_roll=0.,r_forearm_roll=0.,l_hand_angle=0.,r_hand_angle=0.,theta=0.,object_x=5.,object_y=-0.0784):
    ### 
	self.l_shoulder_pan = l_shoulder_pan
        self.r_shoulder_pan = r_shoulder_pan
        self.l_forearm_roll = l_forearm_roll
        self.r_forearm_roll = r_forearm_roll
        self.l_hand_angle = l_hand_angle
        self.r_hand_angle = r_hand_angle
	rospy.sleep(1)
	for i in range(5):
            self.scripted_behaviors_publishing.publish(1)
            self.base_control_pub.publish(1)
	self.raise_arm(theta,object_x,object_y)
	for i in range(5):
            self.scripted_behaviors_publishing.publish(0)
            self.base_control_pub.publish(0)

if __name__ == '__main__':
    rospy.init_node('crona2_raise_arm')
    c2ra = cRoNA2RaiseArm()
    if len(sys.argv) > 1:
    	c2ra.start(float(sys.argv[1]),-float(sys.argv[1]),-float(sys.argv[1]),float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[2]))
    else:
    	c2ra.start()
