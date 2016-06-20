#!/usr/bin/env python
#######################################################
# 
# cRoNA Roll simulation 
# 
########################################################
# NOT FINISHED
# script that is intended to allow the crona2 to roll a body over

import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
import numpy as np
from sttr_msgs.msg import CronaState, MoveJoint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from initGazebocRoNASim import Objects
#from hrl_haptic_manipulation_in_clutter_msgs.msg import RobotHapticState

class cRoNA2Roll(object):
    def __init__(self):
        rospy.init_node('crona_setup')
	rospy.Subscriber('crona/sim_state',CronaState,self.update_state)
	rospy.Subscriber('crona/sim_state',CronaState,self.update_state)
	
	self.move = rospy.Publisher('crona/command',MoveJoint)
	self.larm_mpc_move = rospy.Publisher('/l_arm/haptic_mpc/joint_trajectory',JointTrajectory)
	self.rarm_mpc_move = rospy.Publisher('/r_arm/haptic_mpc/joint_trajectory',JointTrajectory)
	self.torso_move = rospy.Publisher('/crona_torso_controller/command',JointTrajectory)
	self.base_move = rospy.Publisher('/crona_base_controller/command',JointTrajectory)

    def update_state(self,msg):
	self.base_ja = [msg.position[0],msg.position[1],msg.position[2]]	
	self.torso_ja = [np.degrees(msg.position[3]).tolist()]
	self.larm_ja = np.degrees(msg.position[4:10]).tolist()
	self.rarm_ja = np.degrees(msg.position[10:16]).tolist()

	self.base_je = [msg.effort[0],msg.effort[1],msg.effort[2]]
	self.torso_je = [msg.effort[3]]
	self.larm_je = [msg.effort[4],msg.effort[5],msg.effort[6],msg.effort[7],msg.effort[8],msg.effort[9]]
	self.rarm_je = [msg.effort[10],msg.effort[11],msg.effort[12],msg.effort[13],msg.effort[14],msg.effort[15]]
	
    def joint_error(self,goal_trajectory):
	current_ja = self.base_ja+self.torso_ja+self.larm_ja+self.rarm_ja
	error = (np.array(current_ja)-np.array(goal_trajectory)).tolist()
	return error

    def initialize_msg(self):
	self.msg = MoveJoint()
	self.msg.goals = [0.,0.,0.,0.,0.,0.,0.,-45.,0.,0.,0.,0.,0.,45.,0.,0.]
	self.home_pose = [0.,0.,0.,0.,0.,0.,0.,-45.,0.,0.,0.,0.,0.,45.,0.,0.]
	self.msg.names = rospy.get_param('/crona_base_controller/joints')+rospy.get_param('/crona_torso_controller/joints')+rospy.get_param('l_arm_controller/joints')+rospy.get_param('r_arm_controller/joints')
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
	init = self.msg.goals
	goal_trajectory = []
	increment = (np.array(goal)-np.array(init))/(time*freq)
	for i in range(int(time*freq)):
	    next_goal = init+(i+1)*increment
	    goal_trajectory.append(next_goal.tolist())
	return goal_trajectory

    def setup_crona(self):
        #self.initialize_msg()
	#self.send_goal(self.home_pose,5,5)
	self.base_goal = [3.3,0.,0.]
	self.torso_goal = np.radians([-90.]).tolist()
	self.larm_goal = np.radians([-87.,0.,0.,-85.,0.,-50.]).tolist()
	self.rarm_goal = np.radians([87.,0.,0.,85.,0.,50.]).tolist()
	#self.set_pose = self.home_pose[0:3]+self.torso_goal+self.larm_goal+self.rarm_goal
        #self.send_goal(self.set_pose,5,100)
	larm_msg = JointTrajectory()
        larm_msg.header.frame_id = 'base_link'
        larm_msg.joint_names = rospy.get_param("/l_arm_controller/joints")
	goal = JointTrajectoryPoint()
	goal.positions = self.larm_goal
	larm_msg.points.append(goal)
	self.larm_mpc_move.publish(larm_msg)

        rarm_msg = JointTrajectory()
        rarm_msg.header.frame_id = 'base_link'
        rarm_msg.joint_names = rospy.get_param("/r_arm_controller/joints")
        goal = JointTrajectoryPoint()
        goal.positions = self.rarm_goal
        rarm_msg.points.append(goal)
	self.rarm_mpc_move.publish(rarm_msg)
	for i in range(5):
	    self.larm_mpc_move.publish(larm_msg)
	    self.rarm_mpc_move.publish(rarm_msg)
	    rospy.sleep(1)
	
	torso_msg = JointTrajectory()
	torso_msg.header.frame_id = 'base_link'
        torso_msg.joint_names = rospy.get_param("/crona_torso_controller/joints")
        goal = JointTrajectoryPoint()
        goal.positions = self.torso_goal
	goal.time_from_start = rospy.Duration(5.)
        torso_msg.points.append(goal)
	for i in range(5):
	    print torso_msg
            self.torso_move.publish(torso_msg)
            rospy.sleep(1)
	
    def start(self):
	rospy.sleep(5)
	self.setup_crona()
	
	obs = Objects()
	obs.z_p = 0.525
	obs.y_p = 0.
	obs.x_p = 4
	obs.pub_object_switch(False)
	raw_input("Press Enter")
	obs.pub_object_switch(True)
	ok = raw_input("Placement ok? (1 or 0)")
	print int(ok) == 0
	while int(ok) == 0:
	    obs.pub_object_switch(False)
            raw_input("Press Enter")
	    obs.pub_object_switch(True)
            ok = raw_input("Placement ok?")

	base_msg = JointTrajectory()
        base_msg.header.frame_id = 'base_link'
        base_msg.joint_names = rospy.get_param("/crona_base_controller/joints")
        goal = JointTrajectoryPoint()
        goal.positions = self.base_goal
	goal.time_from_start = rospy.Duration(3.)
        base_msg.points.append(goal)
	for i in range(10):
            self.base_move.publish(base_msg)
            rospy.sleep(1)
	
	self.torso_goal = np.radians([-20.]).tolist()
	torso_msg = JointTrajectory()
        torso_msg.header.frame_id = 'base_link'
        torso_msg.joint_names = rospy.get_param("/crona_torso_controller/joints")
        goal = JointTrajectoryPoint()
        goal.positions = self.torso_goal
	goal.time_from_start = rospy.Duration(5.)
        torso_msg.points.append(goal)	

	self.larm_goal = np.radians([-87.,0.,0.,-85.,0.,-50.]).tolist()
        self.rarm_goal = np.radians([87.,0.,0.,85.,0.,50.]).tolist()

	larm_msg = JointTrajectory()
        larm_msg.header.frame_id = 'base_link'
        larm_msg.joint_names = rospy.get_param("/l_arm_controller/joints")
        goal = JointTrajectoryPoint()
        goal.positions = self.larm_goal
        larm_msg.points.append(goal)
        self.larm_mpc_move.publish(larm_msg)

        rarm_msg = JointTrajectory()
        rarm_msg.header.frame_id = 'base_link'
        rarm_msg.joint_names = rospy.get_param("/r_arm_controller/joints")
        goal = JointTrajectoryPoint()
        goal.positions = self.rarm_goal
        rarm_msg.points.append(goal)
        self.rarm_mpc_move.publish(rarm_msg)

	for i in range(5):
	    self.larm_mpc_move.publish(larm_msg)
            self.rarm_mpc_move.publish(rarm_msg)
            self.torso_move.publish(torso_msg)
            rospy.sleep(1)	

	pos = self.robot.kinematics.forward(self.joint_angles,'l_arm_forearm_roll_link')
    	Je = [self.robot.kinematics.jacobian(self.joint_angles, pos[:3,3])]

if __name__ == '__main__':
	lift = CronaSetup()
	lift.start()
