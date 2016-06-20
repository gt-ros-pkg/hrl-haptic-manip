#!/usr/bin/python

import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *
import math
import actionlib
from actionlib_msgs.msg import *
import numpy as np
#import pose_utils as pu

class MoveRightArm():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/r_arm_controller/joint_trajectory_action',JointTrajectoryAction)
        self.client.wait_for_server()

    def publish_movement(self,goal):
        jt = JointTrajectory()
        jt.header.frame_id = "/base_link"
        jt.joint_names = ['r_shoulder_lift_joint', 'r_shoulder_roll_joint', 'r_shoulder_pan_joint','r_elbow_flex_joint','r_forearm_roll_joint','r_hand_joint']
        jtp = JointTrajectoryPoint()
	jtp.positions = goal
        jtp.velocities = [0.,0.,0.,0.,0.,0.]
        jtp.accelerations = [0.,0.,0.,0.,0.,0.]
	jtp.time_from_start = rospy.Duration(1)
        jt.points.append(jtp)
	g = JointTrajectoryGoal()
        g.trajectory = jt
        self.client.send_goal(g)
        self.client.wait_for_result()
        return self.client.get_result()

if __name__=='__main__':
    rospy.init_node('move_right_arm')
    mla = MoveRightArm()
    while not rospy.is_shutdown():
        arm_goal = np.radians([10,0,0,80,0,0.]).tolist()
        #arm_goal = np.radians([10,10,10,80,10,10]).tolist()
	#arm_goal = [-math.pi/2,0,0,0,0,0]
	#arm_goal = [-math.pi/2,-math.pi/12,0,0,0,0]
	#arm_goal = [-math.pi/2,-math.pi/12,-math.pi/12,0,0,0]
	#arm_goal = [-math.pi/2,-math.pi/12,-math.pi/12,-math.pi/3,0,0]
	#arm_goal = [-math.pi/2,-math.pi/12,-math.pi/12,-math.pi/3,-math.pi/6,0]
	#arm_goal = [-math.pi/2,-math.pi/12,-math.pi/12,-math.pi/3,-math.pi/6,-math.pi/3]
	#arm_goal = [0,0,0,math.pi/4,0,0]
	mla.publish_movement(arm_goal)
	rospy.sleep(.1)
	#arm_goal = [0,0,0,0,0,0]
	#arm_goal = [-math.pi/4,0,0,0,0,0]
	#arm_goal = [-math.pi/4,math.pi/12,0,0,0,0]
	#arm_goal = [-math.pi/4,math.pi/12,-math.pi/6,0,0,0]
	#arm_goal = [-math.pi/4,math.pi/12,-math.pi/6,0,0,0]
	#arm_goal = [-math.pi/4,math.pi/12,-math.pi/6,0,math.pi/6,0]
	
	#arm_goal = [-math.pi/4,math.pi/12,-math.pi/6,0,math.pi/6,0]
        #arm_goal = [0,0,0,0,0,0]
	#mla.publish_movement(arm_goal)
