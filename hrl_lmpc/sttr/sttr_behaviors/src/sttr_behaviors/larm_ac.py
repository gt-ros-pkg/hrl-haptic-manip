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

class MoveLeftArm():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/l_arm_controller/joint_trajectory_action',JointTrajectoryAction)
        self.client.wait_for_server()

    def publish_movement(self,goal):
        jt = JointTrajectory()
        jt.header.frame_id = "/base_link"
        jt.joint_names = ['l_shoulder_lift_joint', 'l_shoulder_roll_joint', 'l_shoulder_pan_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_hand_joint']
        jtp = JointTrajectoryPoint()
	jtp.positions = goal
        jtp.velocities = [0.,0.,0.,0.,0.,0.]
        jtp.accelerations = [0.,0.,0.,0.,0.,0.]
	jtp.time_from_start = rospy.Duration(5)
        jt.points.append(jtp)
	g = JointTrajectoryGoal()
        g.trajectory = jt
        self.client.send_goal(g)
        self.client.wait_for_result()
        return self.client.get_result()

if __name__=='__main__':
    rospy.init_node('move_left_arm')
    mla = MoveLeftArm()
    while not rospy.is_shutdown():
	arm_goal = np.radians([10,0,0,80,0,0.]).tolist()
	#arm_goal = np.radians([10.,0.,0.,80.,0.,30.]).tolist()
	#arm_goal = np.radians([10.,10.,10.,50.,10.,10.]).tolist()
	#arm_goal = [-math.pi/2,0,0,0,0,0]
	#arm_goal = [-math.pi/2,-math.pi/12,0,0,0,0]
	#arm_goal = [-math.pi/2,-math.pi/12,-math.pi/12,0,0,0]
	#arm_goal = [-math.pi/2,-math.pi/12,-math.pi/12,-math.pi/3,0,0]
	#arm_goal = [-math.pi/2,-math.pi/12,-math.pi/12,-math.pi/3,-math.pi/6,0]
	#arm_goal = [-math.pi/2,-math.pi/12,-math.pi/12,-math.pi/3,-math.pi/6,-math.pi/3]
	#arm_goal = [0,0,0,-math.pi/4,0,0]
	mla.publish_movement(arm_goal)
	rospy.sleep(.1)
	#arm_goal = [0,0,0,0,0,0]
	#arm_goal = [-math.pi/4,0,0,0,0,0]
	#arm_goal = [0,math.pi/12,0,0,0,0]
	#arm_goal = [0,0,math.pi/6,0,0,0]
	#arm_goal = [0,0,0,-math.pi/6,0,0]
	#arm_goal = [0,0,0,0,math.pi/6,0]
	
	#arm_goal = [0,0,0,0,0,-math.pi/3]
        #arm_goal = [0,0,0,0,0,0]
	#mla.publish_movement(arm_goal)
	#rospy.sleep(1)
