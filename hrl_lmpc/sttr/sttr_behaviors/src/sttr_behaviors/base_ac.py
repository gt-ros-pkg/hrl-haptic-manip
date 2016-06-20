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
import sys
#import pose_utils as pu

class MoveBase():
    def __init__(self):
	self.client = actionlib.SimpleActionClient('/crona_base_controller/joint_trajectory_action',JointTrajectoryAction)	
	self.client.wait_for_server()

    def publish_movement(self,goal_vel,time):
	jt = JointTrajectory()
	jt.header.frame_id = "/base_link"
	jt.joint_names = ["base_transx_joint","base_transy_joint","base_rev_joint"]
	jtp = JointTrajectoryPoint()
	jtp.positions = goal_vel
        jtp.velocities = [0.,0.,0.]
        jtp.accelerations = [0.,0.,0.]
        jtp.time_from_start = rospy.Duration(time)
        jt.points.append(jtp)
        g = JointTrajectoryGoal()
        g.trajectory = jt
	self.client.send_goal(g)
	self.client.wait_for_result()
        return self.client.get_result()

if __name__=='__main__':
    rospy.init_node('move_base')
    mb = MoveBase()
    for i in range(10):
	if len(sys.argv) > 1:
	    base_pos = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
	    mb.publish_movement(base_pos,3)
            rospy.sleep(1)
   	else:
	    base_vel = [0.,0,0.]
            mb.publish_movement(base_vel,3)
	    rospy.sleep(1)
    
