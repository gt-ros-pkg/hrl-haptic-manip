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

class MoveTorso():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/crona_torso_controller/joint_trajectory_action',JointTrajectoryAction)
        self.client.wait_for_server()

    def publish_movement(self,goal):
        jt = JointTrajectory()
        jt.header.frame_id = "/base_link"
        jt.joint_names = ["torso_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = goal
        jtp.velocities = [0.]
        jtp.accelerations = [0.]
        #jtp.time_from_start = rospy.Duration(25.)
        jtp.time_from_start = rospy.Duration(5.)
        jt.points.append(jtp)
        g = JointTrajectoryGoal()
        g.trajectory = jt
        self.client.send_goal(g)
        #self.client.wait_for_result()
        #return self.client.get_result()
	
if __name__=='__main__':
    rospy.init_node('move_torso')
    mt = MoveTorso()
    #torso_pos = [-0.5,2.1,0.2]
    #while not rospy.is_shutdown():
    for i in range(5):
        torso_pos = np.radians([-80.]).tolist()
        #torso_pos = [-0.436,-0.349,0.698]
	mt.publish_movement(torso_pos)
	rospy.sleep(1)
        #torso_pos = np.radians([30.]).tolist()
        #torso_pos = [0.52]
	#torso_pos = [0.1,0.1,0.698]
        #mt.publish_movement(torso_pos)   
	#rospy.sleep(1)
	#rospy.sleep(2)
    #torso_pos = [-0.436,-0.349,0.698]
    #mt.publish_movement(torso_pos)
    #torso_pos = [0.1,-0.349,0.698]
    #mt.publish_movement(torso_pos)
    #torso_pos = [-0.436,-0.349,0.698]
    #mt.publish_movement(torso_pos)
    #torso_pos = [0.436,-0.349,0.698]
    #mt.publish_movement(torso_pos)
     


