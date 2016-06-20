#!/usr/bin/env python

# script for gathering controller debugging data from LMPC
# subscribes to topics published from LMPC and stores them in text files

import sys
import numpy as np
import roslib
roslib.load_manifest('sttr_behaviors')
import rospy

from LMPC_msgs.msg import ControllerData
from sttr_msgs.msg import WrenchArray
import csv


class cRoNA2ControllerData(object):
    def __init__(self):
	controller_data_file = open('constraint1-f1-2','w')
	self.csv_data_file = csv.writer(controller_data_file)
	self.desired_object_pose = []
	self.object_wrench = (0,0,0)

	controller_data_sub = rospy.Subscriber('haptic_mpc/controller_data', ControllerData, self.controller_data_callback)
	object_wrench_sub = rospy.Subscriber('gazebo/object_wrench', WrenchArray, self.object_wrench_callback)

    def controller_data_callback(self,msg):
        # 1-7
    	self.desired_object_pose = msg.desired_object_pose
        # 8-14
    	self.actual_object_pose = msg.actual_object_pose
	# 15-21
    	self.delta_phi = msg.delta_phi
	# 22-24
    	self.friction_norm = msg.friction_norm
	# 25-27
    	self.pred_friction_norm = msg.pred_friction_norm
	# 28
    	self.alpha = msg.alpha
	# 29
    	self.pred_alpha = msg.pred_alpha
	# 30-32
    	self.tangent_direction = msg.tangent_direction
	# 33-35
    	self.pred_tangent_direction = msg.pred_tangent_direction
	# 36
        self.friction_force = msg.friction_force
	# 37
        self.pred_friction_force = msg.pred_friction_force
	# 38
    	self.tangent_dist_force = msg.tangent_dist_force
	# 39
    	self.pred_tangent_dist_force = msg.pred_tangent_dist_force
	# 40
    	self.normal_force = msg.normal_force
	# 41
    	self.pred_normal_force = msg.pred_normal_force
	# 42
    	self.normal_dist_force = msg.normal_dist_force
	# 43
    	self.pred_normal_dist_force = msg.pred_normal_dist_force
	# 44
    	self.dist_to_goal = msg.dist_to_goal
	# 45
    	self.ang_to_goal = msg.ang_to_goal

    def object_wrench_callback(self,msg):
	print msg.force_x, type(msg.force_x)
	print msg.force_y, type(msg.force_y)
	print msg.force_z, type(msg.force_z)
	self.object_wrench = (msg.force_x[0], msg.force_y[0], msg.force_z[0])
	
    def record_data(self):
	r = rospy.Rate(25.)
	while not rospy.is_shutdown():
            if self.desired_object_pose != []:
            	self.csv_data_file.writerow(self.desired_object_pose+self.actual_object_pose+self.delta_phi+self.friction_norm+self.pred_friction_norm+self.alpha+self.pred_alpha+self.tangent_direction+self.pred_tangent_direction+self.friction_force+self.pred_friction_force+self.tangent_dist_force+self.pred_tangent_dist_force+self.normal_force+self.pred_normal_force+self.normal_dist_force+self.pred_normal_dist_force+self.dist_to_goal+self.ang_to_goal+self.object_wrench)
    	    r.sleep()   
 
if __name__ == '__main__':
    rospy.init_node('controller_data')
    c2cd = cRoNA2ControllerData()
    c2cd.record_data()
