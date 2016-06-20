#!/usr/bin/env python
#######################################################
# 
# cRoNA2 Debug Recording Script
# 
########################################################
# subscribes to topics from the LMPC dealing with moment balancing debugging
# records to a text file

import sys
import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
import numpy as np
from sttr_msgs.msg import DebugMomentBalancing, WrenchArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from hrl_msgs.msg import *
from math import *
from LMPC_msgs.msg import *
import csv
from subprocess import call
import copy

class RecordData(object):
    def __init__(self):
 	self.debug_data = open('debug_moment_balancing1', 'wb')
	self.csv_debug_data = csv.writer(self.debug_data)
	rospy.Subscriber('/haptic_mpc/debug_moment_balancing',DebugMomentBalancing,self.debug_moment_balancing_cb)
	rospy.Subscriber('/gazebo/crona_wrench', WrenchArray, self.crona_wrench_cb)

	self.estimated_weight = 0
        self.larm_force = [0,0,0]
        self.rarm_force = [0,0,0]
        self.current_larm_moment = 0
        self.current_rarm_moment = 0
        self.current_net_moment = 0
        self.delta_larm_moment = 0
        self.delta_rarm_moment = 0
        self.delta_net_moment = 0
        self.delta_lforearm_x = [0,0,0]
        self.delta_rforearm_x = [0,0,0]
        self.predicted_larm_moment = 0
        self.predicted_rarm_moment = 0
        self.predicted_net_moment = 0
	self.input_lforearm_force_x = 0
        self.input_lforearm_force_y = 0
        self.input_lforearm_force_z = 0
        self.input_rforearm_force_x = 0
        self.input_rforearm_force_y = 0
        self.input_rforearm_force_z = 0

	while not rospy.is_shutdown():
	    self.csv_debug_data.writerow([self.estimated_weight,self.larm_force[2],self.rarm_force[2],self.current_larm_moment,self.current_rarm_moment,self.current_net_moment,self.delta_larm_moment,self.delta_rarm_moment,self.delta_net_moment,self.delta_lforearm_x[0],self.delta_lforearm_x[1],self.delta_lforearm_x[2],self.delta_rforearm_x[0],self.delta_rforearm_x[1],self.delta_rforearm_x[2],self.predicted_larm_moment,self.predicted_rarm_moment,self.predicted_net_moment,self.input_lforearm_force_x,self.input_lforearm_force_y,self.input_lforearm_force_z,self.input_rforearm_force_x,self.input_rforearm_force_y,self.input_rforearm_force_z])
	    rospy.sleep(0.1)
	self.csv_debug_data.close()

    def debug_moment_balancing_cb(self,msg):
	self.estimated_weight = msg.estimated_weight
 	self.larm_force = msg.larm_force
 	self.rarm_force = msg.rarm_force
	self.current_larm_moment = msg.current_larm_moment
	self.current_rarm_moment = msg.current_rarm_moment
	self.current_net_moment = msg.current_net_moment
	self.delta_larm_moment = msg.delta_larm_moment
	self.delta_rarm_moment = msg.delta_rarm_moment
	self.delta_net_moment = msg.delta_net_moment
	self.delta_lforearm_x = msg.delta_lforearm_x
	self.delta_rforearm_x = msg.delta_rforearm_x
	self.predicted_larm_moment = msg.predicted_larm_moment
	self.predicted_rarm_moment = msg.predicted_rarm_moment
	self.predicted_net_moment = msg.predicted_net_moment

    def crona_wrench_cb(self,msg):
	self.input_lforearm_force_x = msg.force_x[0]
	self.input_lforearm_force_y = msg.force_y[0]
	self.input_lforearm_force_z = msg.force_z[0]
	self.input_rforearm_force_x = msg.force_x[1]
        self.input_rforearm_force_y = msg.force_y[1]
        self.input_rforearm_force_z = msg.force_z[1]

if __name__ == '__main__':
    rospy.init_node('crona2_debug')
    rd = RecordData()
    
