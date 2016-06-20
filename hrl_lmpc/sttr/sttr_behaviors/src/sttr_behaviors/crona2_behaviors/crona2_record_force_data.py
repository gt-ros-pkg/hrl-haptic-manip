#!/usr/bin/env python
#######################################################
# 
# cRoNA2 Force Recording Script
# 
########################################################
# subscribes to force sensor to record data to a text file

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
 	self.force_data = open('force_data1', 'wb')
	self.csv_force_data = csv.writer(self.force_data)
	rospy.Subscriber('/r_arm/crona/est_force', TaxelArray, self.larm_force_cb)
	rospy.Subscriber('/l_arm/crona/est_force', TaxelArray, self.rarm_force_cb)
	rospy.Subscriber('/gazebo/crona_wrench', WrenchArray, self.crona_wrench_cb)

        self.larm_force = [0,0,0]
        self.rarm_force = [0,0,0]
	self.input_lforearm_force_x = 0
        self.input_lforearm_force_y = 0
        self.input_lforearm_force_z = 0
        self.input_rforearm_force_x = 0
        self.input_rforearm_force_y = 0
        self.input_rforearm_force_z = 0

	while not rospy.is_shutdown():
	    self.csv_force_data.writerow([self.input_lforearm_force_x,self.input_lforearm_force_y,self.input_lforearm_force_z,self.input_rforearm_force_x,self.input_rforearm_force_y,self.input_rforearm_force_z,self.larm_force[0],self.larm_force[1],self.larm_force[2],self.rarm_force[0],self.rarm_force[1],self.rarm_force[2]])
	    rospy.sleep(0.1)
	self.csv_force_data.close()

    def larm_force_cb(self,msg):
	self.larm_force[0] = msg.values_x[0]
	self.larm_force[1] = msg.values_y[0]
	self.larm_force[2] = msg.values_z[0]

    def rarm_force_cb(self,msg):
	self.rarm_force[0] = msg.values_x[0]
        self.rarm_force[1] = msg.values_y[0]
        self.rarm_force[2] = msg.values_z[0]

    def crona_wrench_cb(self,msg):
	self.input_lforearm_force_x = msg.force_x[0]
	self.input_lforearm_force_y = msg.force_y[0]
	self.input_lforearm_force_z = msg.force_z[0]
	self.input_rforearm_force_x = msg.force_x[1]
        self.input_rforearm_force_y = msg.force_y[1]
        self.input_rforearm_force_z = msg.force_z[1]

if __name__ == '__main__':
    rospy.init_node('crona2_force_data')
    rd = RecordData()
    
