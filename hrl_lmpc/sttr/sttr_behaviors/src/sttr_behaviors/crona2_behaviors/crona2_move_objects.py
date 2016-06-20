#!/usr/bin/env python
#######################################################
# 
# cRoNA2 Move Objects in Simulation Script
# 
########################################################
# intended to interface with the Gazebo simulation to reposition objects without restarting Gazebo

import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
from initGazebocRoNASim import Objects

class cRoNA2MoveObjectsSim(object):
    def __init__(self):
	self.obs = Objects()

    def set_object_position(self,position):
	self.obs.x_p = position[0]
	self.obs.y_p = position[1]
	self.obs.z_p = position[2]
	print "position[1]: ",position[1]
	print self.obs.x_p, self.obs.y_p, self.obs.z_p

    def start(self):
	self.set_object_position([4.,-0.5134,0.525])
        while not rospy.is_shutdown():
	    reply = raw_input("Please enter True or False (pub_object_switch): ")
	    self.obs.pub_object_switch(reply=='True')

if __name__ == '__main__':
    rospy.init_node('crona2_move_objects_sim')
    c2mos = cRoNA2MoveObjectsSim()
    c2mos.start()
