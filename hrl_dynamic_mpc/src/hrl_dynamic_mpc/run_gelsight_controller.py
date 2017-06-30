#!/usr/bin/python
#
#
# Copyright (c) 2013, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#system level imports
import sys, os
import numpy as np
import scipy.io
import copy
from threading import RLock, Timer
import time
import itertools as it
from scipy.linalg import expm

#ROS or folder level imports
import roslib;
roslib.load_manifest('hrl_dynamic_mpc')
import rospy
import skin_client as sc
import hrl_lib.viz as hv
import hrl_lib.transforms as tr
import hrl_lib.util as ut
import hrl_lib.circular_buffer as cb
import tf
import tf.transformations as trans
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Quaternion, PointStamped
from hrl_haptic_manipulation_in_clutter_msgs.msg import MpcDynFormattedData, RobotHapticState, HapticMpcState
from hrl_haptic_manipulation_in_clutter_srvs.srv import HapticMPCLogging, EnableHapticMPC 
from hrl_msgs.msg import FloatArrayBare
import darci_client_ashenoi as dc

def pixel3d_callback(msg):
    global new_goal
    global end_pos
    global end_rot
    print "Recieved pixel3d message"
    end_pos = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
    end_pos = end_pos.reshape(3,1)
    quat1 = (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
    print quat1
    #quat1 = trans.quaternion_multiply(quat1,(0.2,-0.9,0.0,0))
    quat1 = trans.quaternion_multiply(quat1,( 0., 0.70682518,0. ,0.70738827))
    quat = trans.quaternion_multiply(quat1,( 0. ,  0.,  0.70682518,  0.70738827))
    end_rot = tr.quaternion_to_matrix(quat)
    print end_rot
    new_goal = True



def run():
    global new_goal
    global end_pos
    global end_rot
    next_goal_publiser = rospy.Publisher('/next_goal',PoseStamped, latch=True)
    ee_pose_publiser = rospy.Publisher('/ee_pose',PoseStamped, latch=True)
    end_publiser = rospy.Publisher('/end',PoseStamped, latch=True)
    darci = dc.DarciClient(arm='r')
    #q = [0.0]*7
    q= [-0.53501512730918466, 0.10882719831888275, -0.015439142607576152, 1.957521229573663, 0.12570648778098859, 0.009158985583357, 0.029797142438283013]
    #q= [-0.13501512730918466, 0.10882719831888275, -0.015439142607576152, 1.657521229573663, 0.12570648778098859, 0.009158985583357, 0.029797142438283013]
    print darci.joint_angles
    darci.setDesiredJointAngles(q)
    darci.updateSendCmd() 
    home_q =  q;
    #rospy.sleep(5)
    #q[6] = 1
    #darci.setDesiredJointAngles(q)
    #darci.updateSendCmd() 
    #rospy.sleep(5)
    #darci.setDesiredJointAngles([0.1]*7)
    #darci.updateSendCmd() 
    #rospy.sleep(5)
    #darci.setDesiredJointAngles([0.0]*7)
    #darci.updateSendCmd() 
    #rospy.sleep(5)
    end_pos, end_rot = darci.kinematics.FK(darci.joint_angles)
    org_pos, org_rot = darci.kinematics.FK(darci.joint_angles)
    org_quat = tr.matrix_to_quaternion(org_rot)
    print "angles"
    print darci.joint_angles
   # while True:
   # #    org_pos, org_rot = darci.kinematics.FK(darci.joint_angles)
   # #    org_quat = tr.matrix_to_quaternion(org_rot)
   #     end_quat = tr.matrix_to_quaternion(end_rot)
   #     
   # #    next_goal_msg = PoseStamped()
   # #    next_goal_msg.header.frame_id = '/torso_lift_link'
   # #    next_goal_msg.pose.position.x = org_pos[0];
   # #    next_goal_msg.pose.position.y = org_pos[1];
   # #    next_goal_msg.pose.position.z = org_pos[2];
   # #    next_goal_msg.pose.orientation.x = org_quat[0]
   # #    next_goal_msg.pose.orientation.y = org_quat[1]
   # #    next_goal_msg.pose.orientation.z = org_quat[2]
   # #    next_goal_msg.pose.orientation.w = org_quat[3]
   # #    next_goal_publiser.publish(next_goal_msg)


   #     end_msg = PoseStamped()
   #     end_msg.header.frame_id = '/torso_lift_link'
   #     end_msg.pose.position.x = end_pos[0];
   #     end_msg.pose.position.y = end_pos[1];
   #     end_msg.pose.position.z = end_pos[2];
   #     end_msg.pose.orientation.x = end_quat[0]
   #     end_msg.pose.orientation.y = end_quat[1]
   #     end_msg.pose.orientation.z = end_quat[2]
   #     end_msg.pose.orientation.w = end_quat[3]
   #     end_publiser.publish(end_msg)
    print org_pos
    print org_rot
    org_rot = tr.quaternion_to_matrix(org_quat)
    print org_rot
    org_pos = np.array(org_pos);
    next_goal_msg = PoseStamped()
    next_goal_msg.header.frame_id = '/torso_lift_link'
    next_goal_msg.pose.position.x = org_pos[0];
    next_goal_msg.pose.position.y = org_pos[1];
    next_goal_msg.pose.position.z = org_pos[2];
    next_goal_msg.pose.orientation.x = org_quat[0]
    next_goal_msg.pose.orientation.y = org_quat[1]
    next_goal_msg.pose.orientation.z = org_quat[2]
    next_goal_msg.pose.orientation.w = org_quat[3]
    next_goal_publiser.publish(next_goal_msg)



    r = rospy.Rate(1)
    print "Starting run_gelsight_controller"
    while not rospy.is_shutdown():
        darci.setDesiredJointAngles(q)
        darci.updateSendCmd() 
        if new_goal == True:
            print "Recieved new goal"
	    #print "Going to initial position first"
    	    #darci.setDesiredJointAngles(q)
    	    #darci.updateSendCmd() 
	    #rospy.sleep(5)
            print "Reaching to Goal position:"		
	    #org_x = org_pos[0];
	    #org_y = org_pos[1];
	    #org_z = org_pos[2];
	    #end_x = end_pos[0];
	    #end_y = end_pos[1];
	    #end_z = end_pos[2];


	    #print "Org: ",org_x," ",org_y," ",org_z
	    #print "End: ",end_x," ",end_y," ",end_z
	    dist = np.sum(np.square(org_pos - end_pos))
	    print dist
	    print "End end_pos",end_pos
	    print "Org_pose",org_pos
	    step =10
	    TOTAL_TRIAL = 15;
	    i =1
	    fail = 0;
	    while i <= step:

    		ee_pos, ee_rot = darci.kinematics.FK(darci.joint_angles)
    		ee_quat = tr.matrix_to_quaternion(ee_rot)
		ee_pose_msg = PoseStamped()
		ee_pose_msg.header.frame_id = '/torso_lift_link'
		ee_pose_msg.pose.position.x = ee_pos[0];
		ee_pose_msg.pose.position.y = ee_pos[1];
		ee_pose_msg.pose.position.z = ee_pos[2];
		ee_pose_msg.pose.orientation.x = ee_quat[0]
		ee_pose_msg.pose.orientation.y = ee_quat[1]
		ee_pose_msg.pose.orientation.z = ee_quat[2]
		ee_pose_msg.pose.orientation.w = ee_quat[3]
		ee_pose_publiser.publish(ee_pose_msg)
	#	if i != 1:
    	#	    ee_pos, ee_rot = darci.kinematics.FK(darci.joint_angles)
	#	    dist = np.sum(np.square(ee_pos - pos_next))
	#	    print dist
	#	    while dist > 0.001:
	#		print "Inside closed loop"
	#	        set_goal = darci.SetOrientGoal(pos_next,rot_next)
	#	        rospy.sleep(2)
    	#	        ee_pos, ee_rot = darci.kinematics.FK(darci.joint_angles)
	#	        dist = np.sum(np.square(ee_pos - pos_next))
	#		print dist
			
			
		        
 
	        org_quat = tr.matrix_to_quaternion(org_rot)
	        end_quat = tr.matrix_to_quaternion(end_rot)
	        pos_next = org_pos - (org_pos - end_pos)*(1./step)*i;
		quat_next = trans.quaternion_slerp(org_quat,end_quat,(1./step)*i)
		#quat_next = trans.quaternion_slerp(org_quat,end_quat,(1./step)*0)
		rot_next = tr.quaternion_to_matrix(quat_next)
		#rot_next = org_rot
		#quat_next = org_quat
		#print (org_pos - end_pos)*(1./step)*i;
		#print "Pos next", pos_next
		next_goal_msg = PoseStamped()
		next_goal_msg.header.frame_id = '/torso_lift_link'
		next_goal_msg.pose.position.x = pos_next[0];
		next_goal_msg.pose.position.y = pos_next[1];
		next_goal_msg.pose.position.z = pos_next[2];
		next_goal_msg.pose.orientation.x = quat_next[0]
		next_goal_msg.pose.orientation.y = quat_next[1]
		next_goal_msg.pose.orientation.z = quat_next[2]
		next_goal_msg.pose.orientation.w = quat_next[3]
		next_goal_publiser.publish(next_goal_msg)



		set_goal = darci.SetOrientGoal(pos_next,rot_next)
		#set_goal = True
		
		if set_goal == True:
    		    
		    i +=1
		    print "Success"
		    #org_pos = pos_next
		    #org_rot = rot_next
		    rospy.sleep(2)
		    fail = 0
		else:
		    fail += 1
		if fail >= TOTAL_TRIAL:
		    break;
            if fail == TOTAL_TRIAL:
	        print "FAILED"
	    else:
	        print "Completed. Holding pose for a few seconds"
		rospy.sleep(5)
		print darci.joint_angles
		rospy.sleep(5)

            new_goal = False
        r.sleep()
    

if __name__ == '__main__':
    global new_goal
    new_goal = False
    rospy.init_node('gelsight_arm_node', anonymous = True)
    rospy.sleep(5)
    poxel_3d_sub = rospy.Subscriber("/pixel3d", PoseStamped, pixel3d_callback)
    run()
    


    

