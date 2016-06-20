#!/usr/bin/python

###########################################
# Ros-Ros server used for simulation purposes.  This version of the server uses direct
# joint trajectory commands for low-level control. The actual robot will use a 
# Ros-Meka server.
###########################################

import rospy
import roslib
roslib.load_manifest('sttr_behaviors')
import threading

import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sttr_msgs.msg import CronaState, MoveJoint
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Bool

class CronaServer():
    def __init__(self):
	rospy.init_node('crona_server')
	#############################################################
	# These action clients should be replaced with Meka interface commands to control joints (set_theta_gc)
	# Eventually, in this code get rid of action servers and use straight ROS joint commands
	self.base_pub = rospy.Publisher('/crona_base_controller/command',JointTrajectory)
	self.torso_pub = rospy.Publisher('/crona_torso_controller/command',JointTrajectory)
	self.head_pub = rospy.Publisher('/crona_head_controller/command',JointTrajectory)
	self.larm_pub = rospy.Publisher('/l_arm_controller/command',JointTrajectory)
	self.rarm_pub = rospy.Publisher('/r_arm_controller/command',JointTrajectory)

	self.base_control = 0
	self.base_control_sub = rospy.Subscriber('/crona2_server/base_control',Bool,self.base_control_cb)

	self.state_lock = threading.RLock()
	
	self.joint_dict = {}
	self.joint_dict['base'] = ['base_transx_joint','base_transy_joint','base_rev_joint']
	self.joint_dict['torso'] = ['torso_joint']
	self.joint_dict['head'] = ['neck_joint']
	self.joint_dict['larm'] = ['l_shoulder_lift_joint', 'l_shoulder_roll_joint', 'l_shoulder_pan_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_hand_joint']
	self.joint_dict['rarm'] = ['r_shoulder_lift_joint', 'r_shoulder_roll_joint', 'r_shoulder_pan_joint','r_elbow_flex_joint','r_forearm_roll_joint','r_hand_joint']

	self.larm_kp = [rospy.get_param('/l_arm_controller/gains/'+nm+'/p') for nm in rospy.get_param('/l_arm_controller/joints')]
	self.rarm_kp = [rospy.get_param('/r_arm_controller/gains/'+nm+'/p') for nm in rospy.get_param('/r_arm_controller/joints')]
	self.torso_kp = [rospy.get_param('/crona_torso_controller/gains/'+nm+'/p') for nm in rospy.get_param('/crona_torso_controller/joints')]

	self.pub = rospy.Publisher('crona/sim_state',CronaState)
	self.sub = rospy.Subscriber('joint_states',JointState,self.joint_state_cb)
	self.sub = rospy.Subscriber('crona/command',MoveJoint,self.move_joints)
	rospy.Subscriber('/l_arm_controller/state', JointTrajectoryControllerState, self.larm_joint_controller_states_cb)
	rospy.Subscriber('/r_arm_controller/state', JointTrajectoryControllerState, self.rarm_joint_controller_states_cb)
	rospy.Subscriber('/crona_torso_controller/state', JointTrajectoryControllerState, self.torso_joint_controller_states_cb)

    def base_control_cb(self,msg):
	self.base_control = msg.data

	# primary server function, receives MoveJoint message via callback		
    def move_joints(self,msg):	
	goals = msg.goals
	time = msg.time
		
	jt = JointTrajectory()
	jt.header.frame_id = "/base_link"    
	jp = JointTrajectoryPoint()
	jp.positions = [goals[0],goals[1],np.radians(goals[2]).tolist()]
	#jp.positions = [goals[0],goals[1],goals[2]]
	jp.velocities = [0. for i in range(3)]
	jp.accelerations = [0. for i in range(3)]
	jp.time_from_start = rospy.Duration(time)
	jt.joint_names = self.joint_dict['base']
	jt.points.append(jp)	
	#print "base_control: ",self.base_control
	if self.base_control == 1:
	    self.base_pub.publish(jt)
	
	jt = JointTrajectory()
        jt.header.frame_id = "/base_link"    
        jp = JointTrajectoryPoint()        
	jp.positions = [np.radians(goals[3]).tolist()]
	#jp.positions = [goals[3]]
	jp.velocities = [0.]
	jp.accelerations = [0.]
	jp.time_from_start = rospy.Duration(time)
	jt.joint_names = self.joint_dict['torso']
	jt.points.append(jp)
	self.torso_pub.publish(jt)
        
	jt = JointTrajectory()
        jt.header.frame_id = "/base_link"
        jp = JointTrajectoryPoint()        
	jp.positions = [np.radians(goals[4]).tolist()]
	#jp.positions = [goals[4]]
        jp.velocities = [0.]
        jp.accelerations = [0.]
        jp.time_from_start = rospy.Duration(time)
        jt.joint_names = self.joint_dict['head']
        jt.points.append(jp)
        self.head_pub.publish(jt)
	
	if len(goals)>5:	
	    jt = JointTrajectory()
            jt.header.frame_id = "/base_link"
            jp = JointTrajectoryPoint()
	    jp.positions = np.radians(goals[5:11]).tolist()
	    #jp.positions = goals[5:11]
	    jp.velocities = [0. for i in range(6)]
	    jp.accelerations = [0. for i in range(6)]
	    jp.time_from_start = rospy.Duration(time)
	    jt.joint_names = self.joint_dict['larm']
	    jt.points.append(jp)
	    self.larm_pub.publish(jt)
		
	    jt = JointTrajectory()
            jt.header.frame_id = "/base_link"
            jp = JointTrajectoryPoint()
	    jp.positions = np.radians(goals[11:17]).tolist()
	    #jp.positions = goals[11:17]
	    jp.velocities = [0. for i in range(6)]
	    jp.accelerations = [0. for i in range(6)]
      	    jp.time_from_start = rospy.Duration(time)
	    jt.joint_names = self.joint_dict['rarm']
	    jt.points.append(jp)
	    self.rarm_pub.publish(jt)
	
    # call back function to publish robot state  			
    def joint_state_cb(self,msg):
	self.joint_state_msg = msg

    def larm_joint_controller_states_cb(self,msg):  
	self.larm_desired_pos_msg = msg

    def rarm_joint_controller_states_cb(self,msg):  
	self.rarm_desired_pos_msg = msg

    def torso_joint_controller_states_cb(self,msg):  
	self.torso_desired_pos_msg = msg

    def calculate_joint_torques(self,joint_states,larm_desired_pos,rarm_desired_pos,torso_desired_pos):
	larm_jt = np.array(self.larm_kp)*(np.array(larm_desired_pos)-np.array(joint_states.position[5:11]))
	rarm_jt = np.array(self.rarm_kp)*(np.array(rarm_desired_pos)-np.array(joint_states.position[11:]))
	torso_jt = np.array(self.torso_kp)*(np.array(torso_desired_pos)-np.array(joint_states.position[3]))
	return larm_jt, rarm_jt, torso_jt
	
    def publish_crona_state_msg(self):
	with self.state_lock:
	    joint_states = self.joint_state_msg
	    larm_desired_pos = self.larm_desired_pos_msg.desired.positions
	    rarm_desired_pos = self.rarm_desired_pos_msg.desired.positions
	    torso_desired_pos = self.torso_desired_pos_msg.desired.positions
	    crona_state_msg = CronaState()
	    crona_state_msg.name = joint_states.name
	    crona_state_msg.actual_position = joint_states.position
	    crona_state_msg.velocity = joint_states.velocity
	    crona_state_msg.effort = joint_states.effort
	    crona_state_msg.desired_position = list(joint_states.position[0:3])+list(torso_desired_pos)+[joint_states.position[4]]+list(larm_desired_pos)+list(rarm_desired_pos)
	    larm_jt, rarm_jt, torso_jt = self.calculate_joint_torques(joint_states,self.larm_desired_pos_msg.desired.positions,self.rarm_desired_pos_msg.desired.positions,self.torso_desired_pos_msg.desired.positions)
	    crona_state_msg.joint_torque = [0,0,0]+list(torso_jt)+[0]+list(larm_jt)+list(rarm_jt) 
	    self.pub.publish(crona_state_msg)
	    
	
    def start(self):
	rospy.sleep(1)
	while not rospy.is_shutdown():
	    self.publish_crona_state_msg()
	    rospy.sleep(.01)

if __name__=='__main__':
    cs = CronaServer()
    cs.start()
