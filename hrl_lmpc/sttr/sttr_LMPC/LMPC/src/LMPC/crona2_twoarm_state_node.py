#!/usr/bin/env python

#   Copyright 2014 Georgia Tech Research Corporation
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
#  http://healthcare-robotics.com/

## @package LMPC
#
# @author Kevin Chow, Jeff Hawke
# @version 0.1
# @copyright Apache Licence


import math
import numpy as np
import threading
import itertools as it
import sys, signal

import roslib
roslib.load_manifest("LMPC")
import rospy
import geometry_msgs.msg as geom_msgs
from tf import TransformListener
from std_msgs.msg import Header, Bool
from collections import deque

import hrl_msgs.msg
from hrl_lib import transforms as tr
import LMPC_msgs.msg as haptic_msgs
from sttr_msgs.msg import RagdollObjectArray, WrenchArray, CronaState
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

import multiarray_to_matrix
import haptic_mpc_util
import skin_client as sc

## @class RobotHapticStateServer Haptic state publisher: publishes all relevant haptic state information on a common interface independent of robot type.
class RobotHapticStateServer():
  ## Constructor for robot haptic state server
  def __init__(self, opt, node_name=None):

    self.larm_force_values = [np.zeros(20).tolist(),np.zeros(20).tolist(),np.zeros(20).tolist()]
    self.rarm_force_values = [np.zeros(20).tolist(),np.zeros(20).tolist(),np.zeros(20).tolist()]
    self.object_dist_force = [0,0,0]

    self.opt = opt
    # Set up all ros comms to start with
    self.node_name = node_name
    self.tf_listener = None
    self.state_pub = None
    self.rate = 100.0 # 100 Hz.
    self.msg_seq = 0 # Sequence counter
    self.mpc_terminate = 0

    # ROS Param server paths.
    self.base_path = "haptic_mpc"

    self.initComms()
    
    rospy.Subscriber("/gazebo/objectcog", RagdollObjectArray, self.object_state_callback)
    rospy.Subscriber("/gazebo/object", RagdollObjectArray, self.object_state_callback2)
    rospy.Subscriber("/l_arm/crona/est_force", haptic_msgs.TaxelArray, self.lforearm_force_callback)
    rospy.Subscriber("/r_arm/crona/est_force", haptic_msgs.TaxelArray, self.rforearm_force_callback)
    self.mpc_terminate_sub = rospy.Subscriber("/haptic_mpc/terminate",Bool,self.mpc_terminate_callback)

    rospy.Subscriber('/crona/sim_state', CronaState, self.joint_states_cb)

    # Skin data
    self.skin_topic_list = [] # List of topic names
    self.skin_client = None

    # Robot object. Contains all the subscribers and robot specific kinematics, etc
    self.robot = None

    # Joint data
    self.larm_joint_names = []
    self.larm_joint_angles = []
    self.larm_desired_joint_angles = []
    self.larm_joint_velocities = []
    self.larm_joint_stiffness = []
    self.larm_joint_damping = []
    self.joint_data_lock = threading.RLock()

    self.rarm_joint_names = []
    self.rarm_joint_angles = []
    self.rarm_desired_joint_angles = []
    self.rarm_joint_velocities = []
    self.rarm_joint_stiffness = []
    self.rarm_joint_damping = []

    # End effector pose
    self.larm_end_effector_position = None
    self.larm_end_effector_orient_cart = None
    self.larm_end_effector_orient_quat = None
    
    self.torso_pose = geom_msgs.Pose()

    # Ragdoll variables
    self.ragdoll_m1 = 0
    self.ragdoll_m2 = 0
    self.ragdoll_m3 = 0

    self.ragdoll_m1_ps = geom_msgs.PoseStamped()
    self.ragdoll_m2_ps = geom_msgs.PoseStamped()
    self.ragdoll_m3_ps = geom_msgs.PoseStamped()
    self.larm_friction_p1_ps = geom_msgs.PoseStamped()
    self.larm_friction_p2_ps = geom_msgs.PoseStamped()
    self.larm_friction_p3_ps = geom_msgs.PoseStamped()
    self.rarm_friction_p1_ps = geom_msgs.PoseStamped()
    self.rarm_friction_p2_ps = geom_msgs.PoseStamped()
    self.rarm_friction_p3_ps = geom_msgs.PoseStamped()
    self.object_ps = geom_msgs.PoseStamped()
    self.larm_object_Je = []
    self.rarm_object_Je = []
    self.larm_friction_Je1 = []
    self.larm_friction_Je2 = []
    self.larm_friction_Je3 = []
    self.rarm_friction_Je1 = []
    self.rarm_friction_Je2 = []
    self.rarm_friction_Je3 = []
    self.ragdoll_m2_Je = []
    self.ragdoll_m3_Je = []

    ma_size = 200
    self.larm_ma_x = deque([0 for i in range(ma_size)])
    self.larm_ma_y = deque([0 for i in range(ma_size)])
    self.larm_ma_z = deque([0 for i in range(ma_size)])
    self.rarm_ma_x = deque([0 for i in range(ma_size)])
    self.rarm_ma_y = deque([0 for i in range(ma_size)])
    self.rarm_ma_z = deque([0 for i in range(ma_size)])
    self.lforearm_force_x = 0
    self.lforearm_force_y = 0
    self.lforearm_force_z = 0
    self.rforearm_force_x = 0
    self.rforearm_force_y = 0
    self.rforearm_force_z = 0
    self.lforearm_ma_force_x = 0
    self.lforearm_ma_force_y = 0
    self.lforearm_ma_force_z = 0
    self.rforearm_ma_force_x = 0
    self.rforearm_ma_force_y = 0
    self.rforearm_ma_force_z = 0

    # Jacobian storage
    self.Jc = None # Contact jacobians
    self.larm_Je = None # End effector jacobian
    self.lforearm_J = None # End effector jacobian
    self.rarm_Je = None
    self.rforearm_J = None # End effector jacobian
    self.trim_threshold = 1.0 #this is 1.0 for forces

    # Jacobian MultiArray to Matrix converter
    self.ma_to_m = multiarray_to_matrix.MultiArrayConverter()

    # Initialise various parameters.
    self.initCrona()

  def joint_states_cb(self, msg):
    self.init_larm_desired_ja = msg.desired_position[5:11] 
    self.init_rarm_desired_ja = msg.desired_position[11:17]
    self.init_torso_desired_ja = msg.desired_position[3]

    self.larm_jt = msg.joint_torque[5:11]
    self.rarm_jt = msg.joint_torque[11:]
    self.torso_jt = msg.joint_torque[3]

  def mpc_terminate_callback(self,msg):
    if msg.data == 1:
        self.mpc_terminate = 1

    # SIMULATION ONLY
  def object_state_callback(self,msg):
    self.world_ragdoll_ps = msg   

  def object_state_callback2(self,msg):
    self.ragdoll_ps = msg

  def lforearm_force_callback(self,msg):
	self.lforearm_force_x = msg.values_x[0]
        self.lforearm_force_y = msg.values_y[0]
        self.lforearm_force_z = msg.values_z[0]
	'''
        try:
            if msg.values_x[0] != []:
                lforearm_force_x = []
                lforearm_force_y = []
                lforearm_force_z = []
                n = []
                for i in range(len(msg.normals_x)):
                    n_new = np.array((msg.normals_x[i],msg.normals_y[i],msg.normals_z[i]))
                    if i == 0:
                        lforearm_force_x.append(msg.values_x[i])
                        lforearm_force_y.append(msg.values_y[i])
                        lforearm_force_z.append(msg.values_z[i])
                        n.append(np.array((msg.normals_x[i],msg.normals_y[i],msg.normals_z[i])))
                    else:
                        for j in range(len(n)):
                            if np.dot(n_new,n[j]) > 0.8:
                                lforearm_force_x[j] += msg.values_x[i]
                                lforearm_force_y[j] += msg.values_y[i]
                                lforearm_force_z[j] += msg.values_z[i]
                            else:
                                lforearm_force_x.append(msg.values_x[i])
                                lforearm_force_y.append(msg.values_y[i])
                                lforearm_force_z.append(msg.values_z[i])
                                n.append(np.array((msg.normals_x[i],msg.normals_y[i],msg.normals_z[i])))

                self.lforearm_force_x = sum(lforearm_force_x)
                self.lforearm_force_y = sum(lforearm_force_y)
                self.lforearm_force_z = sum(lforearm_force_z)
                #for i in range(len(vx)):
                #    lforearm_force = np.linalg.norm((self.lforearm_force_x[i],self.lforearm_force_y[i],self.lforearm_force_z[i])) 
                #    if lforearm_force != 0:
                #       self.lforearm_force = lforearm_force
        except:
	    self.lforearm_force_x = 0
	    self.lforearm_force_y = 0
	    self.lforearm_force_z = 0
	'''

  def rforearm_force_callback(self,msg):
	self.rforearm_force_x = msg.values_x[0]
        self.rforearm_force_y = msg.values_y[0]
        self.rforearm_force_z = msg.values_z[0]
	'''
        try:
            if msg.values_x[0] != []:
                rforearm_force_x = []
                rforearm_force_y = []
                rforearm_force_z = []
                n = []
                for i in range(len(msg.normals_x)):
                    n_new = np.array((msg.normals_x[i],msg.normals_y[i],msg.normals_z[i]))
                    if i == 0:
                        rforearm_force_x.append(msg.values_x[i])
                        rforearm_force_y.append(msg.values_y[i])
                        rforearm_force_z.append(msg.values_z[i])
                        n.append(np.array((msg.normals_x[i],msg.normals_y[i],msg.normals_z[i])))
                    else:
                        for j in range(len(n)):
                            if np.dot(n_new,n[j]) > 0.8:
                                rforearm_force_x[j] += msg.values_x[i]
                                rforearm_force_y[j] += msg.values_y[i]
                                rforearm_force_z[j] += msg.values_z[i]
                            else:
                                rforearm_force_x.append(msg.values_x[i])
                                rforearm_force_y.append(msg.values_y[i])
                                rforearm_force_z.append(msg.values_z[i])
                                n.append(np.array((msg.normals_x[i],msg.normals_y[i],msg.normals_z[i])))

                self.rforearm_force_x = sum(rforearm_force_x)
                self.rforearm_force_y = sum(rforearm_force_y)
                self.rforearm_force_z = sum(rforearm_force_z)
                #for i in range(len(vx)):
                #    self.rforearm_force = np.linalg.norm((self.rforearm_force_x[i],self.rforearm_force_y[i],self.rforearm_force_z[i]))
                #    if rforearm_force != 0:
                #        self.rforearm_force = rforearm_force
        except:
	    self.rforearm_force_x = 0
            self.rforearm_force_y = 0
            self.rforearm_force_z = 0
	'''

  def get_force_moving_average(self):
    self.larm_ma_x.popleft()
    self.larm_ma_y.popleft()
    self.larm_ma_z.popleft()
    self.larm_ma_x.append(self.lforearm_force_x)
    self.larm_ma_y.append(self.lforearm_force_y)
    self.larm_ma_z.append(self.lforearm_force_z)
    self.lforearm_ma_force_x = np.mean(self.larm_ma_x)
    self.lforearm_ma_force_y = np.mean(self.larm_ma_y)
    self.lforearm_ma_force_z = np.mean(self.larm_ma_z)
    self.rarm_ma_x.popleft()
    self.rarm_ma_y.popleft()
    self.rarm_ma_z.popleft()
    self.rarm_ma_x.append(self.rforearm_force_x)
    self.rarm_ma_y.append(self.rforearm_force_y)
    self.rarm_ma_z.append(self.rforearm_force_z)
    self.rforearm_ma_force_x = np.mean(self.rarm_ma_x)
    self.rforearm_ma_force_y = np.mean(self.rarm_ma_y)
    self.rforearm_ma_force_z = np.mean(self.rarm_ma_z)

  def transform_object_state(self):
    world_ragdoll_ps = self.world_ragdoll_ps
    ragdoll_ps = self.ragdoll_ps
    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'world'
    if 'forearm_roll' not in world_ragdoll_ps.frame_names[-1]:
    	ps.pose.position.x = world_ragdoll_ps.centers_x[-1]
    	ps.pose.position.y = world_ragdoll_ps.centers_y[-1]
    	ps.pose.position.z = world_ragdoll_ps.centers_z[-1]
    	ps.pose.orientation.x = ragdoll_ps.rotation_x[-1]
    	ps.pose.orientation.y = ragdoll_ps.rotation_y[-1]
    	ps.pose.orientation.z = ragdoll_ps.rotation_z[-1]
    	ps.pose.orientation.w = ragdoll_ps.rotation_w[-1]
    else:
	ps.pose.position.x = (world_ragdoll_ps.centers_x[0]+world_ragdoll_ps.centers_x[1])/2
        #ps.pose.position.y = (world_ragdoll_ps.centers_y[0]+world_ragdoll_ps.centers_y[1])/2
        ps.pose.position.y = 0
        ps.pose.position.z = (world_ragdoll_ps.centers_z[0]+world_ragdoll_ps.centers_z[1])/2
        ps.pose.orientation.x = ragdoll_ps.rotation_x[-1]
        ps.pose.orientation.y = ragdoll_ps.rotation_y[-1]
        ps.pose.orientation.z = ragdoll_ps.rotation_z[-1]
        ps.pose.orientation.w = ragdoll_ps.rotation_w[-1]

    self.object_ps = self.tf_listener.transformPose('base_link',ps)
    self.object_ps.pose.orientation.x = 0
    self.object_ps.pose.orientation.y = 0
    self.object_ps.pose.orientation.z = 0
    self.object_ps.pose.orientation.w = 1
    
    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'torso_link'
    self.torso_pose = self.tf_listener.transformPose('base_link',ps)
    
    ps.header.frame_id = 'world'

    for i in range(len(world_ragdoll_ps.frame_names)):
	if world_ragdoll_ps.frame_names[i] == 'ragdoll_lower_body_cog':
    	    ps.pose.position.x = world_ragdoll_ps.centers_x[i]
    	    ps.pose.position.y = world_ragdoll_ps.centers_y[i]
   	    ps.pose.position.z = world_ragdoll_ps.centers_z[i]
	    self.object_lower_body_ps = self.tf_listener.transformPose('base_link',ps)
	elif world_ragdoll_ps.frame_names[i] == 'ragdoll_l_thigh_cog':
            ps.pose.position.x = world_ragdoll_ps.centers_x[i]
            ps.pose.position.y = world_ragdoll_ps.centers_y[i]
            ps.pose.position.z = world_ragdoll_ps.centers_z[i]
            self.object_l_thigh_ps = self.tf_listener.transformPose('base_link',ps)
	elif world_ragdoll_ps.frame_names[i] == 'ragdoll_r_thigh_cog':
            ps.pose.position.x = world_ragdoll_ps.centers_x[i]
            ps.pose.position.y = world_ragdoll_ps.centers_y[i]
            ps.pose.position.z = world_ragdoll_ps.centers_z[i]
            self.object_r_thigh_ps = self.tf_listener.transformPose('base_link',ps)
	elif world_ragdoll_ps.frame_names[i] == 'ragdoll_upper_body_cog':
            ps.pose.position.x = world_ragdoll_ps.centers_x[i]
            ps.pose.position.y = world_ragdoll_ps.centers_y[i]
            ps.pose.position.z = world_ragdoll_ps.centers_z[i]
            self.object_upper_body_ps = self.tf_listener.transformPose('base_link',ps)
	elif world_ragdoll_ps.frame_names[i] == 'ragdoll_m1':
	    ps.pose.position.x = world_ragdoll_ps.centers_x[i]
            ps.pose.position.y = world_ragdoll_ps.centers_y[i]
            ps.pose.position.z = world_ragdoll_ps.centers_z[i]
	    ps.pose.orientation.x = world_ragdoll_ps.rotation_x[i]
	    ps.pose.orientation.y = world_ragdoll_ps.rotation_y[i]
	    ps.pose.orientation.z = world_ragdoll_ps.rotation_z[i]
	    ps.pose.orientation.w = world_ragdoll_ps.rotation_w[i]
            self.ragdoll_m1_ps = self.tf_listener.transformPose('base_link',ps)
	    self.ragdoll_m1 = world_ragdoll_ps.mass[i-2]
	elif world_ragdoll_ps.frame_names[i] == 'ragdoll_m2':
            ps.pose.position.x = world_ragdoll_ps.centers_x[i]
            ps.pose.position.y = world_ragdoll_ps.centers_y[i]
            ps.pose.position.z = world_ragdoll_ps.centers_z[i]
	    ps.pose.orientation.x = world_ragdoll_ps.rotation_x[i]
            ps.pose.orientation.y = world_ragdoll_ps.rotation_y[i]
            ps.pose.orientation.z = world_ragdoll_ps.rotation_z[i]
            ps.pose.orientation.w = world_ragdoll_ps.rotation_w[i]
            self.ragdoll_m2_ps = self.tf_listener.transformPose('base_link',ps)
	    self.ragdoll_m2 = world_ragdoll_ps.mass[i-2]
	elif world_ragdoll_ps.frame_names[i] == 'ragdoll_m3':
            ps.pose.position.x = world_ragdoll_ps.centers_x[i]
            ps.pose.position.y = world_ragdoll_ps.centers_y[i]
            ps.pose.position.z = world_ragdoll_ps.centers_z[i]
	    ps.pose.orientation.x = world_ragdoll_ps.rotation_x[i]
            ps.pose.orientation.y = world_ragdoll_ps.rotation_y[i]
            ps.pose.orientation.z = world_ragdoll_ps.rotation_z[i]
            ps.pose.orientation.w = world_ragdoll_ps.rotation_w[i]
            self.ragdoll_m3_ps = self.tf_listener.transformPose('base_link',ps)
	    self.ragdoll_m3 = world_ragdoll_ps.mass[i-2]
    

  def initCrona(self):
    import urdf_crona2

    self.robot_path = "/crona"
    self.skin_topic_list = rospy.get_param(self.base_path +
                                           self.robot_path +
                                           '/skin_list/' + self.opt.sensor)

    self.base_frame = rospy.get_param(self.base_path +
                                       self.robot_path +
                                       '/base_frame' )
    self.inertial_frame = rospy.get_param(self.base_path +
                                          self.robot_path +
                                          '/inertial_frame')
    self.skin_client = sc.TaxelArrayClient(self.skin_topic_list,
                                             self.base_frame,
                                             self.tf_listener)
    rospy.loginfo("RobotHapticState: Initialising CRONA haptic state publisher with the following skin topics: \n%s"
                  %str(self.skin_topic_list))

    rospy.loginfo("RobotHapticState: Initialising robot interface")
    if not self.opt.arm:
      rospy.logerr("RobotHapticState: No arm specified for CRONA")
      sys.exit()
    self.robot = urdf_crona2.URDFArm(self.tf_listener, base_link=self.base_frame)
    self.skins = []
    self.Jc = []

    # Push joint angles to the param server.
    if self.opt.arm in ['l', 'r']:
      arm_path = '/left'
      if self.opt.arm == 'r':
        arm_path = '/right'

    self.larm_joint_limits_max = rospy.get_param(self.base_path+self.robot_path+'/joint_limits'+'/torso'+'/max')+rospy.get_param(self.base_path+self.robot_path+'/joint_limits'+'/left'+'/max')
    self.larm_joint_limits_min = rospy.get_param(self.base_path+self.robot_path+'/joint_limits'+'/torso'+'/min')+rospy.get_param(self.base_path+self.robot_path+'/joint_limits'+'/left'+'/min')

    self.rarm_joint_limits_max = rospy.get_param(self.base_path+self.robot_path+'/joint_limits'+'/torso'+'/max')+rospy.get_param(self.base_path+self.robot_path+'/joint_limits'+'/right'+'/max')
    self.rarm_joint_limits_min = rospy.get_param(self.base_path+self.robot_path+'/joint_limits'+'/torso'+'/min')+rospy.get_param(self.base_path+self.robot_path+'/joint_limits'+'/right'+'/min')

    # Push the arm specific param to the location the controller looks.
    self.setControllerJointLimits(self.larm_joint_limits_max, self.larm_joint_limits_min, self.rarm_joint_limits_max, self.rarm_joint_limits_min)

  # Initialise publishers for the robot haptic state,
  # the current gripper pose, and a TF listener.
  # NB: The skin client and robot clients will have their own
  # publishers/subscribers specific to them.
  def initComms(self):
    if self.node_name != None:
      rospy.init_node(self.node_name)
    self.tf_listener = TransformListener()
    self.state_pub = rospy.Publisher('haptic_mpc/robot_state',
                                     haptic_msgs.RobotHapticState)
    self.gripper_pose_pub = rospy.Publisher('haptic_mpc/gripper_pose',
                                            geom_msgs.PoseStamped)


  ## Pushes the given joint limits to a known location on the param server. 
  # The control loads these on startup.
  def setControllerJointLimits(self, larm_joint_limits_max, larm_joint_limits_min, rarm_joint_limits_max, rarm_joint_limits_min):
    # Push the arm specific param to the location the controller looks.
    rospy.set_param(self.base_path + '/larm_joint_limits/max', larm_joint_limits_max)
    rospy.set_param(self.base_path + '/larm_joint_limits/min', larm_joint_limits_min)
    rospy.set_param(self.base_path + '/rarm_joint_limits/max', rarm_joint_limits_max)
    rospy.set_param(self.base_path + '/rarm_joint_limits/min', rarm_joint_limits_min)


  # Returns a header type with the current timestamp.
  # Does not set the frame_id
  def getMessageHeader(self):
    header = Header()
    header.stamp = rospy.get_rostime()
    return header

  def updateObjectJacobian(self):
    object_jacobian_pos = np.matrix([self.object_ps.pose.position.x,self.object_ps.pose.position.y,self.object_ps.pose.position.z]).reshape(3,1)
    #self.larm_object_Je = [self.larm_robot.kinematics.jacobian(self.larm_joint_angles,object_jacobian_pos)]
    self.larm_object_Je = [self.robot.larm_kinematics.jacobian(self.robot.torso_ja+self.larm_joint_angles,object_jacobian_pos)]
    self.larm_object_Je[0][:,6] = 0

    #self.rarm_object_Je = [self.rarm_robot.kinematics.jacobian(self.rarm_joint_angles,object_jacobian_pos)]
    self.rarm_object_Je = [self.robot.rarm_kinematics.jacobian(self.robot.torso_ja+self.rarm_joint_angles,object_jacobian_pos)]
    self.rarm_object_Je[0][:,6] = 0

    # Ragdoll M Jacobians
    try:
    	ragdoll_m2_pos = np.matrix([self.ragdoll_m2_ps.pose.position.x,self.ragdoll_m2_ps.pose.position.y,self.ragdoll_m2_ps.pose.position.z]).reshape(3,1)
    	#self.ragdoll_m2_Je = [self.rarm_robot.kinematics.jacobian(self.rarm_joint_angles,ragdoll_m2_pos)]
    	self.ragdoll_m2_Je = [self.robot.rarm_kinematics.jacobian(self.robot.torso_ja+self.rarm_joint_angles,ragdoll_m2_pos)]
    	self.ragdoll_m2_Je[0][:,6] = 0

    	ragdoll_m3_pos = np.matrix([self.ragdoll_m3_ps.pose.position.x,self.ragdoll_m3_ps.pose.position.y,self.ragdoll_m3_ps.pose.position.z]).reshape(3,1)
    	#self.ragdoll_m3_Je = [self.larm_robot.kinematics.jacobian(self.larm_joint_angles,ragdoll_m3_pos)]
    	self.ragdoll_m3_Je = [self.robot.larm_kinematics.jacobian(self.robot.torso_ja+self.larm_joint_angles,ragdoll_m3_pos)]
    	self.ragdoll_m3_Je[0][:,6] = 0
    except:
	ragdoll_m2_pos = np.matrix([0,0,0]).reshape(3,1)
	self.ragdoll_m2_Je = [np.matrix(np.zeros((6,7)))]
	ragdoll_m3_pos = np.matrix([0,0,0]).reshape(3,1)
	self.ragdoll_m3_Je = [np.matrix(np.zeros((6,7)))]
    
    # Friction Jacobians
    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'r_forearm_roll_link'
    ps.pose.position.z = 0.
    self.rarm_friction_p1_ps = self.tf_listener.transformPose('base_link',ps)
    rarm_friction_jacobian_pos = np.matrix([self.rarm_friction_p1_ps.pose.position.x,self.rarm_friction_p1_ps.pose.position.y,self.rarm_friction_p1_ps.pose.position.z]).reshape(3,1)
    #self.rarm_friction_Je1 = [self.rarm_robot.kinematics.jacobian(self.rarm_joint_angles, rarm_friction_jacobian_pos)]
    self.rarm_friction_Je1 = [self.robot.rarm_kinematics.jacobian(self.robot.torso_ja+self.rarm_joint_angles, rarm_friction_jacobian_pos)]
    self.rarm_friction_Je1[0][:,6] = 0

    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'r_forearm_roll_link'
    ps.pose.position.z = -0.2
    self.rarm_friction_p2_ps = self.tf_listener.transformPose('base_link',ps)
    rarm_friction_jacobian_pos = np.matrix([self.rarm_friction_p2_ps.pose.position.x,self.rarm_friction_p2_ps.pose.position.y,self.rarm_friction_p2_ps.pose.position.z]).reshape(3,1)
    #self.rarm_friction_Je2 = [self.rarm_robot.kinematics.jacobian(self.rarm_joint_angles, rarm_friction_jacobian_pos)]
    self.rarm_friction_Je2 = [self.robot.rarm_kinematics.jacobian(self.robot.torso_ja+self.rarm_joint_angles, rarm_friction_jacobian_pos)]
    self.rarm_friction_Je2[0][:,6] = 0

    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'r_forearm_roll_link'
    ps.pose.position.z = -0.1
    ps.pose.position.y = 0.1
    self.rarm_friction_p3_ps = self.tf_listener.transformPose('base_link',ps)
    rarm_friction_jacobian_pos = np.matrix([self.rarm_friction_p3_ps.pose.position.x,self.rarm_friction_p3_ps.pose.position.y,self.rarm_friction_p3_ps.pose.position.z]).reshape(3,1)
    #self.rarm_friction_Je3 = [self.rarm_robot.kinematics.jacobian(self.rarm_joint_angles, rarm_friction_jacobian_pos)]
    self.rarm_friction_Je3 = [self.robot.rarm_kinematics.jacobian(self.robot.torso_ja+self.rarm_joint_angles, rarm_friction_jacobian_pos)]
    self.rarm_friction_Je3[0][:,6] = 0

    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'l_forearm_roll_link'
    ps.pose.position.z = 0.
    self.larm_friction_p1_ps = self.tf_listener.transformPose('base_link',ps)
    larm_friction_jacobian_pos = np.matrix([self.larm_friction_p1_ps.pose.position.x,self.larm_friction_p1_ps.pose.position.y,self.larm_friction_p1_ps.pose.position.z]).reshape(3,1)
    #self.larm_friction_Je1 = [self.larm_robot.kinematics.jacobian(self.larm_joint_angles, larm_friction_jacobian_pos)]
    self.larm_friction_Je1 = [self.robot.larm_kinematics.jacobian(self.robot.torso_ja+self.larm_joint_angles, larm_friction_jacobian_pos)]
    self.larm_friction_Je1[0][:,6] = 0
 
    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'l_forearm_roll_link'
    ps.pose.position.z = -0.2
    self.larm_friction_p2_ps = self.tf_listener.transformPose('base_link',ps)
    larm_friction_jacobian_pos = np.matrix([self.larm_friction_p2_ps.pose.position.x,self.larm_friction_p2_ps.pose.position.y,self.larm_friction_p2_ps.pose.position.z]).reshape(3,1)
    #self.larm_friction_Je2 = [self.larm_robot.kinematics.jacobian(self.larm_joint_angles, larm_friction_jacobian_pos)]
    self.larm_friction_Je2 = [self.robot.larm_kinematics.jacobian(self.robot.torso_ja+self.larm_joint_angles, larm_friction_jacobian_pos)]
    self.larm_friction_Je2[0][:,6] = 0

    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'l_forearm_roll_link'
    ps.pose.position.z = -0.1
    ps.pose.position.y = 0.1
    self.larm_friction_p3_ps = self.tf_listener.transformPose('base_link',ps)
    larm_friction_jacobian_pos = np.matrix([self.larm_friction_p3_ps.pose.position.x,self.larm_friction_p3_ps.pose.position.y,self.larm_friction_p3_ps.pose.position.z]).reshape(3,1)
    #self.larm_friction_Je3 = [self.larm_robot.kinematics.jacobian(self.larm_joint_angles, larm_friction_jacobian_pos)]
    self.larm_friction_Je3 = [self.robot.larm_kinematics.jacobian(self.robot.torso_ja+self.larm_joint_angles, larm_friction_jacobian_pos)]
    self.larm_friction_Je3[0][:,6] = 0

    


  # Updates the stored end effector Jacobian and forearm Jacobian from the current joint angles
  # and end effector/forearm positions
  def updateEndEffectorJacobian(self):
    self.larm_Je = [self.robot.larm_kinematics.jacobian(self.robot.torso_ja+self.larm_joint_angles, self.larm_end_effector_position)]
    self.rarm_Je = [self.robot.rarm_kinematics.jacobian(self.robot.torso_ja+self.rarm_joint_angles, self.rarm_end_effector_position)]

    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'r_forearm_roll_link'
    ps.pose.position.y = 0.
    ps.pose.position.z = -0.209
    self.rarm_f_p = self.tf_listener.transformPose('base_link',ps)
    rarm_jacobian_pos = np.matrix([self.rarm_f_p.pose.position.x,self.rarm_f_p.pose.position.y,self.rarm_f_p.pose.position.z]).reshape(3,1)
    self.rforearm_J = [self.robot.rarm_kinematics.jacobian(self.robot.torso_ja+self.rarm_joint_angles,rarm_jacobian_pos)]
    self.rforearm_J[0][:,6] = 0
 
    ps = geom_msgs.PoseStamped()
    ps.header.frame_id = 'l_forearm_roll_link'
    ps.pose.position.y = 0.
    ps.pose.position.z = -0.209
    self.larm_f_p = self.tf_listener.transformPose('base_link',ps)
    larm_jacobian_pos = np.matrix([self.larm_f_p.pose.position.x,self.larm_f_p.pose.position.y,self.larm_f_p.pose.position.z]).reshape(3,1)
    self.lforearm_J = [self.robot.larm_kinematics.jacobian(self.robot.torso_ja+self.larm_joint_angles, larm_jacobian_pos)]
    self.lforearm_J[0][:,6] = 0

  ## Compute contact Jacobians based on the provided taxel array dictionary
  # @param skin_data Dictionary containing taxel array messages indexed by topic name
  def updateContactJacobians(self, skin_data):
    # loc_l = list of taxel locations relative the "torso_lift_link" frame.
    # jt_l = list of joints beyond which the jacobian columns are zero.
    # loc_l. jt_l from skin client.
    Jc_l = []
    loc_l, jt_l = self.getTaxelLocationAndJointList(skin_data)

    if len(loc_l) != len(jt_l):
      rospy.logfatal("Haptic State Publisher: Dimensions don't match. %s, %s" % (len(loc_l), len(jt_l)))
      sys.exit()

    for jt_li, loc_li in it.izip(jt_l, loc_l):
      Jc = self.robot.larm_kinematics.jacobian(self.larm_joint_angles, loc_li)
      Jc[:, jt_li+1:] = 0.0
      Jc = Jc[0:3, 0:len(self.larm_joint_stiffness)] # trim the jacobian to suit the number of DOFs.
      Jc_l.append(Jc)
    self.Jc = Jc_l

  ## Returns a Pose object for the torso pose in the stated inertial frame
  def updateTorsoPose(self):
    # Get the transformation from the desired frame to current frame
    self.tf_listener.waitForTransform(self.inertial_frame, self.torso_frame,
                                      rospy.Time(), rospy.Duration(10.0))
    t1, q1 = self.tf_listener.lookupTransform(self.inertial_frame,
                                              self.torso_frame,
                                              rospy.Time(0))
    torso_pose = geom_msgs.Pose()
    torso_pose.position = geom_msgs.Point(*t1)
    torso_pose.orientation = geom_msgs.Quaternion(*q1)
    return torso_pose

  ## Store latest joint states from the specified robot class
  # @var joint_names: Joint names
  # @var joint_angles: Joint angles
  # @var joint_velocities: Joint velocities
  # @var joint_stiffness: Joint stiffness
  # @var joint_damping: Joint damping
  # @var q_des: Desired joint angles
  def updateJointStates(self):
    #rospy.sleep(1.)
    #self.larm_joint_names = self.larm_robot.get_joint_names()
    self.larm_joint_names = self.robot.larm_joint_names_list
    #self.larm_joint_angles = self.larm_robot.get_joint_angles()
    self.larm_joint_angles = self.robot.larm_ja
    #self.larm_joint_stiffness, self.larm_joint_damping = self.larm_robot.get_joint_impedance()
    self.larm_joint_stiffness, self.larm_joint_damping = self.robot.larm_kp, self.robot.larm_kd
    #self.larm_joint_velocities = self.larm_robot.get_joint_velocities()
    self.larm_joint_velocities = self.robot.larm_jv
    #larm_q_des = self.larm_robot.get_ep()
    larm_q_des = self.robot.ep[:7]
    if larm_q_des != None:
      self.larm_desired_joint_angles = larm_q_des

    #self.rarm_joint_names = self.rarm_robot.get_joint_names()
    self.rarm_joint_names = self.robot.rarm_joint_names_list
    #self.rarm_joint_angles = self.rarm_robot.get_joint_angles()
    self.rarm_joint_angles = self.robot.rarm_ja
    #self.rarm_joint_stiffness, self.rarm_joint_damping = self.rarm_robot.get_joint_impedance()
    self.rarm_joint_stiffness, self.rarm_joint_damping = self.robot.rarm_kp, self.robot.rarm_kd
    #self.rarm_joint_velocities = self.rarm_robot.get_joint_velocities()
    self.rarm_joint_velocities = self.robot.rarm_jv
    #rarm_q_des = self.rarm_robot.get_ep()
    rarm_q_des = self.robot.ep[7:]
    if rarm_q_des != None:
      self.rarm_desired_joint_angles = rarm_q_des

    self.rarm_desired_joint_angles = [self.larm_desired_joint_angles[0]]+list(self.rarm_desired_joint_angles)
    #print self.larm_joint_angles
    #print self.rarm_joint_angles

  # Compute and store the end effector position, orientation, and jacobian
  # from the current joint angles.
  def updateEndEffectorPose(self):
    larm_pos, larm_rot = self.robot.larm_kinematics.FK(self.robot.torso_ja+self.larm_joint_angles)
    #print "larm_rot: ",larm_rot
    self.larm_end_effector_position = larm_pos
    self.larm_end_effector_orient_cart = larm_rot
    self.larm_end_effector_orient_quat = tr.matrix_to_quaternion(larm_rot)

    ###### Hack to fix crona2 orientation control issue ########
    ## when the arm is orientated past 120 degrees vertically, some quaternion values switch from positive to negative and vice versa
    ## this fix switches them back to positive or negative
    if self.larm_end_effector_orient_quat[1] > 0.86 and self.larm_end_effector_orient_quat[3] < -0.1:
	#print "Switched!"
	self.larm_end_effector_orient_quat[1] *= -1
	self.larm_end_effector_orient_quat[3] *= -1

    rarm_pos, rarm_rot = self.robot.rarm_kinematics.FK(self.robot.torso_ja+self.rarm_joint_angles)
    self.rarm_end_effector_position = rarm_pos
    self.rarm_end_effector_orient_cart = rarm_rot
    self.rarm_end_effector_orient_quat = tr.matrix_to_quaternion(rarm_rot)

    if self.rarm_end_effector_orient_quat[1] > 0.86 and self.rarm_end_effector_orient_quat[3] < -0.1:
        #print "Switched!"
        self.rarm_end_effector_orient_quat[1] *= -1
        self.rarm_end_effector_orient_quat[3] *= -1
    
    '''
    f_pos, f_rot = self.larm_robot_forearm.kinematics.FK(self.larm_joint_angles[:-1])
    self.lforearm_position = f_pos
    self.lforearm_orient_cart = f_rot
    self.lforearm_orient_quat = tr.matrix_to_quaternion(f_rot)

    f_pos, f_rot = self.rarm_robot_forearm.kinematics.FK(self.rarm_joint_angles[:-1])
    self.rforearm_position = f_pos
    self.rforearm_orient_cart = f_rot
    self.rforearm_orient_quat = tr.matrix_to_quaternion(f_rot)
    '''

  ## Returns a list of taxel locations and a list of joint numbers after which the
  # joint torque will have no effect on the contact force
  # @param skin_data Dictionary of TaxelArrays indexed by topic
  # @retval locations List of taxel locations where a force is present
  # @retval joint_nums List of joints after which the joint torque will have no effect on the contact force 
  # @return These arrays will both be the same length (as the joint number corresponds 
  def getTaxelLocationAndJointList(self, skin_data):
    locations = []
    joint_nums = []
    
    for ta_msg in skin_data.values():
      # Get points list
      ta_locs = self.skin_client.getContactLocationsFromTaxelArray(ta_msg)
      # Create list of joints beyond which the joint torque has no effect on contact force
      ta_jts = []
      for contact_index in range(len(ta_msg.centers_x)):
        jt_num = len(self.larm_joint_angles)-1 # Index of last joint, 0 indexed.
        
        # Verify we have the same number of link names as taxel contacts If not, make no assumptions about joints.
        if len(ta_msg.link_names) >= len(ta_msg.centers_x):
          link_name = ta_msg.link_names[contact_index]
        
          # Iterate over the known joint names looking for the link this is associated with.
          # NB: links should be named based on their joint. 
          # NB: 
          for idx, joint_name in enumerate(self.larm_joint_names):
            if joint_name[:-6] in link_name: 
              jt_num = idx
              break # 
          
        ta_jts.append(jt_num)
          
      # Attach these lists to the end of the global list (incorporating multiple taxel arrays)
      locations.extend(ta_locs)
      joint_nums.extend(ta_jts)
      
    return locations, joint_nums

  ## Modify taxel data for PR2 specific situations
  # TODO: Survy to implement selective taxel ignoring.
  # @param skin_data Dictionary containing taxel array messages indexed by topic name
  def modifyPR2Taxels(self, skin_data):
    #print "modifyPR2Taxels"
    return skin_data

  ## Modifies data from the taxel array based on robot specific configurations.
  # An example of this is ignoring the PR2 wrist taxels when the wrist
  # is near its joint limit as the wrist itself will trigger the taxel.
  # @param skin_data Dict containing taxel array messages indexed by topic name
  # @return skin_data Modified dictionary containing taxel array messages
  def modifyRobotSpecificTaxels(self, skin_data):
    if self.opt.robot == 'pr2':
      return self.modifyPR2Taxels(skin_data)
    return skin_data # If this is running on a different robot, don't modify the data.

  ## Calls all the sub-component updates
  def updateHapticState(self):
    self.updateJointStates()
    #self.torso_pose = self.updateTorsoPose()
    self.updateEndEffectorPose()
    self.updateEndEffectorJacobian()
    if self.opt.behavior != 'lifting_no_obj_data':
    	self.transform_object_state()
    	self.updateObjectJacobian()
	self.get_force_moving_average()
    # Skin sensor calculations.
    # Get the latest skin data from the skin client
    skin_data = self.skin_client.getTrimmedSkinData()
    full_skin_data = self.skin_client.getSkinData()
    # Trim skin_data based on specific robot state (eg wrist configuration).
    skin_data = self.modifyRobotSpecificTaxels(skin_data)
    self.updateContactJacobians(skin_data)
    # Add the list of  TaxelArray messages to the message
    self.skins = skin_data.values()
    self.full_skins = full_skin_data.values()
    
  ## Build the haptic state message data structure
  # @return haptic_state_msg Haptic State message object containing relevant data 
  def getHapticStateMessage(self):
    # Update all haptic state data (jacobians etc)
    self.updateHapticState()

    msg = haptic_msgs.RobotHapticState()

    msg.header = self.getMessageHeader()
    msg.header.frame_id = self.base_frame

    # TODO Locking on data? - check these are copies.
    # Joint states
#    self.updateJointStates()
   
    for i in range(len(self.larm_joint_names)):
	self.larm_joint_names[i] = self.larm_joint_names[i].encode('utf-8')
    for i in range(len(self.rarm_joint_names)):
        self.rarm_joint_names[i] = self.rarm_joint_names[i].encode('utf-8')
   
    msg.larm_joint_names = self.larm_joint_names
    msg.larm_joint_angles = self.robot.torso_ja+self.larm_joint_angles
    msg.larm_desired_joint_angles = self.larm_desired_joint_angles
    msg.larm_joint_velocities = self.larm_joint_velocities
    msg.larm_joint_stiffness = self.robot.torso_kp+self.larm_joint_stiffness
    msg.larm_joint_damping = self.larm_joint_damping
    msg.larm_joint_torques = self.larm_jt
    
    msg.rarm_joint_names = self.rarm_joint_names
    msg.rarm_joint_angles = self.robot.torso_ja+self.rarm_joint_angles
    msg.rarm_desired_joint_angles = self.rarm_desired_joint_angles
    msg.rarm_joint_velocities = self.rarm_joint_velocities
    msg.rarm_joint_stiffness = self.robot.torso_kp+self.rarm_joint_stiffness
    msg.rarm_joint_damping = self.rarm_joint_damping
    msg.rarm_joint_torques = self.rarm_jt

    msg.torso_pose = self.torso_pose.pose #self.updateTorsoPose()
    msg.torso_joint_torque = self.torso_jt

    # End effector calculations
#    self.updateEndEffectorPose()
    msg.l_hand_pose.position = geom_msgs.Point(*(self.larm_end_effector_position.A1))
    msg.l_hand_pose.orientation = geom_msgs.Quaternion(*self.larm_end_effector_orient_quat)

    msg.r_hand_pose.position = geom_msgs.Point(*(self.rarm_end_effector_position.A1))
    msg.r_hand_pose.orientation = geom_msgs.Quaternion(*self.rarm_end_effector_orient_quat)

    #msg.lforearm_pose.position = geom_msgs.Point(*(self.lforearm_position.A1))
    #msg.lforearm_pose.orientation = geom_msgs.Quaternion(*self.lforearm_orient_quat)
    msg.lforearm_pose.position = self.larm_f_p.pose.position
    msg.lforearm_pose.orientation = self.larm_f_p.pose.orientation

    msg.rforearm_pose.position = self.rarm_f_p.pose.position
    msg.rforearm_pose.orientation = self.rarm_f_p.pose.orientation

    #msg.object_lower_body_pose.position = self.object_lower_body_ps.pose.position
    #msg.object_l_thigh_pose.position = self.object_l_thigh_ps.pose.position
    #msg.object_r_thigh_pose.position = self.object_r_thigh_ps.pose.position
    #msg.object_upper_body_pose.position = self.object_upper_body_ps.pose.position 

    msg.ragdoll_m1 = self.ragdoll_m1
    msg.ragdoll_m2 = self.ragdoll_m2
    msg.ragdoll_m3 = self.ragdoll_m3

    msg.ragdoll_m1_pose = self.ragdoll_m1_ps.pose
    msg.ragdoll_m2_pose = self.ragdoll_m2_ps.pose
    msg.ragdoll_m3_pose = self.ragdoll_m3_ps.pose

    msg.larm_friction_p1_pose.position = self.larm_friction_p1_ps.pose.position
    msg.larm_friction_p2_pose.position = self.larm_friction_p2_ps.pose.position
    msg.larm_friction_p3_pose.position = self.larm_friction_p3_ps.pose.position
    msg.rarm_friction_p1_pose.position = self.rarm_friction_p1_ps.pose.position
    msg.rarm_friction_p2_pose.position = self.rarm_friction_p2_ps.pose.position
    msg.rarm_friction_p3_pose.position = self.rarm_friction_p3_ps.pose.position

#    self.updateEndEffectorJacobian()
    msg.larm_end_effector_jacobian = self.ma_to_m.matrixListToMultiarray(self.larm_Je)
    msg.rarm_end_effector_jacobian = self.ma_to_m.matrixListToMultiarray(self.rarm_Je)
    msg.lforearm_jacobian = self.ma_to_m.matrixListToMultiarray(self.lforearm_J)
    msg.rforearm_jacobian = self.ma_to_m.matrixListToMultiarray(self.rforearm_J)

    msg.object_pose.position = self.object_ps.pose.position
    msg.object_pose.orientation = self.object_ps.pose.orientation
    msg.larm_object_jacobian = self.ma_to_m.matrixListToMultiarray(self.larm_object_Je)
    msg.rarm_object_jacobian = self.ma_to_m.matrixListToMultiarray(self.rarm_object_Je)

    msg.larm_friction_jacobian1 = self.ma_to_m.matrixListToMultiarray(self.larm_friction_Je1)
    msg.larm_friction_jacobian2 = self.ma_to_m.matrixListToMultiarray(self.larm_friction_Je2)
    msg.larm_friction_jacobian3 = self.ma_to_m.matrixListToMultiarray(self.larm_friction_Je3)
    msg.rarm_friction_jacobian1 = self.ma_to_m.matrixListToMultiarray(self.rarm_friction_Je1)
    msg.rarm_friction_jacobian2 = self.ma_to_m.matrixListToMultiarray(self.rarm_friction_Je2)
    msg.rarm_friction_jacobian3 = self.ma_to_m.matrixListToMultiarray(self.rarm_friction_Je3)

    msg.ragdoll_m2_jacobian = self.ma_to_m.matrixListToMultiarray(self.ragdoll_m2_Je)
    #msg.ragdoll_m2_jacobian = self.ma_to_m.matrixListToMultiarray(np.matrix([0]))
    msg.ragdoll_m3_jacobian = self.ma_to_m.matrixListToMultiarray(self.ragdoll_m3_Je)
    #msg.ragdoll_m3_jacobian = self.ma_to_m.matrixListToMultiarray(np.matrix([0]))

    msg.larm_object_force = geom_msgs.Vector3(self.lforearm_ma_force_x,self.lforearm_ma_force_y,self.lforearm_ma_force_z)
    msg.rarm_object_force = geom_msgs.Vector3(self.rforearm_ma_force_x,self.rforearm_ma_force_y,self.rforearm_ma_force_z)
    msg.object_dist_force = geom_msgs.Vector3(*self.object_dist_force)

#    # Skin sensor calculations.
#    # Get the latest skin data from the skin client
#    skin_data = self.skin_client.getTrimmedSkinData()
#    # Trim skin_data based on specific robot state (eg wrist configuration).
#    skin_data = self.modifyRobotSpecificTaxels(skin_data)
#    # Add the list of  TaxelArray messages to the message
#    msg.skins = skin_data.values()
#    self.updateContactJacobians(skin_data)
# Add the list of  TaxelArray messages to the message
    msg.skins = self.skins
    msg.contact_jacobians = self.ma_to_m.matrixListToMultiarray(self.Jc)
    
    return msg
  
  ## Build and publish the haptic state message.
  def publishRobotState(self):
    msg = self.getHapticStateMessage()

    # Publish the newly formed state message
    for i in range(len(msg.larm_joint_names)):
	msg.larm_joint_names[i] = str(msg.larm_joint_names[i]) 
    self.state_pub.publish(msg)

    # Publish gripper pose for debug purposes
    ps_msg = geom_msgs.PoseStamped()
    ps_msg.header = self.getMessageHeader()
    ps_msg.header.frame_id = self.base_frame

    ps_msg.pose.position = geom_msgs.Point(*(self.larm_end_effector_position.A1))
    ps_msg.pose.orientation = geom_msgs.Quaternion(*self.larm_end_effector_orient_quat)
  
    self.gripper_pose_pub.publish(ps_msg)
    

  ## Handler for Ctrl-C signals. Some of the ROS spin loops don't respond well to
  # Ctrl-C without this. 
  def signal_handler(self, signal, frame):
    print 'Ctrl+C pressed - exiting'
    sys.exit(0)

  ## Start the state publisher
  def start(self):
    rospy.sleep(5.)
    signal.signal(signal.SIGINT, self.signal_handler) # Catch Ctrl-Cs
    
    rospy.loginfo("RobotHapticState: Starting Robot Haptic State publisher")
    rate = rospy.Rate(self.rate) # 100Hz, nominally.

    # Blocking sleep to prevent the node publishing until joint states
    # are read by the robot client.
    rospy.loginfo("RobotHapticState: Waiting for robot state")
    #joint_stiffness, joint_damping = self.larm_robot.get_joint_impedance()
   
    joint_stiffness, joint_damping = self.robot.larm_kp, self.robot.larm_kd
    while (self.robot.larm_ja == None or
           self.robot.larm_jv == None or
           joint_stiffness == None):
      joint_stiffness, joint_damping = self.robot.larm_kp, self.robot.larm_kd
      rate.sleep()
    

    rospy.loginfo("RobotHapticState: Got robot state")

    if self.robot.ep == None:
      rospy.loginfo("RobotHapticState: Setting desired joint angles to current joint_angles")
      # Use desired joint positions instead of actual
      #self.larm_robot.set_ep(self.larm_robot.get_joint_angles())
      self.robot.set_ep([self.init_torso_desired_ja]+list(self.init_larm_desired_ja)+list(self.init_rarm_desired_ja))

    #if self.rarm_robot.get_ep() == None:
    #  rospy.loginfo("RobotHapticState: Setting desired joint angles to current joint_angles")
      # Use desired joint positions instead of actual
      #self.rarm_robot.set_ep(self.rarm_robot.get_joint_angles()[1:])
    #  self.rarm_robot.set_ep(self.init_rarm_desired_ja)

    rospy.loginfo("RobotHapticState: Starting publishing")
    while self.mpc_terminate == 0:
      self.publishRobotState()
#      rospy.spin() # Blocking spin for debug/dev purposes
      rate.sleep()


if __name__ == "__main__":
  # Parse an options list specifying robot type
  import optparse
  p = optparse.OptionParser()
  haptic_mpc_util.initialiseOptParser(p)
  opt = haptic_mpc_util.getValidInput(p)

  if not opt.robot or not opt.sensor or not opt.arm:
    p.error("Robot haptic state publisher requires a specified robot, sensor, AND arm to use.")

  robot_state = RobotHapticStateServer(opt, "robot_haptic_state_server")
  robot_state.start()
