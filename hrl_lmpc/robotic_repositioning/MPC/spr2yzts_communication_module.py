#/usr/bin/env python

# Software License Agreement (New BSD License)
#
# Copyright (c) 2016, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of Georgia Tech nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE GEORGIA TECH RESEARCH CORPORATION BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Kevin Chow
# Healthcare Robotics Laboratory



################# SPR2YZTS Communication Module #################
# This script provides the low level control interface for the SPR2YZTS (two prismatic robots with y/z translation positioned to be placed on the thigh and shin).  If a different robot is used, then a similar script to this should be written.

# Classes:
# 	SPR2YZTSComNode
#################################################################



import roslib
roslib.load_manifest('sttr_behaviors')
import rospy
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sttr_msgs.msg import WrenchArray
from hrl_msgs.msg import FloatArray
from geometry_msgs.msg import Pose
import numpy as np
import math

### Communication Module ###

class SPR2YZTSComNode(object):
    def __init__(self):
        rospy.init_node('SPR2YZTSComNode')

        # Initialize status variables
        self.robot_ja = [0.,0.,0.,0.,0.,0.]                 # y, z, theta
        self.robot_desired_ja = [0.,0.,0.,0.,0.,0.]
        self.joint_error = [0.,0.,0.,0.,0.,0.]
        n = 20 # size of moving average for force/torque arrays
        self.force_y_array_1 = [0] * n
        self.force_z_array_1 = [0] * n
        self.torque_y_array_1 = [0] * n                    
        self.force_y_array_2 = [0] * n
        self.force_z_array_2 = [0] * n
        self.torque_y_array_2 = [0] * n
        self.max_velocity = 100 # maximum velocity of the low level controller in m/s
        self.time_from_start = 0.1 # s
        self.setup_robot_MPC_state()
        self.y1 = self.y1_initial
        self.y2 = self.y2_initial
        self.z1 = 0.
        self.z2 = 0.
        self.knee_y = self.L_t
        self.knee_z = 0.
        # assuming that the robots start out contacting the bottom surface of the leg
        self.z1_initial = -self.r_t
        self.z2_initial = -self.r_s

        self.ragdoll_leg_0_cart = np.matrix([0,0,0]).T
        self.ragdoll_leg_0_COM_cart = np.matrix([0,0,0]).T
        self.ragdoll_leg_1_cart = np.matrix([0,0,0]).T
        self.ragdoll_leg_1_COM_cart = np.matrix([0,0,0]).T
       

        # Subscribers
        rospy.Subscriber('/simple_prismatic_robot_controller_2/state', JointTrajectoryControllerState, self.update_robot_kinematic_state)
        rospy.Subscriber('/gazebo/sprjt', WrenchArray, self.update_ft)
        rospy.Subscriber('/gazebo/leg_angles', FloatArray, self.update_actual_leg_angle)
       
        rospy.Subscriber('/gazebo/ragdoll_leg_0_cart', Pose, self.update_leg_0_cart)
        rospy.Subscriber('/gazebo/ragdoll_leg_0_COM_cart', Pose, self.update_leg_0_COM_cart)
        rospy.Subscriber('/gazebo/ragdoll_leg_1_cart', Pose, self.update_leg_1_cart)
        rospy.Subscriber('/gazebo/ragdoll_leg_1_COM_cart', Pose, self.update_leg_1_COM_cart)

        # Publishers
        self.move_pub = rospy.Publisher('/simple_prismatic_robot_controller_2/command', JointTrajectory)

        # calculate actual angle using vectors
    def calculate_actual_leg_angles(self):
        thigh_vector = self.ragdoll_leg_0_COM_cart-self.ragdoll_leg_0_cart
        shank_vector = self.ragdoll_leg_1_COM_cart-self.ragdoll_leg_1_cart
        norm_vector = np.matrix([0, 0, 1]).T # finding the angle between the vector and the x-y plane, so using z vector as the norm
	thigh_angle = math.asin((norm_vector.T*thigh_vector)/(np.linalg.norm(norm_vector)*np.linalg.norm(thigh_vector)))
        shank_angle = math.asin((norm_vector.T*shank_vector)/(np.linalg.norm(norm_vector)*np.linalg.norm(shank_vector)))

        self.actual_theta_t = thigh_angle
        self.actual_theta_s = shank_angle

        self.knee_y = self.L_t*math.cos(self.actual_theta_t)
        self.knee_z = self.L_t*math.sin(self.actual_theta_t)#+self.r_t
        self.d1 = math.sqrt((self.y1-self.r_t*math.sin(self.actual_theta_t))**2+(self.z1+self.r_t*math.cos(self.actual_theta_t))**2)
        self.d2 = math.sqrt((self.y2-self.r_s*math.sin(self.actual_theta_s)-self.knee_y)**2+(self.z2+self.r_s*math.cos(self.actual_theta_s)-self.knee_z)**2)

    def update_leg_0_cart(self, msg):
        self.ragdoll_leg_0_cart = np.matrix([msg.position.x, msg.position.y, msg.position.z]).T

    def update_leg_0_COM_cart(self, msg):
        self.ragdoll_leg_0_COM_cart = np.matrix([msg.position.x, msg.position.y, msg.position.z]).T

    def update_leg_1_cart(self, msg):
        self.ragdoll_leg_1_cart = np.matrix([msg.position.x, msg.position.y, msg.position.z]).T

    def update_leg_1_COM_cart(self, msg):
        self.ragdoll_leg_1_COM_cart = np.matrix([msg.position.x, msg.position.y, msg.position.z]).T


    def update_robot_kinematic_state(self, msg):
        # call back function for spr controller state subscriber 
        self.robot_ja = [msg.actual.positions[0], msg.actual.positions[1], msg.actual.positions[2],msg.actual.positions[3], msg.actual.positions[4], msg.actual.positions[5]]
        self.robot_desired_ja = [msg.desired.positions[0], msg.desired.positions[1], msg.desired.positions[2],msg.desired.positions[3], msg.desired.positions[4], msg.desired.positions[5]]
        self.joint_error = (np.array(self.robot_desired_ja)-np.array(self.robot_ja)).tolist()

        # y and z calculations do not account for the radius of the leg (small inaccuracy)
        # EDIT: I think they do now
        self.q1 = self.robot_ja[0]
        # sensors return robot ja in simulation frame, so we subtract the y coordinate to transform it to the model frame
        # to clear up any confusion about y1, z1, y2, z2: they are coordinates relative an inertial frame with origin at the hip joint.  
        self.y1 = self.y1_initial-self.robot_ja[1]        # this is relative to the hip joint
        self.z1 = self.z1_initial+self.robot_ja[2]        # this is relative to the hip joint

        self.q2 = self.robot_ja[3]
        self.y2 = self.y2_initial-self.robot_ja[4]        # this is relative to the hip joint
        self.z2 = self.z2_initial+self.robot_ja[5]        # this is relative to the knee joint (actually hip joint)

        self.q_eq1 = self.robot_desired_ja[0]
        self.y_eq1 = self.y1_initial-self.robot_desired_ja[1]
        self.z_eq1 = self.z1_initial+self.robot_desired_ja[2]

        self.q_eq2 = self.robot_desired_ja[3]
        self.y_eq2 = self.y2_initial-self.robot_desired_ja[4]
        self.z_eq2 = self.z2_initial+self.robot_desired_ja[5]

    def update_ft(self, msg):
        # call back function for sprjt subscriber
        # using a moving average filter to calculate the current forces and torques
        self.force_y_array_1.append(msg.force_y[1]) # using the 1st and 4th indicies from the message because these are from the second joint on the SPRs rather than the rotating frame
        self.force_z_array_1.append(msg.force_z[1])
        self.torque_y_array_1.append(msg.torque_y[1])
        self.force_y_array_2.append(msg.force_y[4])
        self.force_z_array_2.append(msg.force_z[4])
        self.torque_y_array_2.append(msg.torque_y[4])
        del self.force_y_array_1[0]
        del self.force_z_array_1[0]
        del self.torque_y_array_1[0]
        del self.force_y_array_2[0]
        del self.force_z_array_2[0]
        del self.torque_y_array_2[0]
        self.force_y_1 = sum(self.force_y_array_1)/len(self.force_y_array_1)
        self.force_z_1 = sum(self.force_z_array_1)/len(self.force_z_array_1)
        self.torque_y_1 = sum(self.torque_y_array_1)/len(self.torque_y_array_1)
        self.force_y_2 = sum(self.force_y_array_2)/len(self.force_y_array_2)
        self.force_z_2 = sum(self.force_z_array_2)/len(self.force_z_array_2)
        self.torque_y_2 = sum(self.torque_y_array_2)/len(self.torque_y_array_2)

        self.F_a1_1 = -self.force_z_1*math.sin(self.actual_theta_t)+self.force_y_1*math.cos(self.actual_theta_t)
        self.F_a2_1 = -self.force_z_1*math.cos(self.actual_theta_t)-self.force_y_1*math.sin(self.actual_theta_t)
        self.F_b1_2 = -self.force_z_2*math.sin(self.actual_theta_s)+self.force_y_2*math.cos(self.actual_theta_s)
        self.F_b2_2 = -self.force_z_2*math.cos(self.actual_theta_s)-self.force_y_2*math.sin(self.actual_theta_s)
        self.F_y_1 = self.force_y_1 # don't need to take the negation (reaction force) b/c we are transforming from simulation coordinate frame (y positive toward the upper body) to model coordinate frame (y positive toward feet)
        self.F_z_1 = -self.force_z_1
        self.F_y_2 = self.force_y_2
        self.F_z_2 = -self.force_z_2

    def update_actual_leg_angle(self, msg):
        # call back function leg angles subscriber
        #self.actual_theta_t = (msg.data[0]+7)*(math.pi/180)
        #self.actual_theta_s = (msg.data[1]+7)*(math.pi/180)
        #print "Actual theta t: ",self.actual_theta_t
        #print "Actual theta s: ",self.actual_theta_s

        self.calculate_actual_leg_angles()

        #self.knee_y = self.L_t*math.cos(self.actual_theta_t)
        #self.knee_z = self.L_t*math.sin(self.actual_theta_t)#+self.r_t
        #self.d1 = math.sqrt((self.y1-self.r_t*math.sin(self.actual_theta_t))**2+(self.z1+self.r_t*math.cos(self.actual_theta_t))**2)
        #self.d2 = math.sqrt((self.y2-self.r_s*math.sin(self.actual_theta_s)-self.knee_y)**2+(self.z2+self.r_s*math.cos(self.actual_theta_s)-self.knee_z)**2)
  
    def setup_robot_MPC_state(self):
        # load parameters from server
        base_path = 'robot_MPC_params/'
        self.step_size = rospy.get_param(base_path+'control_params/step_size') 
        self.at_waypoint_dist_threshold = rospy.get_param(base_path+'control_params/at_waypoint_dist_threshold') 
        self.at_waypoint_goal_dist_threshold = rospy.get_param(base_path+'control_params/at_waypoint_goal_dist_threshold') 
        self.frequency = rospy.get_param(base_path+'control_params/frequency') 
        self.cost1_weight = rospy.get_param(base_path+'control_params/cost1_weight')
        self.cost2_weight = rospy.get_param(base_path+'control_params/cost2_weight')
        self.cost3_weight = rospy.get_param(base_path+'control_params/cost3_weight')
        self.cost4_weight = rospy.get_param(base_path+'control_params/cost4_weight')

        self.g = rospy.get_param(base_path+'model_params/g')
        self.mu = rospy.get_param(base_path+'model_params/mu')
        self.y1_initial = rospy.get_param(base_path+'model_params/y1_initial')
        self.y2_initial = rospy.get_param(base_path+'model_params/y2_initial')
        self.y_min = rospy.get_param(base_path+'model_params/y_min')
        self.y_max = rospy.get_param(base_path+'model_params/y_max')
        self.z_min = rospy.get_param(base_path+'model_params/z_min')
        self.z_max = rospy.get_param(base_path+'model_params/z_max')
        self.K_1 = rospy.get_param(base_path+'model_params/K_1')
        self.K_2 = rospy.get_param(base_path+'model_params/K_2')
        self.F_break = rospy.get_param(base_path+'model_params/F_break')
        self.F_comfort = rospy.get_param(base_path+'model_params/F_comfort')
        self.q_t_min = rospy.get_param(base_path+'model_params/q_t_min')
        self.q_t_max = rospy.get_param(base_path+'model_params/q_t_max')
        self.q_s_min_rel_q_t = rospy.get_param(base_path+'model_params/q_s_min_rel_q_t')
        self.q_s_max_rel_q_t = rospy.get_param(base_path+'model_params/q_s_max_rel_q_t')
        self.r_t = rospy.get_param(base_path+'model_params/r_t')
        self.r_s = rospy.get_param(base_path+'model_params/r_s')
        self.m_t = rospy.get_param(base_path+'model_params/m_t')
        self.m_s = rospy.get_param(base_path+'model_params/m_s')
        self.L_t = rospy.get_param(base_path+'model_params/L_t')
        self.L_s = rospy.get_param(base_path+'model_params/L_s')
        self.m_t_actual = rospy.get_param(base_path+'model_params/m_t_actual')
        self.m_s_actual = rospy.get_param(base_path+'model_params/m_s_actual')
        self.L_t_actual = rospy.get_param(base_path+'model_params/L_t_actual')
        self.L_s_actual = rospy.get_param(base_path+'model_params/L_s_actual')

        self.actual_theta_t = 0.
        self.actual_theta_s = 0.

    def initialize_msg(self):
        # set up a template message to publish to move robot
        self.msg = JointTrajectory()
        self.msg.header.frame_id = "/base_link"
        jp = JointTrajectoryPoint()
        jp.positions = [0.,0.,0.,0.,0.,0.]
        jp.time_from_start = rospy.Duration(0.)
        self.msg.points.append(jp)
        self.msg.joint_names = rospy.get_param('/simple_prismatic_robot_controller_2/joints')

    def create_msg(self,goal,time):
        # populate template message 
        self.msg.points[0].positions = goal
        self.msg.points[0].time_from_start = rospy.Duration(time)
        self.msg.header.stamp = rospy.Time.now()

    def send_goal(self,goal,time,freq,previous_goal=None):
        # call create message to populate message and publish 
        jerror_threshold = 50 # arbitrarily chosen for now (it's too large to have an effect...50 meters/radians)
        r = rospy.Rate(freq)
        goal_trajectories = self.linear_interpolation_goal(goal,time,freq,previous_goal)
        for i in range(len(goal_trajectories)):
            self.create_msg(goal_trajectories[i],1./freq)
            self.move_pub.publish(self.msg)
            r.sleep()
            # check to make sure the joint error is not too large
            jerror_cond = np.linalg.norm(np.array(self.joint_error)) > jerror_threshold
            while jerror_cond:
                self.move.publish(self.msg)
                r.sleep()
                print 'wait for joint error small than threshold...', 'err=',  np.linalg.norm(np.array(self.joint_error(goal_trajectories[i])))
                jerror_cond = np.linalg.norm(np.array(self.joint_error(goal_trajectories[i]))) > jerror_threshold
        return True

    def send_goal_new(self,goal,time,freq):
        r = rospy.Rate(freq)
        for i in range(200):         # 10 is arbitrarily selected.  just need to send enough messages to make sure the low level controller got it
            self.create_msg(goal,time)
            self.move_pub.publish(self.msg)
            r.sleep()

    def linear_interpolation_goal(self,goal,time,freq,previous_goal=None):
        # create a linear interpolation for the low level controller to follow
        # could just send the goal position, but this is not as smooth/you would have to decrease the step size 
        if previous_goal == None:
            init = self.robot_desired_ja
        else:
            init = previous_goal
        goal_trajectory = []
        increment = (np.array(goal)-np.array(init))/(time*freq)
        for i in range(int(time*freq)):
            next_goal = init+(i+1)*increment
            goal_trajectory.append(next_goal.tolist())
        return goal_trajectory

    def lift(self,goal):
        self.initialize_msg()
        self.goal = goal
        # we are sending desired joint angles, not actual joint angles because this is an impedance controller and we don't know if the joint will actually reach the goal
        # determine the required velocity based on the maximum required velocity out of all the joints
        #required_velocity = max(abs(np.array(self.goal)-np.array(self.robot_desired_ja))/self.time_from_start)
        # if the required velocity is greater than the maximum velocity then we will increase the time from start so that the joint will still only move at the max velocity
        #if required_velocity < self.max_velocity:
        #    self.send_goal(self.goal,self.time_from_start,1000)
        #else:
        #    print "new time from start: ",(required_velocity*self.time_from_start)/self.max_velocity
        #    self.send_goal(self.goal,(required_velocity*self.time_from_start)/self.max_velocity,1000)
        self.send_goal_new(self.goal, self.time_from_start, 1000)
        print "end time: ",rospy.get_time()


if __name__ == '__main__':
    #rospy.init_node('spr_lift_node')
    spr_lift = SPR2YZTSComNode() 
    spr_lift.lift([0.,0.5,0.5,0.,0.5,0.5])
    rospy.spin()
