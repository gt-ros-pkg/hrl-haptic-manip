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



########################## Main Script ##########################
# This is the main script for the MPC.  It makes calls to all other modules and performs the main control loop.

# File arguments:
# 1. Goal joint angle for the thigh (float)
# 2. Goal joint angle for the shank (float)
# 3. Friction constraint (boolean)
# 4. Data file name (string)

# Functions: 
# update_robot_MPC_state
#       - inputs: MPC communication object (low level control interface), robot MPC state object
#       - outputs: updated robot MPC state object
#################################################################


import waypoint_module
import spr2yzts_communication_module
import model_module
import optimization_module
import data_logging_module

import copy
import rospy
import numpy as np
import sys
import time
import math

# make sure that a goal position is provided
assert len(sys.argv) >= 4

# initialize data file
if len(sys.argv) < 5:
    csv_data_file = data_logging_module.initialize_data_file("no_name_data_file")
else:
    csv_data_file = data_logging_module.initialize_data_file(sys.argv[4])

# instantiate an object with class from spr2yzts_communication_module
# change depending on what robot communication model you are using
robot_MPC_communication = spr2yzts_communication_module.SPR2YZTSComNode()
rospy.sleep(5.)

class robot_MPC_state(object):
    def __init__(self):
        # initialize variables that will stay the same during the simulation (constant parameters)
        self.step_size = copy.copy(robot_MPC_communication.step_size)
        self.at_waypoint_dist_threshold = copy.copy(robot_MPC_communication.at_waypoint_dist_threshold)
        self.at_waypoint_goal_dist_threshold = copy.copy(robot_MPC_communication.at_waypoint_goal_dist_threshold)
        self.frequency = copy.copy(robot_MPC_communication.frequency)
        self.cost1_weight = copy.copy(robot_MPC_communication.cost1_weight)
        self.cost2_weight = copy.copy(robot_MPC_communication.cost2_weight)
        self.cost3_weight = copy.copy(robot_MPC_communication.cost3_weight)
        self.cost4_weight = copy.copy(robot_MPC_communication.cost4_weight)
        self.g = copy.copy(robot_MPC_communication.g)
        self.mu = copy.copy(robot_MPC_communication.mu)
        self.y1_initial = copy.copy(robot_MPC_communication.y1_initial)
        self.y2_initial = copy.copy(robot_MPC_communication.y2_initial)
        self.y_min = copy.copy(robot_MPC_communication.y_min)
        self.y_max = copy.copy(robot_MPC_communication.y_max)
        self.z_min = copy.copy(robot_MPC_communication.z_min)
        self.z_max = copy.copy(robot_MPC_communication.z_max)
        self.K_1 = copy.copy(robot_MPC_communication.K_1)
        self.K_2 = copy.copy(robot_MPC_communication.K_2)
        self.F_break = copy.copy(robot_MPC_communication.F_break)
        self.F_comfort = copy.copy(robot_MPC_communication.F_comfort)
        self.q_t_min = copy.copy(robot_MPC_communication.q_t_min)
        self.q_t_max = copy.copy(robot_MPC_communication.q_t_max)
        self.q_s_min_rel_q_t = copy.copy(robot_MPC_communication.q_s_min_rel_q_t)
        self.q_s_max_rel_q_t = copy.copy(robot_MPC_communication.q_s_max_rel_q_t)
        self.r_t = copy.copy(robot_MPC_communication.r_t)
        self.r_s = copy.copy(robot_MPC_communication.r_s)
        self.m_t = copy.copy(robot_MPC_communication.m_t)
        self.m_s = copy.copy(robot_MPC_communication.m_s)
        self.L_t = copy.copy(robot_MPC_communication.L_t)
        self.L_s = copy.copy(robot_MPC_communication.L_s)
        self.m_t_actual = copy.copy(robot_MPC_communication.m_t_actual)
        self.m_s_actual = copy.copy(robot_MPC_communication.m_s_actual)
        self.L_t_actual = copy.copy(robot_MPC_communication.L_t_actual)
        self.L_s_actual = copy.copy(robot_MPC_communication.L_s_actual)
        self.z1_initial = copy.copy(robot_MPC_communication.z1_initial)
        self.z2_initial = copy.copy(robot_MPC_communication.z2_initial)

        # these variables change during the simulation
        self.i = 1
        self.deadzone_ind = 0
        self.current_waypoint_ind = 0
        self.deadzone = False
        self.goal_positions = [0.2, -0.1]
        self.current_waypoint = [0., 0.]
        self.previous_waypoint = [0., 0.]

# instantiate object with above class
robot_MPC_state = robot_MPC_state()

def update_robot_MPC_state(robot_MPC_communication, robot_MPC_state):
    # update robot parameters (non-constant parameters)
    robot_MPC_state.F_a1_1 = copy.copy(robot_MPC_communication.F_a1_1)
    robot_MPC_state.F_a2_1 = copy.copy(robot_MPC_communication.F_a2_1)
    robot_MPC_state.F_b1_2 = copy.copy(robot_MPC_communication.F_b1_2)
    robot_MPC_state.F_b2_2 = copy.copy(robot_MPC_communication.F_b2_2)
    robot_MPC_state.F_y_1 = copy.copy(robot_MPC_communication.F_y_1)
    robot_MPC_state.F_y_2 = copy.copy(robot_MPC_communication.F_y_2)
    robot_MPC_state.F_z_1 = copy.copy(robot_MPC_communication.F_z_1)
    robot_MPC_state.F_z_2 = copy.copy(robot_MPC_communication.F_z_2)
    robot_MPC_state.actual_theta_t = copy.copy(robot_MPC_communication.actual_theta_t)
    robot_MPC_state.actual_theta_s = copy.copy(robot_MPC_communication.actual_theta_s)
    robot_MPC_state.d1 = copy.copy(robot_MPC_communication.d1)
    robot_MPC_state.d2 = copy.copy(robot_MPC_communication.d2)
    robot_MPC_state.q1 = copy.copy(robot_MPC_communication.q1)
    robot_MPC_state.q2 = copy.copy(robot_MPC_communication.q2)
    robot_MPC_state.y1 = copy.copy(robot_MPC_communication.y1)
    robot_MPC_state.y2 = copy.copy(robot_MPC_communication.y2)
    robot_MPC_state.z1 = copy.copy(robot_MPC_communication.z1)
    robot_MPC_state.z2 = copy.copy(robot_MPC_communication.z2)
    robot_MPC_state.q_eq1 = copy.copy(robot_MPC_communication.q_eq1)
    robot_MPC_state.q_eq2 = copy.copy(robot_MPC_communication.q_eq2)
    robot_MPC_state.y_eq1 = copy.copy(robot_MPC_communication.y_eq1)
    robot_MPC_state.y_eq2 = copy.copy(robot_MPC_communication.y_eq2)
    robot_MPC_state.z_eq1 = copy.copy(robot_MPC_communication.z_eq1)
    robot_MPC_state.z_eq2 = copy.copy(robot_MPC_communication.z_eq2)
    robot_MPC_state.knee_y = copy.copy(robot_MPC_communication.knee_y)
    robot_MPC_state.knee_z = copy.copy(robot_MPC_communication.knee_z)

    return robot_MPC_state
    

# Start time
simulation_start_time = rospy.Time.now().to_sec()
wall_start_time = time.time()
simulation_current_time = 0
simulation_prev_time = 0
wall_current_time = 0
wall_prev_time = 0

# initialization sequence
goal_positions_thigh=[0.2,0.2]
goal_positions_shank=[0.,-0.1]
for j in range(len(goal_positions_thigh)):
    # initialize parameters
    robot_MPC_state.i = 1
    robot_MPC_state.deadzone_ind = 0
    robot_MPC_state.current_waypoint_ind = 0
    r = rospy.Rate(robot_MPC_state.frequency)
    robot_MPC_state.deadzone = False

    # update robot state
    robot_MPC_state = update_robot_MPC_state(robot_MPC_communication, robot_MPC_state)

    robot_MPC_state.goal_positions = [goal_positions_thigh[j],goal_positions_shank[j]]

    # generate waypoints
    initialization_waypoints = waypoint_module.generate_linear_trajectory([robot_MPC_state.actual_theta_t, robot_MPC_state.actual_theta_s], robot_MPC_state.goal_positions, robot_MPC_state.step_size)

    # loop for using MPC to reach goal position
    # loops until the robot is in the deadzone for at least 5 seconds
    while robot_MPC_state.deadzone_ind != int(round(robot_MPC_state.frequency*5)):

        # update robot state
        robot_MPC_state = update_robot_MPC_state(robot_MPC_communication, robot_MPC_state)
  
        # if the robot does not reach the goal position in 30/60 seconds, then exit
        #if robot_MPC_state.i == robot_MPC_state.frequency*30:
        if robot_MPC_state.i == int(round(robot_MPC_state.frequency*60)):
            # exit program if final goal in sequence is not reached
            if j == 1:
                print "[ERROR]: COULD NOT INITIALIZE"
                sys.exit()
            # continue on to next goal
            else:
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                break
        
        # give the MPC a local goal based on the waypoints and perform the optimization
        robot_MPC_state.previous_waypoint[0] = robot_MPC_state.current_waypoint[0]
        robot_MPC_state.previous_waypoint[1] = robot_MPC_state.current_waypoint[1]
        robot_MPC_state.current_waypoint[0] = initialization_waypoints[0,robot_MPC_state.current_waypoint_ind]
        robot_MPC_state.current_waypoint[1] = initialization_waypoints[1,robot_MPC_state.current_waypoint_ind]
        delta_theta_waypoint = initialization_waypoints[:,robot_MPC_state.current_waypoint_ind]-np.matrix([robot_MPC_state.actual_theta_t, robot_MPC_state.actual_theta_s]).T
        H, f, A, b, lb, ub, dq_dxeq, df_dxeq, cost1_quad_matrix, cost1_linear_matrix, cost2_quad_matrix, cost2_linear_matrix, \
        cost3_quad_matrix, cost3_linear_matrix, cost4_quad_matrix, cost4_linear_matrix = model_module.model_1(robot_MPC_state, delta_theta_waypoint, initialize=True, friction_constraint=True)
        delta_phi = optimization_module.cvxopt_optimization(H, f, A, b, lb, ub)

        # for debugging/viewing purposes
        lower_friction_bound1 = -robot_MPC_state.mu*(robot_MPC_state.F_a2_1)-robot_MPC_state.mu*(-df_dxeq[0,:]*math.sin(robot_MPC_state.actual_theta_t)+df_dxeq[1,:]*math.cos(robot_MPC_state.actual_theta_t))*delta_phi
        tangential_force1 = robot_MPC_state.F_a1_1+(df_dxeq[0,:]*math.cos(robot_MPC_state.actual_theta_t)+df_dxeq[1,:]*math.sin(robot_MPC_state.actual_theta_t))*delta_phi
        upper_friction_bound1 = robot_MPC_state.mu*(robot_MPC_state.F_a2_1)+robot_MPC_state.mu*(-df_dxeq[0,:]*math.sin(robot_MPC_state.actual_theta_t)+df_dxeq[1,:]*math.cos(robot_MPC_state.actual_theta_t))*delta_phi
        lower_friction_bound2 = -robot_MPC_state.mu*(robot_MPC_state.F_b2_2)-robot_MPC_state.mu*(-df_dxeq[2,:]*math.sin(robot_MPC_state.actual_theta_s)+df_dxeq[3,:]*math.cos(robot_MPC_state.actual_theta_s))*delta_phi
        tangential_force2 = robot_MPC_state.F_b1_2+(df_dxeq[2,:]*math.cos(robot_MPC_state.actual_theta_s)+df_dxeq[3,:]*math.sin(robot_MPC_state.actual_theta_s))*delta_phi 
        upper_friction_bound2 = robot_MPC_state.mu*(robot_MPC_state.F_b2_2)+robot_MPC_state.mu*(-df_dxeq[2,:]*math.sin(robot_MPC_state.actual_theta_s)+df_dxeq[3,:]*math.cos(robot_MPC_state.actual_theta_s))*delta_phi 

        print "******************************************"
        print "***********Friction Constraint************"
        print lower_friction_bound1, " <= ", tangential_force1, " <= ", upper_friction_bound1
        print lower_friction_bound2, " <= ", tangential_force2, " <= ", upper_friction_bound2
        print "******************************************"

        # predict the new theta and forces based on the optimization (useful for debugging, checking predictive accuracy) (used in the robot angle control)
        predicted_theta_t = robot_MPC_state.actual_theta_t+dq_dxeq[0,:]*delta_phi
        predicted_theta_s = robot_MPC_state.actual_theta_s+dq_dxeq[1,:]*delta_phi
        predicted_F_y_1 = robot_MPC_state.F_y_1+df_dxeq[0,:]*delta_phi
        predicted_F_z_1 = robot_MPC_state.F_z_1+df_dxeq[1,:]*delta_phi
        predicted_F_y_2 = robot_MPC_state.F_y_2+df_dxeq[2,:]*delta_phi
        predicted_F_z_2 = robot_MPC_state.F_z_2+df_dxeq[3,:]*delta_phi
        
        # send new robot equilibrium positions to low level controller
        # the y position command that is sent to the low level controller has to be relative to the initial starting point (same as robot_desired_ja)...that is why we have y_initial subtracting the value
        #robot_MPC_communication.lift([-predicted_theta_t, robot_MPC_state.y_initial1-(robot_MPC_state.y_eq1+delta_phi[0,0]), robot_MPC_state.z_eq1+delta_phi[1,0], -predicted_theta_s, robot_MPC_state.y_initial2-(robot_MPC_state.y_eq2+delta_phi[2,0]), robot_MPC_state.z_eq2+delta_phi[3,0]])
        robot_MPC_communication.lift([-predicted_theta_t, robot_MPC_state.y1_initial-(robot_MPC_state.y_eq1+delta_phi[0,0]), robot_MPC_state.z_eq1+delta_phi[1,0]-robot_MPC_state.z1_initial, -predicted_theta_s, robot_MPC_state.y2_initial-(robot_MPC_state.y_eq2+delta_phi[2,0]), robot_MPC_state.z_eq2+delta_phi[3,0]-robot_MPC_state.z2_initial])

        # calculate real time factor
        simulation_current_time = rospy.Time.now().to_sec()-simulation_start_time
        print "Simulation Time: ",rospy.Time.now().to_sec()-simulation_start_time
        print "Delta Simulation Time: ",simulation_current_time-simulation_prev_time
        wall_current_time = time.time()-wall_start_time
        print "Wall Time: ",time.time()-wall_start_time
        print "Delta Wall Time: ",wall_current_time-wall_prev_time
        if (wall_current_time-wall_prev_time) == 0.:
            real_time_factor = 0.
        else:
            real_time_factor = (simulation_current_time-simulation_prev_time)/(wall_current_time-wall_prev_time)
        print "real time factor: ",real_time_factor
        simulation_prev_time = simulation_current_time
        wall_prev_time = wall_current_time
        

        # sleep until time for the next iteration
        r.sleep()
  
        # check if we are at the next waypoint (this is at the end b/c it gives the robots time to settle) 
        robot_MPC_state.current_waypoint_ind, robot_MPC_state.deadzone, robot_MPC_state.deadzone_ind = waypoint_module.check_waypoint([robot_MPC_state.actual_theta_t, robot_MPC_state.actual_theta_s], robot_MPC_state.goal_positions, robot_MPC_state.current_waypoint_ind, initialization_waypoints, robot_MPC_state.deadzone, robot_MPC_state.deadzone_ind, robot_MPC_state.at_waypoint_goal_dist_threshold, robot_MPC_state.at_waypoint_dist_threshold)
        data_logging_module.record_data(csv_data_file, robot_MPC_state, delta_phi, dq_dxeq, df_dxeq, real_time_factor, 
                                        cost1_quad_matrix, cost1_linear_matrix, cost2_quad_matrix, cost2_linear_matrix,  
                                        cost3_quad_matrix, cost3_linear_matrix, cost4_quad_matrix, cost4_linear_matrix)

        if robot_MPC_state.deadzone == True:
            robot_MPC_state.i = 0
        else:
            robot_MPC_state.i += 1

        print "current_waypoint_ind: ",robot_MPC_state.current_waypoint_ind
        print "delta_theta_waypoint: ",delta_theta_waypoint
        print "q_t: ",robot_MPC_state.actual_theta_t
        print "q_s: ",robot_MPC_state.actual_theta_s
        print "delta_phi: ",delta_phi
        print "deadzone_ind: ", robot_MPC_state.deadzone_ind
        print "i: ",robot_MPC_state.i
        print "i_limit: ", robot_MPC_state.frequency*30
    
# lifting sequence
# take in lifting goals from user
goal_positions_thigh=[float(sys.argv[1]),0.2]
goal_positions_shank=[float(sys.argv[2]),-0.1]
for j in range(len(goal_positions_thigh)):
    # initialize parameters
    robot_MPC_state.i = 1
    robot_MPC_state.deadzone_ind = 0
    robot_MPC_state.current_waypoint_ind = 0
    r = rospy.Rate(robot_MPC_state.frequency)
    robot_MPC_state.deadzone = False

    # update robot state
    robot_MPC_state = update_robot_MPC_state(robot_MPC_communication, robot_MPC_state)

    robot_MPC_state.goal_positions = [goal_positions_thigh[j],goal_positions_shank[j]]
    # generate waypoints
    lifting_waypoints = waypoint_module.generate_linear_trajectory([robot_MPC_state.actual_theta_t, robot_MPC_state.actual_theta_s], robot_MPC_state.goal_positions, robot_MPC_state.step_size)

    # loop for using MPC to reach goal position
    # loops until the robot is in the deadzone for at least 5 seconds
    while robot_MPC_state.deadzone_ind != int(round(robot_MPC_state.frequency*5)):
 
        # update robot state
        robot_MPC_state = update_robot_MPC_state(robot_MPC_communication, robot_MPC_state)

        # if the robot does not reach the goal position in 30/60 seconds, then exit
        #if robot_MPC_state.i == robot_MPC_state.frequency*30:
        if robot_MPC_state.i == int(round(robot_MPC_state.frequency*60)):
            if j == 1:
                print "[ERROR]: COULD NOT INITIALIZE"
                sys.exit()
            else:
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                print "CONTINUING"
                break

        # give the MPC a local goal based on the waypoints and perform the optimization
        robot_MPC_state.previous_waypoint[0] = robot_MPC_state.current_waypoint[0]
        robot_MPC_state.previous_waypoint[1] = robot_MPC_state.current_waypoint[1]
        robot_MPC_state.current_waypoint[0] = lifting_waypoints[0,robot_MPC_state.current_waypoint_ind]
        robot_MPC_state.current_waypoint[1] = lifting_waypoints[1,robot_MPC_state.current_waypoint_ind]
        delta_theta_waypoint = lifting_waypoints[:,robot_MPC_state.current_waypoint_ind]-np.matrix([robot_MPC_state.actual_theta_t, robot_MPC_state.actual_theta_s]).T
        H, f, A, b, lb, ub, dq_dxeq, df_dxeq, cost1_quad_matrix, cost1_linear_matrix, cost2_quad_matrix, cost2_linear_matrix, \
        cost3_quad_matrix, cost3_linear_matrix, cost4_quad_matrix, cost4_linear_matrix = model_module.model_1(robot_MPC_state, delta_theta_waypoint, initialize=False, friction_constraint=bool(sys.argv[3]))
        delta_phi = optimization_module.cvxopt_optimization(H, f, A, b, lb, ub)

        # for debugging/viewing purposes
        lower_friction_bound1 = -robot_MPC_state.mu*(robot_MPC_state.F_a2_1)-robot_MPC_state.mu*(-df_dxeq[0,:]*math.sin(robot_MPC_state.actual_theta_t)+df_dxeq[1,:]*math.cos(robot_MPC_state.actual_theta_t))*delta_phi
        tangential_force1 = robot_MPC_state.F_a1_1+(df_dxeq[0,:]*math.cos(robot_MPC_state.actual_theta_t)+df_dxeq[1,:]*math.sin(robot_MPC_state.actual_theta_t))*delta_phi
        upper_friction_bound1 = robot_MPC_state.mu*(robot_MPC_state.F_a2_1)+robot_MPC_state.mu*(-df_dxeq[0,:]*math.sin(robot_MPC_state.actual_theta_t)+df_dxeq[1,:]*math.cos(robot_MPC_state.actual_theta_t))*delta_phi
        lower_friction_bound2 = -robot_MPC_state.mu*(robot_MPC_state.F_b2_2)-robot_MPC_state.mu*(-df_dxeq[2,:]*math.sin(robot_MPC_state.actual_theta_s)+df_dxeq[3,:]*math.cos(robot_MPC_state.actual_theta_s))*delta_phi
        tangential_force2 = robot_MPC_state.F_b1_2+(df_dxeq[2,:]*math.cos(robot_MPC_state.actual_theta_s)+df_dxeq[3,:]*math.sin(robot_MPC_state.actual_theta_s))*delta_phi
        upper_friction_bound2 = robot_MPC_state.mu*(robot_MPC_state.F_b2_2)+robot_MPC_state.mu*(-df_dxeq[2,:]*math.sin(robot_MPC_state.actual_theta_s)+df_dxeq[3,:]*math.cos(robot_MPC_state.actual_theta_s))*delta_phi

        print "******************************************"
        print "***********Friction Constraint************"
        print lower_friction_bound1, " <= ", tangential_force1, " <= ", upper_friction_bound1
        print lower_friction_bound2, " <= ", tangential_force2, " <= ", upper_friction_bound2
        print "******************************************"


        # predict the new theta and forces based on the optimization (useful for debugging, checking predictive accuracy) (used in the robot angle control)
        predicted_theta_t = robot_MPC_state.actual_theta_t+dq_dxeq[0,:]*delta_phi
        predicted_theta_s = robot_MPC_state.actual_theta_s+dq_dxeq[1,:]*delta_phi
        print "predicted theta t: ",predicted_theta_t
        print "predicted theta s: ",predicted_theta_s
        predicted_F_y_1 = robot_MPC_state.F_y_1+df_dxeq[0,:]*delta_phi
        predicted_F_z_1 = robot_MPC_state.F_z_1+df_dxeq[1,:]*delta_phi
        predicted_F_y_2 = robot_MPC_state.F_y_2+df_dxeq[2,:]*delta_phi
        predicted_F_z_2 = robot_MPC_state.F_z_2+df_dxeq[3,:]*delta_phi

        # send new robot equilibrium positions to low level controller
        robot_MPC_communication.lift([-predicted_theta_t, robot_MPC_state.y1_initial-(robot_MPC_state.y_eq1+delta_phi[0,0]), robot_MPC_state.z_eq1+delta_phi[1,0]-robot_MPC_state.z1_initial, -predicted_theta_s, robot_MPC_state.y2_initial-(robot_MPC_state.y_eq2+delta_phi[2,0]), robot_MPC_state.z_eq2+delta_phi[3,0]-robot_MPC_state.z2_initial])

        # calculate real time factor
        simulation_current_time = rospy.Time.now().to_sec()-simulation_start_time
        print "Simulation Time: ",rospy.Time.now().to_sec()-simulation_start_time
        print "Delta Simulation Time: ",simulation_current_time-simulation_prev_time
        wall_current_time = time.time()-wall_start_time
        print "Wall Time: ",time.time()-wall_start_time
        print "Delta Wall Time: ",wall_current_time-wall_prev_time
        if (wall_current_time-wall_prev_time) == 0.:
            real_time_factor = 0.
        else:
            real_time_factor = (simulation_current_time-simulation_prev_time)/(wall_current_time-wall_prev_time)
        print "real time factor: ",real_time_factor
        simulation_prev_time = simulation_current_time
        wall_prev_time = wall_current_time

        # sleep until time for the next iteration
        r.sleep()
    
        # check if we are at the next waypoint (this is at the end b/c it gives the robots time to settle)
        robot_MPC_state.current_waypoint_ind, robot_MPC_state.deadzone, robot_MPC_state.deadzone_ind = waypoint_module.check_waypoint([robot_MPC_state.actual_theta_t, robot_MPC_state.actual_theta_s], robot_MPC_state.goal_positions, robot_MPC_state.current_waypoint_ind, lifting_waypoints, robot_MPC_state.deadzone, robot_MPC_state.deadzone_ind, robot_MPC_state.at_waypoint_goal_dist_threshold, robot_MPC_state.at_waypoint_dist_threshold)
        data_logging_module.record_data(csv_data_file, robot_MPC_state, delta_phi, dq_dxeq, df_dxeq, real_time_factor,
                                        cost1_quad_matrix, cost1_linear_matrix, cost2_quad_matrix, cost2_linear_matrix, 
                                        cost3_quad_matrix, cost3_linear_matrix, cost4_quad_matrix, cost4_linear_matrix)

        if robot_MPC_state.deadzone == True:
            robot_MPC_state.i = 0
        else:
            robot_MPC_state.i += 1

        print "current_waypoint_ind: ",robot_MPC_state.current_waypoint_ind
        print "delta_theta_waypoint: ",delta_theta_waypoint
        print "q_t: ",robot_MPC_state.actual_theta_t
        print "q_s: ",robot_MPC_state.actual_theta_s
        print "delta_phi: ",delta_phi
        print "deadzone_ind: ", robot_MPC_state.deadzone_ind
        print "i: ",robot_MPC_state.i
        print "i_limit: ", robot_MPC_state.frequency*30



