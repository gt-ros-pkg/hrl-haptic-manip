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



########################## Model Module ##########################
# This script contains the robot model for the MPC.  It outputs the matrices used in the optimization module.

# Functions: 
# general_quad_linear_cost
#  	conversion of a quadratic cost function into the quadratic and linear components
#       - inputs: J (matrix), goal (vector)
#       - outputs: csv file (csv writer object)
# model_1
#       - inputs: robot MPC state object, delta theta waypoint (vector), initialize (bool), friction constraint (bool)
#       - outputs: H (nxn matrix, symmetric, positive-definite), f (vector of length n), A (mxn matrix subjected to A * x <= b), b (vector of length m), lb (vector of length n), ub (vector of length n), dq_dxeq (matrix, debugging/viewing purposes), df_dxeq (matrix, debugging/viewing purposes), cost matrices 
#################################################################



import numpy as np
from math import *

def general_quad_linear_cost(J, goal):
    # In the form (goal+J*delta_x)^2
    cost_quad_matrix = J.T*J
    if type(goal) == float:
        cost_linear_matrix = 2.*goal*J
    else:
        cost_linear_matrix = 2.*goal.T*J
    return cost_quad_matrix, cost_linear_matrix


# Model 1 refers to using two SPRYZ robots to lift a human leg that has a hip and knee joint in 2 dimensions.  This was used in the Humanoids 2015 paper "Robotic Repositioning of Human Limbs via Model Predictive Control".
def model_1(robot_MPC_state, delta_theta_waypoint, initialize, friction_constraint):
    
    # variable name mapping
    step_size = robot_MPC_state.step_size
    cost1_weight = robot_MPC_state.cost1_weight
    cost2_weight = robot_MPC_state.cost2_weight
    cost3_weight = robot_MPC_state.cost3_weight
    cost4_weight = robot_MPC_state.cost4_weight
    g_a = robot_MPC_state.g
    mu = robot_MPC_state.mu
    y_min = robot_MPC_state.y_min
    y_max = robot_MPC_state.y_max
    z_min = robot_MPC_state.z_min
    z_max = robot_MPC_state.z_max
    y1 = robot_MPC_state.y1
    y2 = robot_MPC_state.y2
    z1 = robot_MPC_state.z1
    z2 = robot_MPC_state.z2
    k_t = robot_MPC_state.K_1
    k_s = robot_MPC_state.K_2
    F_break = robot_MPC_state.F_break
    F_comfort = robot_MPC_state.F_comfort
    F_a1_1 = robot_MPC_state.F_a1_1
    F_a2_1 = robot_MPC_state.F_a2_1
    F_b1_2 = robot_MPC_state.F_b1_2
    F_b2_2 = robot_MPC_state.F_b2_2
    F_y_1 = robot_MPC_state.F_y_1
    F_z_1 = robot_MPC_state.F_z_1
    F_y_2 = robot_MPC_state.F_y_2
    F_z_2 = robot_MPC_state.F_z_2
    q_t_min = robot_MPC_state.q_t_min
    q_t_max = robot_MPC_state.q_t_max
    q_s_min_rel_q_t = robot_MPC_state.q_s_min_rel_q_t
    q_s_max_rel_q_t = robot_MPC_state.q_s_max_rel_q_t
    r_t = robot_MPC_state.r_t
    r_s = robot_MPC_state.r_s
    q_t = robot_MPC_state.actual_theta_t
    q_s = robot_MPC_state.actual_theta_s
    d_t = robot_MPC_state.d1
    d_s = robot_MPC_state.d2
    if initialize == True:
        m_t = robot_MPC_state.m_t_actual
        m_s = robot_MPC_state.m_s_actual
        L_t = robot_MPC_state.L_t_actual
        L_s = robot_MPC_state.L_s_actual
##### temp
        cost1_weight = 1000000000
        cost2_weight = 1000
        cost3_weight = 2
        cost4_weight = 2
####
    else:
        m_t = robot_MPC_state.m_t
        m_s = robot_MPC_state.m_s
        L_t = robot_MPC_state.L_t
        L_s = robot_MPC_state.L_s


    print "############################################"
    print "m_t = ", m_t
    print "m_s = ", m_s
    print "L_t = ", L_t
    print "L_t = ", L_s
    print "############################################"
    

    # Charlie's derivation for dq_dxeq matrix
    dq_dxeq = np.matrix([[(4*(k_t)*((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)*(cos(q_t)*(r_t) - (d_t)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), (4*(k_t)*((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)*(cos(q_t)*(d_t) + (r_t)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), ((k_s)*(-2*(d_s)*(2*cos(2*(q_s) - (q_t))*(k_s)*(L_t)*(r_s) + (g_a)*(L_s)*(m_s)*sin(q_s)**2) + 4*cos(q_s)*(d_s)**2*(k_s)*(L_t)*sin((q_s) - (q_t)) + 2*sin(q_s)*(-2*cos((q_s) - (q_t))*(k_s)*(L_t)*(r_s)**2 + (g_a)*(L_s)*(m_s)*(cos(q_s)*(r_s) + (L_t)*sin(q_t)))))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t))), ((k_s)*(2*(g_a)*(L_s)*(m_s)*sin(q_s)*(cos(q_s)*(d_s) - cos(q_t)*(L_t) + (r_s)*sin(q_s)) + 4*(k_s)*(L_t)*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*(cos((q_s) - (q_t))*(r_s) - (d_s)*sin((q_s) - (q_t)))))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t)))],

[(-4*(k_s)*(k_t)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(cos(q_t)*(r_t) - (d_t)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), (-4*(k_s)*(k_t)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(cos(q_t)*(d_t) + (r_t)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), (2*(k_s)*((cos(q_s)*(r_s) - (d_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*cos(q_t)**2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t))) + (k_s)*(L_t)**2*(cos(q_s)*(d_s) + (r_s)*sin(q_s))*sin(2*(q_t))))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), (2*(k_s)*((cos(q_s)*(d_s) + (r_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*sin(q_t)*(-((g_a)*(2*(m_s) + (m_t))) + 2*(k_s)*(L_t)*sin(q_t))) + (k_s)*(L_t)**2*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*sin(2*(q_t))))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t))))]])


    # Charlie's derivation for df_dxeq matrix
    df_dxeq = np.matrix([[(k_t)*(1 - (4*(k_t)*((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)*(cos(q_t)*(r_t) - (d_t)*sin(q_t))**2)/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t))))), (-4*(k_t)**2*((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)*(cos(q_t)*(r_t) - (d_t)*sin(q_t))*(cos(q_t)*(d_t) + (r_t)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), -(((k_s)*(k_t)*(cos(q_t)*(r_t) - (d_t)*sin(q_t))*(-2*(d_s)*(2*cos(2*(q_s) - (q_t))*(k_s)*(L_t)*(r_s) + (g_a)*(L_s)*(m_s)*sin(q_s)**2) + 4*cos(q_s)*(d_s)**2*(k_s)*(L_t)*sin((q_s) - (q_t)) + 2*sin(q_s)*(-2*cos((q_s) - (q_t))*(k_s)*(L_t)*(r_s)**2 + (g_a)*(L_s)*(m_s)*(cos(q_s)*(r_s) + (L_t)*sin(q_t)))))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t)))), -(((k_s)*(k_t)*(2*(g_a)*(L_s)*(m_s)*sin(q_s)*(cos(q_s)*(d_s) - cos(q_t)*(L_t) + (r_s)*sin(q_s)) + 4*(k_s)*(L_t)*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*(cos((q_s) - (q_t))*(r_s) - (d_s)*sin((q_s) - (q_t))))*(cos(q_t)*(r_t) - (d_t)*sin(q_t)))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t))))],

[(-4*(k_t)**2*((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)*(cos(q_t)*(r_t) - (d_t)*sin(q_t))*(cos(q_t)*(d_t) + (r_t)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), (k_t)*(1 - (4*(k_t)*((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)*(cos(q_t)*(d_t) + (r_t)*sin(q_t))**2)/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t))))), -(((k_s)*(k_t)*(cos(q_t)*(d_t) + (r_t)*sin(q_t))*(-2*(d_s)*(2*cos(2*(q_s) - (q_t))*(k_s)*(L_t)*(r_s) + (g_a)*(L_s)*(m_s)*sin(q_s)**2) + 4*cos(q_s)*(d_s)**2*(k_s)*(L_t)*sin((q_s) - (q_t)) + 2*sin(q_s)*(-2*cos((q_s) - (q_t))*(k_s)*(L_t)*(r_s)**2 + (g_a)*(L_s)*(m_s)*(cos(q_s)*(r_s) + (L_t)*sin(q_t)))))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t)))), -(((k_s)*(k_t)*(2*(g_a)*(L_s)*(m_s)*sin(q_s)*(cos(q_s)*(d_s) - cos(q_t)*(L_t) + (r_s)*sin(q_s)) + 4*(k_s)*(L_t)*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*(cos((q_s) - (q_t))*(r_s) - (d_s)*sin((q_s) - (q_t))))*(cos(q_t)*(d_t) + (r_t)*sin(q_t)))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t))))],

[(4*(k_s)*(k_t)*(L_t)*(cos(q_t)*(r_t) - (d_t)*sin(q_t))*((k_s)*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t))) + ((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), (4*(k_s)*(k_t)*(L_t)*(cos(q_t)*(d_t) + (r_t)*sin(q_t))*((k_s)*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t))) + ((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), (k_s)*(1 + ((k_s)*(L_t)*sin(q_t)*(-2*(d_s)*(2*cos(2*(q_s) - (q_t))*(k_s)*(L_t)*(r_s) + (g_a)*(L_s)*(m_s)*sin(q_s)**2) + 4*cos(q_s)*(d_s)**2*(k_s)*(L_t)*sin((q_s) - (q_t)) + 2*sin(q_s)*(-2*cos((q_s) - (q_t))*(k_s)*(L_t)*(r_s)**2 + (g_a)*(L_s)*(m_s)*(cos(q_s)*(r_s) + (L_t)*sin(q_t)))))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t))) - (2*(k_s)*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*((cos(q_s)*(r_s) - (d_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*cos(q_t)**2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t))) + (k_s)*(L_t)**2*(cos(q_s)*(d_s) + (r_s)*sin(q_s))*sin(2*(q_t))))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t))))), (k_s)**2*(((L_t)*(2*(g_a)*(L_s)*(m_s)*sin(q_s)*(cos(q_s)*(d_s) - cos(q_t)*(L_t) + (r_s)*sin(q_s)) + 4*(k_s)*(L_t)*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*(cos((q_s) - (q_t))*(r_s) - (d_s)*sin((q_s) - (q_t))))*sin(q_t))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t))) - (2*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*((cos(q_s)*(d_s) + (r_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*sin(q_t)*(-((g_a)*(2*(m_s) + (m_t))) + 2*(k_s)*(L_t)*sin(q_t))) + (k_s)*(L_t)**2*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*sin(2*(q_t))))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))))],

[(4*(k_s)*(k_t)*(L_t)*(-(cos(q_t)*((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)) + (k_s)*(cos(q_s)*(d_s) + (r_s)*sin(q_s))*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t))))*(cos(q_t)*(r_t) - (d_t)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), (4*(k_s)*(k_t)*(L_t)*(-(cos(q_t)*((k_s)*((d_s)**2 + (r_s)**2) - ((g_a)*(L_s)*(m_s)*sin(q_s))/2)) + (k_s)*(cos(q_s)*(d_s) + (r_s)*sin(q_s))*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t))))*(cos(q_t)*(d_t) + (r_t)*sin(q_t)))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))), (k_s)**2*(-((cos(q_t)*(L_t)*(-2*(d_s)*(2*cos(2*(q_s) - (q_t))*(k_s)*(L_t)*(r_s) + (g_a)*(L_s)*(m_s)*sin(q_s)**2) + 4*cos(q_s)*(d_s)**2*(k_s)*(L_t)*sin((q_s) - (q_t)) + 2*sin(q_s)*(-2*cos((q_s) - (q_t))*(k_s)*(L_t)*(r_s)**2 + (g_a)*(L_s)*(m_s)*(cos(q_s)*(r_s) + (L_t)*sin(q_t)))))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t)))) - (2*(cos(q_s)*(d_s) + (r_s)*sin(q_s))*((cos(q_s)*(r_s) - (d_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*cos(q_t)**2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t))) + (k_s)*(L_t)**2*(cos(q_s)*(d_s) + (r_s)*sin(q_s))*sin(2*(q_t))))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t))))), (k_s)*(1 - (cos(q_t)*(k_s)*(L_t)*(2*(g_a)*(L_s)*(m_s)*sin(q_s)*(cos(q_s)*(d_s) - cos(q_t)*(L_t) + (r_s)*sin(q_s)) + 4*(k_s)*(L_t)*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*(cos((q_s) - (q_t))*(r_s) - (d_s)*sin((q_s) - (q_t)))))/(4*(k_s)*(r_s)**2*(cos((q_s) - (q_t))**2*(k_s)*(L_t)**2 + (k_t)*((d_t)**2 + (r_t)**2)) + 2*cos((q_s) - (q_t))*(d_s)*(k_s)*(L_t)*((g_a)*(L_s)*(m_s)*sin(q_s) - 4*(k_s)*(L_t)*(r_s)*sin((q_s) - (q_t))) + (g_a)**2*(L_s)*(L_t)*(m_s)*(2*(m_s) + (m_t))*sin(q_s)*sin(q_t) - 2*(d_s)**2*(k_s)*(-2*(k_t)*((d_t)**2 + (r_t)**2) - 2*(k_s)*(L_t)**2*sin((q_s) - (q_t))**2 + (g_a)*(L_t)*(2*(m_s) + (m_t))*sin(q_t)) - (g_a)*(2*(L_s)*(m_s)*sin(q_s)*((k_t)*((d_t)**2 + (r_t)**2) + (k_s)*(L_t)*((L_t) - (r_s)*sin((q_s) - (q_t)))) + 2*(k_s)*(L_t)*(2*(m_s) + (m_t))*(r_s)**2*sin(q_t))) - (2*(k_s)*(cos(q_s)*(d_s) + (r_s)*sin(q_s))*((cos(q_s)*(d_s) + (r_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*sin(q_t)*(-((g_a)*(2*(m_s) + (m_t))) + 2*(k_s)*(L_t)*sin(q_t))) + (k_s)*(L_t)**2*(cos(q_s)*(r_s) - (d_s)*sin(q_s))*sin(2*(q_t))))/(-2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))*(-((g_a)*(L_s)*(m_s)*sin(q_s)) + 2*(k_s)*(L_t)*(cos((q_s) - (q_t))*(d_s) + (r_s)*sin((q_s) - (q_t)))) + (2*(k_s)*((d_s)**2 + (r_s)**2) - (g_a)*(L_s)*(m_s)*sin(q_s))*(2*(k_t)*((d_t)**2 + (r_t)**2) + (L_t)*(2*(k_s)*(L_t) - (g_a)*(2*(m_s) + (m_t))*sin(q_t)))))]])

    print "################################"
    print "dq_dxeq: ",dq_dxeq
    print "df_dxeq: ",df_dxeq
    print "################################"

    # Cost Functions
    # 1. go to goal cost
    cost1_quad_matrix, cost1_linear_matrix = general_quad_linear_cost(-dq_dxeq,delta_theta_waypoint)
    # 2. cost on normal force difference between the two arms
    cost2_quad_matrix, cost2_linear_matrix = general_quad_linear_cost((-df_dxeq[0,:]*sin(q_t)+df_dxeq[1,:]*cos(q_t))-(-df_dxeq[2,:]*sin(q_s)+df_dxeq[3,:]*cos(q_s)),np.matrix(F_a2_1-F_b2_2))

    # 3. cost on total force for each arm
    cost3_quad_matrix, cost3_linear_matrix = general_quad_linear_cost(df_dxeq[:2,:],np.matrix([[F_y_1],[F_z_1]]))
    cost4_quad_matrix, cost4_linear_matrix = general_quad_linear_cost(df_dxeq[2:,:],np.matrix([[F_y_2],[F_z_2]]))

    # Constraint Functions
    # 1. -2*step_size <= dx_eq <= 2*step_size (limit on change in dx_eq) 
    constraint1_A = np.matrix([1,0,0,0])
    constraint1_b = np.matrix(step_size)
    constraint2_A = -np.matrix([1,0,0,0])
    constraint2_b = np.matrix(step_size)

    constraint3_A = np.matrix([0,1,0,0])
    constraint3_b = np.matrix(step_size)
    constraint4_A = -np.matrix([0,1,0,0])
    constraint4_b = np.matrix(step_size)

    constraint5_A = np.matrix([0,0,1,0])
    constraint5_b = np.matrix(step_size)
    constraint6_A = -np.matrix([0,0,1,0])
    constraint6_b = np.matrix(step_size)

    constraint7_A = np.matrix([0,0,0,1])
    constraint7_b = np.matrix(step_size)
    constraint8_A = -np.matrix([0,0,0,1])
    constraint8_b = np.matrix(step_size)

    # 2. joint limits
    constraint9_A = dq_dxeq[0,:]
    constraint9_b = np.matrix(q_t_max-q_t)
    constraint10_A = -dq_dxeq[0,:]
    constraint10_b = np.matrix(-q_t_min+q_t)

    constraint11_A = dq_dxeq[1,:]-dq_dxeq[0,:]
    constraint11_b = np.matrix(q_s_max_rel_q_t-(q_s-q_t))
    #constraint12_A = -dq_dxeq[1,:]-dq_dxeq[0,:] # possible error
    constraint12_A = -dq_dxeq[1,:]+dq_dxeq[0,:] 
    constraint12_b = np.matrix(-q_s_min_rel_q_t+(q_s-q_t))

    # 3. -mu*F_n <= F_t <= mu*F_n
    constraint13_A = df_dxeq[0,:]*cos(q_t)+df_dxeq[1,:]*sin(q_t)-mu*(-df_dxeq[0,:]*sin(q_t)+df_dxeq[1,:]*cos(q_t))
    constraint13_b = np.matrix(-F_a1_1+mu*F_a2_1)
    constraint14_A = -df_dxeq[0,:]*cos(q_t)-df_dxeq[1,:]*sin(q_t)-mu*(-df_dxeq[0,:]*sin(q_t)+df_dxeq[1,:]*cos(q_t))
    constraint14_b = np.matrix(F_a1_1+mu*F_a2_1)

    constraint15_A = df_dxeq[2,:]*cos(q_s)+df_dxeq[3,:]*sin(q_s)-mu*(-df_dxeq[2,:]*sin(q_s)+df_dxeq[3,:]*cos(q_s))
    constraint15_b = np.matrix(-F_b1_2+mu*F_b2_2)
    constraint16_A = -df_dxeq[2,:]*cos(q_s)-df_dxeq[3,:]*sin(q_s)-mu*(-df_dxeq[2,:]*sin(q_s)+df_dxeq[3,:]*cos(q_s))
    constraint16_b = np.matrix(F_b1_2+mu*F_b2_2)

    # 4. F_break <= F_n <= F_comfort
    constraint17_A = -(-df_dxeq[0,:]*sin(q_t)+df_dxeq[1,:]*cos(q_t))
    constraint17_b = np.matrix(-F_break+F_a2_1)
    constraint18_A = (-df_dxeq[0,:]*sin(q_t)+df_dxeq[1,:]*cos(q_t))
    constraint18_b = np.matrix(F_comfort-F_a2_1)
    constraint19_A = -(-df_dxeq[2,:]*sin(q_s)+df_dxeq[3,:]*cos(q_s))
    constraint19_b = np.matrix(-F_break+F_b2_2)
    constraint20_A = (-df_dxeq[2,:]*sin(q_s)+df_dxeq[3,:]*cos(q_s))
    constraint20_b = np.matrix(F_comfort-F_b2_2)
    

    # Upper and lower bounds
    # Max/min height
    ub = np.matrix([y_max-y1, z_max-z1, y_max-y2, z_max-z2]).T
    lb = np.matrix([y_min-y1, z_min-z1, y_max-y2, z_max-z2]).T


    if initialize == True:
        H = 2*(cost1_quad_matrix)
        f = cost1_linear_matrix
        A = np.concatenate([constraint1_A, constraint2_A, constraint3_A, constraint4_A, constraint5_A, constraint6_A, constraint7_A, constraint8_A, constraint13_A, constraint14_A, constraint15_A, constraint16_A])
        b = np.concatenate([constraint1_b, constraint2_b, constraint3_b, constraint4_b, constraint5_b, constraint6_b, constraint7_b, constraint8_b, constraint13_b, constraint14_b, constraint15_b, constraint16_b])
    elif friction_constraint == True:
        H = cost1_weight*(cost1_quad_matrix)+cost2_weight*(cost2_quad_matrix)+cost3_weight*(cost3_quad_matrix)+cost4_weight*(cost4_quad_matrix)
        f = (cost1_weight/2.)*cost1_linear_matrix+(cost2_weight/2.)*cost2_linear_matrix+(cost3_weight/2.)*cost3_linear_matrix+(cost4_weight/2.)*cost4_linear_matrix
        A = np.concatenate([constraint1_A, constraint2_A, constraint3_A, constraint4_A, constraint5_A, constraint6_A, constraint7_A, constraint8_A, constraint9_A, constraint10_A, constraint11_A, constraint12_A, constraint13_A, constraint14_A, constraint15_A, constraint16_A, constraint17_A, constraint18_A, constraint19_A, constraint20_A])
        b = np.concatenate([constraint1_b, constraint2_b, constraint3_b, constraint4_b, constraint5_b, constraint6_b, constraint7_b, constraint8_b, constraint9_b, constraint10_b, constraint11_b, constraint12_b, constraint13_b, constraint14_b, constraint15_b, constraint16_b, constraint17_b, constraint18_b, constraint19_b, constraint20_b])
    else:
        H = cost1_weight*(cost1_quad_matrix)+cost2_weight*(cost2_quad_matrix)+cost3_weight*(cost3_quad_matrix)+cost4_weight*(cost4_quad_matrix)
        f = (cost1_weight/2.)*cost1_linear_matrix+(cost2_weight/2.)*cost2_linear_matrix+(cost3_weight/2.)*cost3_linear_matrix+(cost4_weight/2.)*cost4_linear_matrix
        A = np.concatenate([constraint1_A, constraint2_A, constraint3_A, constraint4_A, constraint5_A, constraint6_A, constraint7_A, constraint8_A, constraint9_A, constraint10_A, constraint11_A, constraint12_A, constraint17_A, constraint18_A, constraint19_A, constraint20_A])
        b = np.concatenate([constraint1_b, constraint2_b, constraint3_b, constraint4_b, constraint5_b, constraint6_b, constraint7_b, constraint8_b, constraint9_b, constraint10_b, constraint11_b, constraint12_b, constraint17_b, constraint18_b, constraint19_b, constraint20_b])

    return H, f, A, b, lb, ub, dq_dxeq, df_dxeq, cost1_quad_matrix, cost1_linear_matrix, cost2_quad_matrix, cost2_linear_matrix, cost3_quad_matrix, cost3_linear_matrix, cost4_quad_matrix, cost4_linear_matrix

 
