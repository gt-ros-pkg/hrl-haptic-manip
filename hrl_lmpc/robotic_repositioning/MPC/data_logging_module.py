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



###################### Data Logging Module ######################
# This script provides functions to print data to a csv file

# Functions: 
# initalize_data_file 
# 	- inputs: file name (string)
#       - outputs: csv file (csv writer object)
# record_data
#	- inputs: csv file (csv writer object), data
# 	- outputs: true (bool)
#################################################################



import csv
import rospy

def initialize_data_file(data_filename):
    data_file = open(data_filename,'wb')
    csv_data_file = csv.writer(data_file)
    return csv_data_file

# can modify the input to record_data based on what you want to log
def record_data(csv_data_file, robot_MPC_state, delta_phi, dq_dxeq, df_dxeq, real_time_factor,
                cost1_quad_matrix, cost1_linear_matrix, cost2_quad_matrix, cost2_linear_matrix,
                cost3_quad_matrix, cost3_linear_matrix, cost4_quad_matrix, cost4_linear_matrix):
    csv_data_file.writerow([robot_MPC_state.actual_theta_t, robot_MPC_state.actual_theta_s, robot_MPC_state.d1, robot_MPC_state.d2,
                                robot_MPC_state.goal_positions[0], robot_MPC_state.goal_positions[1], robot_MPC_state.current_waypoint[0], robot_MPC_state.current_waypoint[1],
                                robot_MPC_state.q1, robot_MPC_state.q2, robot_MPC_state.y1, robot_MPC_state.z1, robot_MPC_state.y2, robot_MPC_state.z2,
                                robot_MPC_state.q_eq1, robot_MPC_state.q_eq2, robot_MPC_state.y_eq1, robot_MPC_state.z_eq1, robot_MPC_state.y_eq2, robot_MPC_state.z_eq2,
                                robot_MPC_state.F_a1_1, robot_MPC_state.F_a2_1, robot_MPC_state.F_b1_2, robot_MPC_state.F_b2_2,
                                delta_phi[0,0], delta_phi[1,0], delta_phi[2,0], delta_phi[3,0],
                                robot_MPC_state.i, robot_MPC_state.previous_waypoint[0], robot_MPC_state.previous_waypoint[1],
                                robot_MPC_state.F_y_1, robot_MPC_state.F_z_1, robot_MPC_state.F_y_2, robot_MPC_state.F_z_2,
                                dq_dxeq[0,0], dq_dxeq[0,1], dq_dxeq[0,2], dq_dxeq[0,3],
                                dq_dxeq[1,0], dq_dxeq[1,1], dq_dxeq[1,2], dq_dxeq[1,3],
                                df_dxeq[0,0], df_dxeq[0,1], df_dxeq[0,2], df_dxeq[0,3],
                                df_dxeq[1,0], df_dxeq[1,1], df_dxeq[1,2], df_dxeq[1,3],
                                df_dxeq[2,0], df_dxeq[2,1], df_dxeq[2,2], df_dxeq[2,3],
                                df_dxeq[3,0], df_dxeq[3,1], df_dxeq[3,2], df_dxeq[3,3], real_time_factor,
                                cost1_quad_matrix[0,0], cost1_quad_matrix[0,1], cost1_quad_matrix[0,2], cost1_quad_matrix[0,3],
                                cost1_quad_matrix[0,0], cost1_quad_matrix[0,1], cost1_quad_matrix[0,2], cost1_quad_matrix[0,3],
                                cost1_quad_matrix[0,0], cost1_quad_matrix[0,1], cost1_quad_matrix[0,2], cost1_quad_matrix[0,3],
                                cost1_quad_matrix[0,0], cost1_quad_matrix[0,1], cost1_quad_matrix[0,2], cost1_quad_matrix[0,3],
                                cost1_linear_matrix[0,0], cost1_linear_matrix[0,1], cost1_linear_matrix[0,2], cost1_linear_matrix[0,3],
                                cost2_quad_matrix[0,0], cost2_quad_matrix[0,1], cost2_quad_matrix[0,2], cost2_quad_matrix[0,3],
                                cost2_quad_matrix[0,0], cost2_quad_matrix[0,1], cost2_quad_matrix[0,2], cost2_quad_matrix[0,3],
                                cost2_quad_matrix[0,0], cost2_quad_matrix[0,1], cost2_quad_matrix[0,2], cost2_quad_matrix[0,3],
                                cost2_quad_matrix[0,0], cost2_quad_matrix[0,1], cost2_quad_matrix[0,2], cost2_quad_matrix[0,3],
                                cost2_linear_matrix[0,0], cost2_linear_matrix[0,1], cost2_linear_matrix[0,2], cost2_linear_matrix[0,3],
                                cost3_quad_matrix[0,0], cost3_quad_matrix[0,1], cost3_quad_matrix[0,2], cost3_quad_matrix[0,3],
                                cost3_quad_matrix[0,0], cost3_quad_matrix[0,1], cost3_quad_matrix[0,2], cost3_quad_matrix[0,3],
                                cost3_quad_matrix[0,0], cost3_quad_matrix[0,1], cost3_quad_matrix[0,2], cost3_quad_matrix[0,3],
                                cost3_quad_matrix[0,0], cost3_quad_matrix[0,1], cost3_quad_matrix[0,2], cost3_quad_matrix[0,3],
                                cost3_linear_matrix[0,0], cost3_linear_matrix[0,1], cost3_linear_matrix[0,2], cost3_linear_matrix[0,3],
                                cost4_quad_matrix[0,0], cost4_quad_matrix[0,1], cost4_quad_matrix[0,2], cost4_quad_matrix[0,3],
                                cost4_quad_matrix[0,0], cost4_quad_matrix[0,1], cost4_quad_matrix[0,2], cost4_quad_matrix[0,3],
                                cost4_quad_matrix[0,0], cost4_quad_matrix[0,1], cost4_quad_matrix[0,2], cost4_quad_matrix[0,3],
                                cost4_quad_matrix[0,0], cost4_quad_matrix[0,1], cost4_quad_matrix[0,2], cost4_quad_matrix[0,3],
                                cost4_linear_matrix[0,0], cost4_linear_matrix[0,1], cost4_linear_matrix[0,2], cost4_linear_matrix[0,3], rospy.Time.now().to_sec()])
    return True

 
