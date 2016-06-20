#!/usr/bin/env python

# Software License Agreement (New BSD License)
#
# Copyright (c) 2014, Georgia Tech Research Corporation
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
# Authors: Kevin Chow, Jeff Hawke, Advait Jain, Charlie Kemp
# Healthcare Robotics Laboratory
#
# This module contains the core mathematics used for the LMPC.

import copy
import openopt as pp # python-openopt package is required for the QP solver
import itertools as it
import numpy as np, math
import tf
import hrl_lib.util as ut

# currently being used
# computes weighted matrix for jerk cost
def min_jerk_quadratic_matrix(jerk_opt_weight, K_j):
    return (jerk_opt_weight * K_j.T * K_j)

# currently being used
# computes the distance from the current joint angles to the joint limits in terms of delta joint angles
def joint_limit_bounds(min_q, max_q, v):
    m = v.shape[0]
   
    min_q = (np.matrix(min_q).T)[0:m]
    max_q = (np.matrix(max_q).T)[0:m]

    delta_q_max = np.maximum(max_q - v, 0.)
    delta_q_min = np.minimum(min_q - v, 0.)

    return delta_q_min, delta_q_max

# currently being used
# computes the distance from the difference between the current joint angles and the current JEP and the difference limit
# used to constrain the new joint angles from being too far from the JEP
def theta_phi_absolute_difference_bounds(theta, phi, max_diff):
    phi_min_abs = theta - np.maximum(theta - phi, max_diff)
    phi_max_abs = theta + np.maximum(phi - theta, max_diff)

    return phi_min_abs - phi, phi_max_abs - phi

# currently being used
# calculates the maximum allowed angle of the forearms based on the friction coefficient of the object
def friction_constraint(miu):
    max_angle = math.atan(miu)
   
    return max_angle

# not being used
# calculates the maximum allowed angle of the forearms based on the dimensions of the object, so that it does not topple over
# current implementation only useful for rectangular objects
def toppling_constraint(object_height, object_width, object_depth):
    forearm_width = 0.6 
    forearm_depth = 0.2

    h = object_height
    w = min(object_width, forearm_width)
    d = min(object_depth, forearm_depth)

    toppling_forward_angle = math.atan(d/h)
    toppling_sideways_angle = math.atan(w/h)

    return min(toppling_forward_angle, toppling_sideways_angle)

# currently being used
# calculates the change in the alpha angle (forearm angle with respect to the ground plane) as the robot joint angles change
def d_alpha_d_theta(friction_p1_pose, friction_J1, friction_p2_pose, friction_J2, friction_p3_pose, friction_J3):
    # calculate alpha based on plane from three points on arm and plane
    vector1 = np.array(friction_p2_pose-friction_p1_pose)
    vector2 = np.array(friction_p3_pose-friction_p1_pose)
    vector1 = vector1.reshape(1,3)
    vector2 = vector2.reshape(1,3)
    friction_norm = np.cross(vector1[0],vector2[0])/np.linalg.norm(np.cross(vector1[0],vector2[0]))
    alpha = math.acos(np.dot(np.array([0,0,1]),friction_norm))

    f_J1 = np.matrix(friction_J1[0][:3])
    f_J2 = np.matrix(friction_J2[0][:3])
    f_J3 = np.matrix(friction_J3[0][:3])

    d_v2v1_d_theta = (f_J2-f_J1)
    d_v3v1_d_theta = (f_J3-f_J1)
    d_alpha_d_theta = []
    d_f2_d_theta = []
    for i in range(7):
        mult_mat = np.matrix(np.zeros(7)).reshape(7,1)
        mult_mat[i] = 1.
        temp1 = np.array(d_v2v1_d_theta*mult_mat).reshape(1,3)
        temp2 = np.array(d_v3v1_d_theta*mult_mat).reshape(1,3)
        temp = (np.cross(temp1,vector2)+np.cross(vector1,temp2))/np.linalg.norm(np.cross(vector1[0],vector2[0]))
        d_f2_d_theta.append(temp.tolist()[0])
        d_alpha_d_theta_num = -np.dot(np.array([0,0,1]),np.array(temp).reshape(1,3)[0])
        d_alpha_d_theta_den = math.sqrt(1-np.dot(np.array([0,0,1]),friction_norm)**2)
        d_alpha_d_theta.append(float(d_alpha_d_theta_num/d_alpha_d_theta_den))

    return d_alpha_d_theta, friction_norm, alpha

# currently not being used
# cost function that tries to keep the arms coplanar with the body at the place of contact
# uses 3 points on the arms to create a plane on the forearm 
# assumes that the body pose at the locations of contact between the arms and the body are known
# tries to match the forearm planes and the body planes by minimizing the angle between them
# in order to use this cost function, you may have to modify the robot state node to publish the jacobians and pose of the 3 points on the arm
# ask kevin about this one if you have questions...the math is somewhat involved
def arm_body_costs(larm_friction_p1_pose, larm_friction_p2_pose, larm_friction_p3_pose, rarm_friction_p1_pose, rarm_friction_p2_pose, rarm_friction_p3_pose, larm_friction_J1, larm_friction_J2, larm_friction_J3, rarm_friction_J1, rarm_friction_J2, rarm_friction_J3, object_lower_body_pose, object_l_thigh_pose, object_r_thigh_pose, object_upper_body_pose):
    lb_vector1 = np.array(object_l_thigh_pose-object_lower_body_pose)
    lb_vector2 = np.array(object_r_thigh_pose-object_lower_body_pose)
    lb_vector1 = lb_vector1.reshape(1,3)
    lb_vector2 = lb_vector2.reshape(1,3)
    lower_body_norm = np.cross(lb_vector1[0],lb_vector2[0])/np.linalg.norm(np.cross(lb_vector1[0],lb_vector2[0])) 
    lower_body_vector = ((lb_vector1+lb_vector2)/2)/np.linalg.norm((lb_vector1+lb_vector2)/2)
    l_vector1 = np.array(larm_friction_p2_pose-larm_friction_p1_pose)
    l_vector2 = np.array(larm_friction_p3_pose-larm_friction_p1_pose)
    l_vector1 = l_vector1.reshape(1,3)
    l_vector2 = l_vector2.reshape(1,3)
    l_friction_norm = np.cross(l_vector1[0],l_vector2[0])/np.linalg.norm(np.cross(l_vector1[0],l_vector2[0]))
    l_beta = math.acos(np.dot(lower_body_norm,l_friction_norm))
    object_l_friction_norm_dot = np.dot(lower_body_vector,l_friction_norm)
    d_v2v1_d_theta = (larm_friction_J2[0][:3]-larm_friction_J1[0][:3])
    d_v3v1_d_theta = (larm_friction_J3[0][:3]-larm_friction_J1[0][:3])
    dot_vector_1 = np.array((object_l_thigh_pose+object_r_thigh_pose)/2-object_lower_body_pose)/np.linalg.norm(np.array((object_l_thigh_pose+object_r_thigh_pose)/2-object_lower_body_pose))
    dot_vector_2 = copy.copy(l_vector1)/np.linalg.norm(l_vector1)
    object_l_forearm_dot = np.dot(dot_vector_1.reshape(1,3)[0],dot_vector_2[0])
    object_l_forearm_dot = np.dot(lower_body_vector[0],l_vector1[0])
    d_l_beta_d_theta = []
    d_object_l_forearm_dot_d_theta = []
    d_object_l_friction_norm_dot_d_theta = []
    for i in range(7):
        mult_mat = np.matrix([[0.],[0.],[0.],[0.],[0.],[0.],[0.]])
        mult_mat[i] = 1.
        temp1 = np.array(d_v2v1_d_theta*mult_mat).reshape(1,3)
        temp2 = np.array(d_v3v1_d_theta*mult_mat).reshape(1,3)
        temp = (np.cross(temp1,l_vector2)+np.cross(l_vector1,temp2))/np.linalg.norm(np.cross(l_vector1[0],l_vector2[0]))
	d_object_l_forearm_dot_d_theta.append(float(np.dot(np.array(dot_vector_1).reshape(1,3)[0],np.array(temp1).reshape(1,3)[0])))
	d_object_l_friction_norm_dot_d_theta.append(float(np.dot(lower_body_vector,np.array(temp).reshape(1,3)[0])))    
	
    rb_vector1 = np.array(object_upper_body_pose-object_lower_body_pose)
    rb_vector1 = rb_vector1.reshape(1,3)
    upper_body_vector = rb_vector1/np.linalg.norm(rb_vector1)
    r_vector1 = np.array(rarm_friction_p2_pose-rarm_friction_p1_pose)
    r_vector2 = np.array(rarm_friction_p3_pose-rarm_friction_p1_pose)
    r_vector1 = r_vector1.reshape(1,3)
    r_vector2 = r_vector2.reshape(1,3)
    r_friction_norm = np.cross(r_vector1[0],r_vector2[0])/np.linalg.norm(np.cross(r_vector1[0],r_vector2[0]))
    d_v2v1_d_theta = (rarm_friction_J2[0][:3]-rarm_friction_J1[0][:3])
    d_v3v1_d_theta = (rarm_friction_J3[0][:3]-rarm_friction_J1[0][:3])
    d_object_r_forearm_dot_d_theta = []
    d_object_r_friction_norm_dot_d_theta = []
    object_r_friction_norm_dot = np.dot(upper_body_vector,r_friction_norm)
    object_r_forearm_dot = np.dot(upper_body_vector[0],r_vector1[0])
    for i in range(7):
        mult_mat = np.matrix([[0.],[0.],[0.],[0.],[0.],[0.],[0.]])
        mult_mat[i] = 1.
        temp1 = np.array(d_v2v1_d_theta*mult_mat).reshape(1,3)
        temp2 = np.array(d_v3v1_d_theta*mult_mat).reshape(1,3)
        temp = (np.cross(temp1,l_vector2)+np.cross(l_vector1,temp2))/np.linalg.norm(np.cross(l_vector1[0],l_vector2[0]))
	d_object_r_forearm_dot_d_theta.append(float(np.dot(upper_body_vector,np.array(temp1).reshape(1,3)[0])))    
	d_object_r_friction_norm_dot_d_theta.append(float(np.dot(upper_body_vector,np.array(temp).reshape(1,3)[0])))    

    if object_l_friction_norm_dot < 0:
	d_object_l_friction_norm_dot_d_theta = -np.array(d_object_l_friction_norm_dot_d_theta)
    if object_l_forearm_dot < 0:
	d_object_l_forearm_dot_d_theta = -np.array(d_object_l_forearm_dot_d_theta)
    if object_r_friction_norm_dot < 0:
	d_object_r_friction_norm_dot_d_theta = -np.array(d_object_r_friction_norm_dot_d_theta)
    if object_r_forearm_dot < 0:
	d_object_r_forearm_dot_d_theta = -np.array(d_object_r_forearm_dot_d_theta)

    return d_object_l_forearm_dot_d_theta, d_object_l_friction_norm_dot_d_theta, d_object_r_forearm_dot_d_theta, d_object_r_friction_norm_dot_d_theta

# not currently being used
# calculates the current center of mass of the ragdoll based on the pose of the 3 sections of the ragdoll 
def calculate_com(ragdoll_m1, ragdoll_m1_pose, ragdoll_m2, ragdoll_m2_pose, ragdoll_m3, ragdoll_m3_pose):
    x_com = ragdoll_m1*ragdoll_m1_pose[0,0]+ragdoll_m2*ragdoll_m2_pose[0,0]+ragdoll_m3*ragdoll_m3_pose[0,0]
    y_com = ragdoll_m1*ragdoll_m1_pose[1,0]+ragdoll_m2*ragdoll_m2_pose[1,0]+ragdoll_m3*ragdoll_m3_pose[1,0]
    z_com = ragdoll_m1*ragdoll_m1_pose[2,0]+ragdoll_m2*ragdoll_m2_pose[2,0]+ragdoll_m3*ragdoll_m3_pose[2,0]
    return x_com, y_com, z_com

# currently being used
# cost function that tries to minimize the height difference between the two robot forearms (z in the world frame)
def arm_height_difference_cost(larm_J_forearm,rarm_J_forearm,larm_x_forearm,rarm_x_forearm):
    F_1 = np.hstack((larm_J_forearm[2,:],-rarm_J_forearm[2,:]))
    arm_height_difference = larm_x_forearm[2,0]-rarm_x_forearm[2,0]
    #print "arm_height_difference: ",larm_x_forearm[2,0]-rarm_x_forearm[2,0]
    cost_quad_matrix = F_1.T*F_1
    cost_linear_matrix = 2.*(larm_x_forearm[2,0]-rarm_x_forearm[2,0])*F_1   
 
    return cost_quad_matrix, cost_linear_matrix, abs(arm_height_difference)

# currently being used
# cost function that tries to minimize the width difference between the two robot forearms (y in the world frame)
def arm_width_difference_cost(larm_J_forearm,rarm_J_forearm,larm_x_forearm,rarm_x_forearm,arm_width_distance):
    F_1 = np.hstack((larm_J_forearm[1,:],-rarm_J_forearm[1,:]))
    arm_width_difference = larm_x_forearm[1,0]-rarm_x_forearm[1,0]
    print "arm_width_difference: ",larm_x_forearm[1,0]-rarm_x_forearm[1,0]
    cost_quad_matrix = F_1.T*F_1
    #cost_linear_matrix = 2.*(0.53)*F_1
    cost_linear_matrix = 2.*(arm_width_difference-arm_width_distance)*F_1

    return cost_quad_matrix, cost_linear_matrix, abs(arm_width_difference)

# not being used
# cost function that tries to minimize the difference in force between the two robot forearms (z in the world frame) by adjusting their heights
def arm_height_force_difference_cost(larm_J_forearm,rarm_J_forearm,larm_force,rarm_force):
    F_1 = np.hstack((larm_J_forearm[2,:],-rarm_J_forearm[2,:]))
    force_difference = larm_force[2]-rarm_force[2]
    cost_quad_matrix = F_1.T*F_1
    cost_linear_matrix = 2.*(-larm_force[2]+rarm_force[2])*F_1

    return cost_quad_matrix, cost_linear_matrix, abs(force_difference)


# not currently used 
# this cost function is attended to minimize the moment of the arms with respect to the body
# not entirely sure how this would fit in with the quasistatic model of the situation
def moment_balancing_cost(larm_J_forearm,rarm_J_forearm,larm_x_forearm,rarm_x_forearm,larm_force,rarm_force,object_x):
    # ********** We are balancing the moments on the object, so we need to map the forces from the robot arms to the link object ********
    larm_force *= -1
    rarm_force *= -1
    # cost = cross(larm_force,(larm_x_forearm+larm_J_forearm*delta_larm_phi)-object_COM)-cross(rarm_force,(rarm_x_forearm+rarm_J_forearm*delta_rarm_phi)-object_COM)
    #F_1_x = np.hstack((-larm_force[2]*larm_J_forearm[1,:]-larm_force[1]*larm_J_forearm[2,:],-rarm_force[2]*rarm_J_forearm[1,:]-rarm_force[1]*rarm_J_forearm[2,:]))
    F_1_x = np.hstack((np.zeros((1,7)),-rarm_force[2]*rarm_J_forearm[1,:]))
    #F_1_x = np.hstack((rarm_force[2]*rarm_J_forearm[1,:],larm_force[2]*larm_J_forearm[1,:]))
    F_1_y = np.hstack((larm_force[0]*larm_J_forearm[2,:]-larm_force[2]*larm_J_forearm[0,:],rarm_force[0]*rarm_J_forearm[2,:]-rarm_force[2]*rarm_J_forearm[0,:]))
    F_1_z = np.hstack((larm_force[0]*larm_J_forearm[1,:]-larm_force[1]*larm_J_forearm[0,:],rarm_force[0]*rarm_J_forearm[1,:]-rarm_force[1]*rarm_J_forearm[0,:]))
    F_2_x = rarm_force[2]*(object_x[1]-rarm_x_forearm[1,0])+rarm_force[1]*(object_x[2]-rarm_x_forearm[2,0])-larm_force[2]*(object_x[1]-larm_x_forearm[1,0])+larm_force[1]*(object_x[2]-larm_x_forearm[2,0])
    #F_2_x = -rarm_force[2]*(rarm_x_forearm[1,0]-object_x[1])-larm_force[2]*(larm_x_forearm[1,0]-object_x[1])
    F_2_y = -rarm_force[0]*(rarm_x_forearm[2,0]-object_x[2])+rarm_force[2]*(rarm_x_forearm[0,0]-object_x[0])-larm_force[0]*(larm_x_forearm[2,0]-object_x[2])+larm_force[2]*(larm_x_forearm[0,0]-object_x[0])
    F_2_z = -rarm_force[0]*(rarm_x_forearm[1,0]-object_x[1])+rarm_force[1]*(rarm_x_forearm[0,0]-object_x[0])-larm_force[0]*(larm_x_forearm[1,0]-object_x[1])+larm_force[1]*(larm_x_forearm[0,0]-object_x[0])
    cost_quad_matrix_x = F_1_x.T*F_1_x
    cost_linear_matrix_x = 2.*F_2_x*F_1_x
    cost_quad_matrix_y = F_1_y.T*F_1_y
    cost_linear_matrix_y = 2.*F_2_y*F_1_y
    cost_quad_matrix_z = F_1_z.T*F_1_z
    cost_linear_matrix_z = 2.*F_2_z*F_1_z
    larm_moment = np.cross(larm_force.T,(object_x-larm_x_forearm).T)
    rarm_moment = np.cross(rarm_force.T,(object_x-rarm_x_forearm).T)
    delta_larm_moment_x = -F_1_x[0,:7] 
    delta_rarm_moment_x = -F_1_x[0,7:]
    delta_lforearm_x = copy.copy(larm_J_forearm[:3,:])
    delta_rforearm_x = copy.copy(rarm_J_forearm[:3,:])
    delta_rarm_moment_x_force1 = rarm_force[1]*rarm_J_forearm[2,:]
    delta_rarm_moment_x_force2 = rarm_force[2]*rarm_J_forearm[1,:]

    return cost_quad_matrix_x, cost_linear_matrix_x, cost_quad_matrix_y, cost_linear_matrix_y, cost_quad_matrix_z, cost_linear_matrix_z, larm_moment+rarm_moment, larm_moment, rarm_moment, delta_larm_moment_x, delta_rarm_moment_x, delta_lforearm_x, delta_rforearm_x, delta_rarm_moment_x_force1, delta_rarm_moment_x_force2

# currently being used
# cost function that attempts to minimize the joint torque experienced by the robot forearms by rotating the forearm
def forearm_jt_cost(forearm_jt):
    #print "forearm_jt: ",forearm_jt
    K = 1
    forearm_matrix = np.zeros((1,7))
    forearm_matrix[0][5] = 1.
    cost_linear_matrix = 2.*(-1*K*forearm_jt)*(-forearm_matrix) 
    return cost_linear_matrix

# currently being used
# general function to create cost functions in the form of minimizing the distance to a goal for a jacobian multiplied to a change in joint angles
def general_quad_linear_cost(J, goal):
    cost_quad_matrix = J.T*J
    cost_linear_matrix = 2.*goal.T*(-J)
  
    return cost_quad_matrix, cost_linear_matrix


# main function that is called by crona2_twoarm_main.py
# generates cost functions, constraints, lower bounds, and upper bounds in a form that may be accepted by the optimization solver
# also gathers the weights to the various cost functions from the parameter file
# may be modified to change the cost functions/constraints/lower bounds/upper bounds directly
def convert_to_qp_posture(mpc_dat, larm_object_J, rarm_object_J, lforearm_J, rforearm_J, ragdoll_m2_J, ragdoll_m3_J, delta_object_x_g, delta_lforearm_x_g, delta_rforearm_x_g, delta_m2_x_g, delta_m3_x_g, larm_joint_limits_min, rarm_joint_limits_min, larm_joint_limits_max, rarm_joint_limits_max, angle_constraint_threshold):

    cost_quadratic_matrices = []
    cost_linear_matrices = []
    constraint_matrices = []
    constraint_vectors = []
    constraint_matrices_eq = []
    constraint_vectors_eq = []

    ## estimate weight of object and scale larm/rarm force in z direction
    ## if there is no object, then don't scale
    torso_jt = mpc_dat.torso_jt 
    moment_arm = mpc_dat.object_x[0]-mpc_dat.torso_x[0]
    estimated_weight = torso_jt/moment_arm

    # scale larm_force/rarm_force in z direction
    old_larm_force = copy.copy(mpc_dat.larm_force)
    old_rarm_force = copy.copy(mpc_dat.rarm_force)
    if old_larm_force[2] < 0 and old_rarm_force[2] < 0:
    	mpc_dat.larm_force[2] = (old_larm_force[2]/(abs(old_larm_force[2])+abs(old_rarm_force[2])))*estimated_weight
    	mpc_dat.rarm_force[2] = (old_rarm_force[2]/(abs(old_larm_force[2])+abs(old_rarm_force[2])))*estimated_weight

    arm_height_quad_matrix, arm_height_linear_matrix, arm_height_difference = arm_height_difference_cost(lforearm_J,rforearm_J,mpc_dat.lforearm_x,mpc_dat.rforearm_x) 
    arm_width_distance = abs(mpc_dat.rforearm_x_g[1]-mpc_dat.lforearm_x_g[1])
    arm_width_quad_matrix, arm_width_linear_matrix, arm_width_difference = arm_width_difference_cost(lforearm_J,rforearm_J,mpc_dat.lforearm_x,mpc_dat.rforearm_x,arm_width_distance) 
    arm_height_quad_matrix *= 10*arm_height_difference
    arm_height_linear_matrix *= 10*arm_height_difference
    arm_width_quad_matrix *= 10*arm_width_difference
    arm_width_linear_matrix *= 10*arm_width_difference

    arm_height_force_quad_matrix, arm_height_force_linear_matrix, force_difference = arm_height_difference_cost(lforearm_J,rforearm_J,mpc_dat.larm_force,mpc_dat.rarm_force)
    arm_height_force_quad_matrix *= 0.0001*force_difference
    arm_height_force_linear_matrix *= 0.0001*force_difference

    ## Apply the pose weights: position & orientation.
    # These are sqrt'd as the quadratic term takes the form Jh^T * Jh, and the linear is delta_x_g * Jh
    #orient_weight = 1.
    delta_object_x_g[:3] = delta_object_x_g[:3] * np.sqrt(mpc_dat.object_pos_weight)
    delta_object_x_g[3:] = delta_object_x_g[3:] * np.sqrt(mpc_dat.object_orient_weight)
    larm_object_J[:3] = larm_object_J[:3] * np.sqrt(mpc_dat.object_pos_weight)
    larm_object_J[3:] = larm_object_J[3:] * np.sqrt(mpc_dat.object_orient_weight)  
    rarm_object_J[:3] = rarm_object_J[:3] * np.sqrt(mpc_dat.object_pos_weight)
    rarm_object_J[3:] = rarm_object_J[3:] * np.sqrt(mpc_dat.object_orient_weight)

    delta_lforearm_x_g[:3] = delta_lforearm_x_g[:3] * np.sqrt(mpc_dat.forearm_pos_weight)
    delta_lforearm_x_g[3:] = delta_lforearm_x_g[3:] * np.sqrt(mpc_dat.forearm_orient_weight)
    delta_rforearm_x_g[:3] = delta_rforearm_x_g[:3] * np.sqrt(mpc_dat.forearm_pos_weight)
    delta_rforearm_x_g[3:] = delta_rforearm_x_g[3:] * np.sqrt(mpc_dat.forearm_orient_weight)
    lforearm_J[:3] = lforearm_J[:3] * np.sqrt(mpc_dat.forearm_pos_weight)
    lforearm_J[3:] = lforearm_J[3:] * np.sqrt(mpc_dat.forearm_orient_weight)
    rforearm_J[:3] = rforearm_J[:3] * np.sqrt(mpc_dat.forearm_pos_weight)
    rforearm_J[3:] = rforearm_J[3:] * np.sqrt(mpc_dat.forearm_orient_weight)

    delta_m2_x_g[:3] = delta_m2_x_g[:3] * np.sqrt(mpc_dat.object_pos_weight)
    delta_m2_x_g[3:] = delta_m2_x_g[3:] * np.sqrt(mpc_dat.object_orient_weight)
    delta_m3_x_g[:3] = delta_m3_x_g[:3] * np.sqrt(mpc_dat.object_pos_weight)
    delta_m3_x_g[3:] = delta_m3_x_g[3:] * np.sqrt(mpc_dat.object_orient_weight)
    ragdoll_m2_J[:3] = ragdoll_m2_J[:3] * np.sqrt(mpc_dat.object_pos_weight)
    ragdoll_m2_J[3:] = ragdoll_m2_J[3:] * np.sqrt(mpc_dat.object_orient_weight)
    ragdoll_m3_J[:3] = ragdoll_m3_J[:3] * np.sqrt(mpc_dat.object_pos_weight)
    ragdoll_m3_J[3:] = ragdoll_m3_J[3:] * np.sqrt(mpc_dat.object_orient_weight)

    cost_quad_matrix_larm_object, cost_linear_matrix_larm_object = general_quad_linear_cost(larm_object_J, delta_object_x_g) 
    cost_quad_matrix_rarm_object, cost_linear_matrix_rarm_object = general_quad_linear_cost(rarm_object_J, delta_object_x_g) 
    Q_arm_object = np.zeros((14,14))
    Q_arm_object[:7,:7] = cost_quad_matrix_larm_object
    Q_arm_object[7:,7:] = cost_quad_matrix_rarm_object
    L_arm_object = np.hstack((cost_linear_matrix_larm_object, cost_linear_matrix_rarm_object))

    cost_quad_matrix_lforearm, cost_linear_matrix_lforearm = general_quad_linear_cost(lforearm_J, delta_lforearm_x_g)
    cost_quad_matrix_rforearm, cost_linear_matrix_rforearm = general_quad_linear_cost(rforearm_J, delta_rforearm_x_g)
    Q_forearm = np.zeros((14,14))
    Q_forearm[:7,:7] = cost_quad_matrix_lforearm
    Q_forearm[7:,7:] = cost_quad_matrix_rforearm
    L_forearm = np.hstack((cost_linear_matrix_lforearm, cost_linear_matrix_rforearm))

    cost_quad_matrix_ragdoll_m2, cost_linear_matrix_ragdoll_m2 = general_quad_linear_cost(ragdoll_m2_J, delta_m2_x_g)
    cost_quad_matrix_ragdoll_m3, cost_linear_matrix_ragdoll_m3 = general_quad_linear_cost(ragdoll_m3_J, delta_m3_x_g)
    Q_ragdoll_m2m3 = np.zeros((14,14))
    Q_ragdoll_m2m3[:7,:7] = cost_quad_matrix_ragdoll_m3
    Q_ragdoll_m2m3[7:,7:] = cost_quad_matrix_ragdoll_m2
    L_ragdoll_m2m3 = np.hstack((cost_linear_matrix_ragdoll_m3, cost_linear_matrix_ragdoll_m2))

    cost_quad_matrix_momentx, cost_linear_matrix_momentx, cost_quad_matrix_momenty, cost_linear_matrix_momenty, cost_quad_matrix_momentz, cost_linear_matrix_momentz, net_moment, current_larm_moment, current_rarm_moment, delta_larm_moment, delta_rarm_moment, delta_lforearm_x, delta_rforearm_x, delta_rarm_moment_x_force1, delta_rarm_moment_x_force2 = moment_balancing_cost(lforearm_J,rforearm_J,mpc_dat.lforearm_x,mpc_dat.rforearm_x,mpc_dat.larm_force,mpc_dat.rarm_force,mpc_dat.object_x)

    cost_quad_matrix_momentx *= abs(net_moment[0][0])
    cost_linear_matrix_momentx *= abs(net_moment[0][0])

    cost_linear_matrix_lforearm_orient = forearm_jt_cost(mpc_dat.larm_jt[4])
    cost_linear_matrix_rforearm_orient = forearm_jt_cost(mpc_dat.rarm_jt[4])
    L_forearm_orient = np.hstack((cost_linear_matrix_lforearm_orient,cost_linear_matrix_rforearm_orient))    

    # forearm angle constraint for slipping
    slipping_angle = friction_constraint(mpc_dat.miu)

    larm_K_j_t = mpc_dat.larm_K_j
    rarm_K_j_t = mpc_dat.rarm_K_j
    larm_min_jerk_mat = min_jerk_quadratic_matrix(mpc_dat.jerk_opt_weight, larm_K_j_t)
    rarm_min_jerk_mat = min_jerk_quadratic_matrix(mpc_dat.jerk_opt_weight, rarm_K_j_t)
    min_jerk_mat = np.zeros((14,14))
    min_jerk_mat[:7,:7] = larm_min_jerk_mat
    min_jerk_mat[7:14,7:14] = rarm_min_jerk_mat

    # New matrices - merge Q_pose, Q_posture, etc
    cost_quadratic_matrices += [1. * Q_arm_object, 1. * Q_forearm, 1. * arm_height_quad_matrix, 1. * arm_width_quad_matrix, 0. * arm_height_force_quad_matrix, 0. * cost_quad_matrix_momentx, 0. * cost_quad_matrix_momenty, 0. * cost_quad_matrix_momentz, 0. * Q_ragdoll_m2m3, 1. * min_jerk_mat]
    #cost_quadratic_matrices += [1. * Q_arm_object, 1. * Q_forearm, 0. * arm_height_quad_matrix, 0. * arm_width_quad_matrix, 1. * cost_quad_matrix_momentx, 0. * cost_quad_matrix_momenty, 0. * cost_quad_matrix_momentz, 0. * Q_ragdoll_m2m3, 1. * min_jerk_mat]
    cost_linear_matrices += [1. * L_arm_object, 1. * L_forearm, 1. * arm_height_linear_matrix, 1. * arm_width_linear_matrix, 0. * arm_height_force_linear_matrix, 0. * cost_linear_matrix_momentx, 0. * cost_linear_matrix_momenty, 0. * cost_linear_matrix_momentz, 0. * L_ragdoll_m2m3, 1. * L_forearm_orient]
    #cost_linear_matrices += [1. * L_arm_object, 1. * L_forearm, 0. * arm_height_linear_matrix, 0. * arm_width_linear_matrix, 1. * cost_linear_matrix_momentx, 0. * cost_linear_matrix_momenty, 0. * cost_linear_matrix_momentz, 0. * L_ragdoll_m2m3, 1. * L_forearm_orient]
           
    D8 = np.eye(14)
    max_joint_velocity = 0.001*np.matrix(np.ones((14,1)))
    #max_joint_velocity = 0.1*np.matrix(np.ones((14,1)))
    constraint_matrices.append(D8)
    constraint_vectors.append(max_joint_velocity)
    constraint_matrices.append(-D8)
    constraint_vectors.append(max_joint_velocity)

    ### forearm angle constraint/cost
    max_angle = slipping_angle

    larm_D7, larm_friction_norm, larm_alpha = d_alpha_d_theta(mpc_dat.larm_friction_p1_pose, mpc_dat.larm_friction_J1, mpc_dat.larm_friction_p2_pose, mpc_dat.larm_friction_J2, mpc_dat.larm_friction_p3_pose, mpc_dat.larm_friction_J3)
    delta_max_angle = max_angle-math.acos(np.dot(np.array([0,0,1]),larm_friction_norm))

    larm_D7 = np.matrix(np.hstack((np.array(larm_D7),np.zeros((1,7))[0])))
    delta_max_angle = np.matrix(delta_max_angle)
    larm_D7[0][0,0] *= -1
    if larm_alpha > max_angle:
        cost_linear_matrices.append(10.*larm_D7)
    if larm_alpha <= max_angle:
        constraint_matrices.append(larm_D7)
        constraint_vectors.append(delta_max_angle)

    max_angle = slipping_angle
    rarm_D7, rarm_friction_norm, rarm_alpha = d_alpha_d_theta(mpc_dat.rarm_friction_p1_pose, mpc_dat.rarm_friction_J1, mpc_dat.rarm_friction_p2_pose, mpc_dat.rarm_friction_J2, mpc_dat.rarm_friction_p3_pose, mpc_dat.rarm_friction_J3)
    delta_max_angle = max_angle-math.acos(np.dot(np.array([0,0,1]),rarm_friction_norm))

    rarm_D7 = np.matrix(np.hstack((np.zeros((1,7))[0],np.array(rarm_D7))))
    delta_max_angle = np.matrix(delta_max_angle)
    rarm_D7[0][0,7] *= -1

    if rarm_alpha > max_angle:
        cost_linear_matrices.append(10*rarm_D7)
    if rarm_alpha <= max_angle:
        constraint_matrices.append(rarm_D7)
        constraint_vectors.append(delta_max_angle)
    
    # Make sure that the delta torso joint angle is the same for both arms 
    D9 = np.zeros((1,14))
    D9[0,0] = 1. 
    D9[0,7] = -1.
    constraint_matrices_eq.append(D9)
    constraint_vectors_eq.append(np.zeros((1,1))) 
    
    # Try to make the delta object pose for both arms be the same
    D10 = np.zeros((6,14))
    D10[:3,:7] = larm_object_J[:3,:]
    D10[:3,7:] = -rarm_object_J[:3,:]
    D10 = np.matrix(D10)
    D11 = D10.T * D10
    cost_quadratic_matrices.append(1*D11)
   
    # joint limit bounds and current joint angle/JEP difference bounds 
    larm_delta_phi_min, larm_delta_phi_max = joint_limit_bounds(larm_joint_limits_min, larm_joint_limits_max, mpc_dat.larm_phi_curr)
    larm_delta_phi_min2, larm_delta_phi_max2 = theta_phi_absolute_difference_bounds(np.matrix(mpc_dat.larm_q).T,
                                                                          mpc_dat.larm_phi_curr,
                                                                          angle_constraint_threshold)
    rarm_delta_phi_min, rarm_delta_phi_max = joint_limit_bounds(rarm_joint_limits_min, rarm_joint_limits_max, mpc_dat.rarm_phi_curr)
    rarm_delta_phi_min2, rarm_delta_phi_max2 = theta_phi_absolute_difference_bounds(np.matrix(mpc_dat.rarm_q).T,
                                                                          mpc_dat.rarm_phi_curr,
                                                                          angle_constraint_threshold)

    larm_lb = np.maximum(larm_delta_phi_min, larm_delta_phi_min2)
    larm_ub = np.minimum(larm_delta_phi_max, larm_delta_phi_max2)
    rarm_lb = np.maximum(rarm_delta_phi_min, rarm_delta_phi_min2)
    rarm_ub = np.minimum(rarm_delta_phi_max, rarm_delta_phi_max2)

    lb = np.vstack((larm_lb,rarm_lb))
    ub = np.vstack((larm_ub,rarm_ub))
    
    return cost_quadratic_matrices, cost_linear_matrices, \
	   constraint_matrices, constraint_vectors, \
           constraint_matrices_eq, constraint_vectors_eq, lb, ub, current_larm_moment, current_rarm_moment, delta_larm_moment, delta_rarm_moment, \
	   delta_lforearm_x, delta_rforearm_x, delta_rarm_moment_x_force1, delta_rarm_moment_x_force2, mpc_dat.larm_force, mpc_dat.rarm_force, estimated_weight
	
## Set up and solve QP 
##
# In [3]: pp.QP?
# Docstring:
#     QP: constructor for Quadratic Problem assignment
#     1/2 x' H x  + f' x -> min
#     subjected to
#     A x <= b
#     Aeq x = beq
#     lb <= x <= ub
#
#     Examples of valid calls:
#     p = QP(H, f, <params as kwargs>)
#     p = QP(numpy.ones((3,3)), f=numpy.array([1,2,4]), <params as kwargs>)
#     p = QP(f=range(8)+15, H = numpy.diag(numpy.ones(8)), <params as kwargs>)
#     p = QP(H, f, A=A, Aeq=Aeq, b=b, beq=beq, lb=lb, ub=ub, <other params as kwargs>)
#     See also: /examples/qp_*.py
#
#     INPUT:
#     H: size n x n matrix, symmetric, positive-definite
#     f: vector of length n
#     lb, ub: vectors of length n, some coords may be +/- inf
#     A: size m1 x n matrix, subjected to A * x <= b
#     Aeq: size m2 x n matrix, subjected to Aeq * x = beq
#     b, beq: vectors of lengths m1, m2
#     Alternatively to A/Aeq you can use Awhole matrix as it's described 
#     in LP documentation (or both A, Aeq, Awhole)
def solve_qp(cost_quadratic_matrices, cost_linear_matrices,
 	     constraint_matrices, constraint_vectors, 
             constraint_matrices_eq, constraint_vectors_eq, lb, ub,
             debug_qp):

    total = np.zeros(cost_quadratic_matrices[0].shape)
    for cqm in cost_quadratic_matrices:
        total = total + cqm
    H = 2.0 * total
    # H is size m x m

    total = np.zeros(cost_linear_matrices[0].shape)
    for clm in cost_linear_matrices:
        total = total + clm
    f = total.T
    # f is size 1 x m
    A = np.concatenate(constraint_matrices)
    b = np.concatenate(constraint_vectors)
    Aeq = np.concatenate(constraint_matrices_eq)
    beq = np.concatenate(constraint_vectors_eq)
    # iprint: do text output each iprint-th iteration You can
    # use iprint = 0 for final output only or iprint < 0 to
    # omit whole output In future warnings are intended to be
    # shown if iprint >= -1.  
    if debug_qp: 
        iprint_val = 1
    else:
        iprint_val = -1

    # Result structure
    # http://openopt.org/OOFrameworkDoc
    # r = p.solve(nameOfSolver) 
    # >>> dir(r)
    # ['__doc__', '__module__', 'advanced', 'elapsed',
    # 'evals', 'ff', 'isFeasible', 'istop', 'iterValues',
    # 'msg', 'rf', 'solverInfo', 'stopcase', 'xf']
    #

    opt_error = False
    feasible = None
    delta_phi_zero = np.matrix(np.zeros(lb.shape))
    # this is Marc's fix for some cases in simulation when the arm
    # gets stuck and keeps setting delta_phi=0. Advait also saw
    # this problem in one case (Aug 23, 2011), and copied the next
    # line from the forked_simulation_files folder.
    #delta_phi_zero = np.matrix(np.random.normal(0.0, 0.003, lb.shape))
    #delta_phi_zero = np.matrix(np.random.normal(0.0, 0.01, lb.shape))

    try:
        qp = pp.QP(H, f, A=A, b=b, Aeq=Aeq, beq=beq, lb=lb, ub=ub)
        qp.solve('cvxopt_qp', iprint = iprint_val)
        delta_phi_opt = np.matrix(qp.xf).T
        val_opt = qp.ff
        feasible = qp.isFeasible

        if not feasible:
            print '====================================='
            print 'QP did not find a feasible solution.'
            print '====================================='
            opt_error = True
            delta_phi_opt = delta_phi_zero

        if np.isnan(delta_phi_opt.sum()):
            print '*****************************************************'
            print 'ERROR: QP FAILED TO FIND A SOLUTION AND RETURNED NaN(s)'
            print '*****************************************************'

        if qp.stopcase == 0:
            print 'maxIter, maxFuncEvals, maxTime or maxCPUTime have been exceeded, or the situation is unclear somehow else'
            delta_phi_opt = delta_phi_zero

        if qp.stopcase == -1:
            print 'solver failed to solve the problem'
            delta_phi_opt = delta_phi_zero

    except ValueError as inst:
        opt_error = True
        print type(inst)     # the exception instance
        print inst.args      # arguments stored in .args
        print inst           # __str__ allows args to printed directly
        print "ValueError raised by OpenOpt and/or CVXOPT"
        print "Setting new equilibrium angles to be same as old."
        print "delta_phi = 0"
        print "phi[t+1] = phi[t]"
        delta_phi_opt = delta_phi_zero
        val_opt = np.nan

    return delta_phi_opt, opt_error, feasible







