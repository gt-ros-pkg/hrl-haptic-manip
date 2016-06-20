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



######################## Waypoint Module ########################
# This script provides the low level control interface for the SPR2YZTS (two prismatic robots with y/z translation positioned to be placed on the thigh and shin).  If a different robot is used, then a similar script to this should be written.

# Functions: 
# generate_linear_trajectory
# 	- inputs: current states (vector containing current trajectory states), goal states (vector containing goal trajectory states), maximum step size (double) 
#	- outputs: trajectory (matrix)
# generate_holding_trajectory
# 	- inputs: current states (vector containing current trajectory states), goal states (vector containing goal trajectory states), holding duration (double), frequency (double)
# check_waypoint
# 	- inputs: current states (vector containing current trajectory states), goal states (vector containing goal trajectory states), current waypoint indicie (int), waypoint trajectory (matrix), deadzone (bool), deadzone indicie (int), at goal distance threshold (double), at waypoint distance threshold (double) 
# 	- outputs: incremented current waypoint indicie (int), deadzone (bool), incremented deadzone indicie (int) 
#################################################################



import numpy as np
import math

### Waypoint Module ###

def generate_linear_trajectory(current_states, goal_states, max_step_size):
    # Determine waypoints from goal position, current position, and step size

    # Determine the number of waypoints from the maximum number of waypoints between the various n states. 
    # This makes sure that the maximum step size is the step size limit. 
    # Use the ceiling function because this will make sure that the max step size requirement is met.
    num_of_waypoints_array = []
    for i in range(len(goal_states)):
        num_of_waypoints_array.append(int(math.ceil(abs((goal_states[i]-current_states[i])/max_step_size))))
    num_of_waypoints = max(num_of_waypoints_array)
    waypoint_trajectory = np.matrix(current_states).T

    # step sizes is a 1xn array, where n is the number of states (robot positions)
    step_sizes = abs((np.matrix(goal_states).T-np.matrix(current_states).T)/num_of_waypoints)

    # loop to add in all of the waypoints (using the step sizes array)
    for i in range(1,num_of_waypoints):
        waypoint_trajectory = np.hstack((waypoint_trajectory,(waypoint_trajectory[:,i-1]+np.multiply(np.copysign(np.ones((1,len(goal_states))),np.matrix(goal_states)-np.matrix(current_states)).T,step_sizes))))
    waypoint_trajectory = np.hstack((waypoint_trajectory,np.matrix(goal_states).T))
    return waypoint_trajectory

def generate_holding_trajectory(current_states, goal_states, holding_duration, frequency):
    # generate a waypoint trajectory that holds the goal position for a certain amount of time
    num_of_waypoints = int(holding_duration*frequency)
    waypoint_trajectory = np.matrix([current_states]).T
    for i in range(1,num_of_waypoints+1):
        waypoint_trajectory = np.hstack((waypoint_trajectory,np.matrix(goal_states).T))
    return waypoint_trajectory

def check_waypoint(current_states, goal_states, current_waypoint_ind, waypoint_trajectory, deadzone, deadzone_ind, at_goal_dist_threshold, at_waypoint_dist_threshold):
    # check to see if the next waypoint has been reached

    # if the current state is close to the goal state, then we are in the deadzone
    if np.all(abs(np.matrix(current_states)-waypoint_trajectory[:,-1].T)<np.ones((1,len(current_states)))*at_goal_dist_threshold):
        deadzone = True
        #deadzone_ind += 1
    # if the current state is at the next waypoint, then increment the current waypoint index
    elif np.all(waypoint_trajectory[:,current_waypoint_ind].T!=np.matrix(goal_states)) and np.all(abs(np.matrix(current_states)-waypoint_trajectory[:,current_waypoint_ind].T)<np.ones((1,len(current_states)))*at_waypoint_dist_threshold):
        current_waypoint_ind += 1
    if deadzone == True:
        deadzone_ind += 1
    return current_waypoint_ind, deadzone, deadzone_ind


