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
#

# \authors: Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
# \adviser: Charles Kemp (Healthcare Robotics Lab, Georgia Tech.)


import itertools
import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('hrl_dynamic_mpc')

import rospy

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu
from matplotlib import pyplot as pl

from numpy import matrix, cos, arange, zeros, ones, asarray, zeros, mat, array, transpose, vstack, min, max
from numpy import dot, radians, degrees, size, shape, cov, log, linalg, random as rd
import numpy as np
import random
import math
import sys
import time 

######################################################################################
## Joint Limits ##
# create joint limit dicts
#        if arm == 'r':
#            max_lim = np.radians([ 120.00, 122.15, 77.5, 144., 122.,  45.,  45.])
#            min_lim = np.radians([ -47.61,  -20., -77.5,   0., -80., -45., -45.])
#        else:
#            max_lim = np.radians([ 120.00,   20.,  77.5, 144.,   80.,  45.,  45.])
#            min_lim = np.radians([ -47.61, -122.15, -77.5,   0., -122., -45., -45.])

## Velocity Limits ##
# 5 deg/s to 15 deg/s

# Optimization Criteria
# d-optimality criterion : -log(det(M)) : M = covariance matrix of regressor matrix

#####################################################################################

def get_init_condition(vl, vh, tf, wf, N):
    qdot_list = []
    t = arange(0,tf,0.1)
    M = size(t)
    A = zeros((M,2*N+1))
    for i in range(M):
        qdot_list.append(radians(random.randrange(vl,vh)))

    for i in range(M):
        j = 0
        k = 1
        while (j < 2*N):
            A[i][j] = (math.sin(wf*t[i]*k))/(wf*k)
            A[i][j+1] = (math.cos(wf*t[i]*k))/(wf*k)
            k=k+1
            j=j+2
        A[i][j] = 1   

    ans = dot((array(((matrix(A).T)*matrix(A)).I)*(matrix(A).T)),array(qdot_list))
    return ans.tolist()

def get_ref_traj_at_time(x, time, num_dof, N):
    wf = 0.1*2*math.pi
    q = []
    qd = []
    qdd = []

    for j in xrange(num_dof):
        q_buff = x[(j+1)*(2*N)]
        qd_buff = 0.
        qdd_buff = 0.
        for k in xrange(N):
            t = time
            a = x[j*(2*N+1)+2*k]
            b = x[j*(2*N+1)+2*k+1]
            q_buff = q_buff + a*(math.sin(wf*t*(k+1)))/(wf*(k+1)) - b*(math.cos(wf*t*(k+1)))/(wf*(k+1))
            qd_buff = qd_buff + a*(math.cos(wf*t*(k+1))) - b*(math.sin(wf*t*(k+1)))
            qdd_buff = qdd_buff + a*(wf*(k+1))*(math.sin(wf*t*(k+1))) + b*(wf*(k+1))*(math.cos(wf*t*(k+1)))
        q.append(q_buff)
        qd.append(qd_buff)
        qdd.append(qdd_buff)

    return q, qd, qdd



def get_ref_traj(x, t_total, rate, num_dof, N):
    wf = 0.1*2*math.pi

    num_samples = int(t_total/float(rate))
    q = []
    qd = []
    qdd = []

    # print "x is :", x
    # print "num_samples is :", num_samples
    for i in xrange(num_samples):
        q_cur = []
        qd_cur = []
        qdd_cur = []

        for j in xrange(num_dof):
            q_buff = x[(j+1)*(2*N)]
            # print "j is :", j
            # print "x[(j+1)*(2*N)]", x[(j+1)*(2*N)]
            qd_buff = 0.
            qdd_buff = 0.
            for k in xrange(N):
                t = i*rate
                a = x[j*(2*N+1)+2*k]
                b = x[j*(2*N+1)+2*k+1]
                if False:
                    print "t is :", t
                    print "a is :", a
                    print "b is :", b
                    print "wf is :", wf
                    print "k is :", k
                    print "(wf*t*(k+1))", (wf*t*(k+1))
                q_buff = q_buff + a*(math.sin(wf*t*(k+1)))/(wf*(k+1)) - b*(math.cos(wf*t*(k+1)))/(wf*(k+1))
                qd_buff = qd_buff + a*(math.cos(wf*t*(k+1))) - b*(math.sin(wf*t*(k+1)))
                qdd_buff = qdd_buff + a*(wf*(k+1))*(math.sin(wf*t*(k+1))) + b*(wf*(k+1))*(math.cos(wf*t*(k+1)))
                #raw_input()
            q_cur.append(q_buff)
            qd_cur.append(qd_buff)
            qdd_cur.append(qdd_buff)
        q.append(q_cur)
        qd.append(qd_cur)
        qdd.append(qdd_cur)

    # pp.figure()
    # pp.plot(q)
    # pp.show()
    #time.sleep(5)
    return q, qd, qdd


# def get_ref_traj(init_delta, t, num_dof):
#     wf = 0.1
#     N = 5
#     numJoints = num_dof
#     q_list = []
#     qdot_list = []
#     qddot_list = []
#     rows = numJoints
#     columns = 2*N + 1
#     temp_delta = zeros((rows, columns))
#     for row in range(rows):
#         for column in range(columns):
#             temp_delta[row][column] = init_delta[row*columns+column]
#     for row in range(rows):
#         delta = temp_delta[row].tolist()
#         q_element = 0.0
#         qdot_element = 0.0
#         qddot_element = 0.0
#         j = 0
#         k = 1
#         while (j < 2*N):
#             q_element = q_element + delta[j]*(math.sin(wf*t*k))/(wf*k) - delta[j+1]*(math.cos(wf*t*k))/(wf*k)
#             qdot_element = qdot_element + delta[j]*(math.cos(wf*t*k)) + delta[j+1]*(math.sin(wf*t*k)) 
#             qdfdot_element = qddot_element - delta[j]*(wf*k)*(math.sin(wf*t*k)) + delta[j+1]*(wf*k)*(math.cos(wf*t*k))
#             k=k+1
#             j=j+2   
#         q_list.append(q_element + delta[2*N])
#         qdot_list.append(qdot_element)
#         qddot_list.append(qddot_element)
#     return q_list, qdot_list, qddot_list

if __name__ == '__main__':
    
    print "hello. world."


