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

# \authors: Marc Killpack, Tapomayukh Bhattacharjee (Healthcare Robotics Lab, Georgia Tech.)
# \adviser: Charles Kemp (Healthcare Robotics Lab, Georgia Tech.)


import itertools
import matplotlib.pyplot as pp

import roslib
roslib.load_manifest('hrl_dynamic_mpc')

import rospy

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu
from matplotlib import pyplot as pl

from openopt import NLP
from numpy import matrix, cos, arange, zeros, ones, asarray, zeros, mat, array, transpose, vstack, min, max
from numpy import dot, radians, degrees, size, shape, cov, log10, linalg, random as rd
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

if __name__ == '__main__':
    
    darci = True
    kreacher = False
    arm = 'l'    

    if darci == True:
        from darci_dynamics import regressor
        
        if arm == 'r':
            max_lim = radians([200, 150, 85, 133, 195, 60, 60])
            min_lim = radians([-80, -25, -85, 0, -20, -60, -60])
        else:
            max_lim = radians([200, 25, 85, 133, 20, 60, 60])
            min_lim = radians([-80, -150, -85, 0, -195, -60, -60])

        robot = "darci"
        num_dof = 7

    elif kreacher == True:
        from kreacher_dynamics import regressor
        max_lim = radians([ 80., 80., 80., 80.])
        min_lim = radians([ -80., -80., -80., -80.])

        # max_lim = radians([10.]*4)
        # min_lim = radians([ -10]*4)

        robot = "kreacher"
        num_dof = 4
    else:
        print "need to define which robot"
        sys.exit()


    vl = radians(-45)
    vh = radians(45)
    tf = 10
    rate = 0.1

    wf = 0.1*(2*math.pi)
    N = 5
    time_ls = arange(0,tf,0.1)

    init_delta = []
    init_q = {}
    final_delta = []
    final_q = {}

    # Objective Function
    #f = lambda x: -log(linalg.det(cov(regressor(get_ref_traj(x,t, num_dof)[0], get_ref_traj(x,t, num_dof)[1], get_ref_traj(x,t, num_dof)[2]))))

    def f(x):
        #q, q_dot, qddot = get_ref_traj(x, t_total, rate, num_dof, N)
        q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
        for i in xrange(len(q)):
            if i == 0:
                regressor_mat = regressor(q[i], qd[i], qdd[i])
            else:
                regressor_mat = vstack((regressor_mat, regressor(q[i], qd[i], qdd[i])))

        # this is to get rid of columns that are linearly dependent causing the opt to fail
        row, col = regressor_mat.shape
        indices = []
        j = 1
        for i in xrange(col):
            if np.linalg.matrix_rank(regressor_mat[:, 0:i+1]) == j:
                indices.append(i)
                if j == 1:
                    cost_regressor = regressor_mat[:,i]
                else:
                    cost_regressor = np.hstack((cost_regressor, regressor_mat[:,i]))
                j = j + 1
        cost = -log10(linalg.det(cost_regressor.T*cost_regressor))
        #cost = linalg.cond(cost_regressor)
        
        # if np.isnan(cost):
        #     print "matrix is :", (cost_regressor.T*cost_regressor).A.tolist()
        #     print "det is :", linalg.det(cost_regressor.T*cost_regressor)
        #     U, s, V = linalg.svd(regressor_mat)
        #     print "singular values are :\n", s
        #     sys.exit()
        # elif linalg.det(cost_regressor.T*cost_regressor) == 0:
        #     print "matrix is :", (cost_regressor.T*cost_regressor).A.tolist()
        #     print "det is :", linalg.det(cost_regressor.T*cost_regressor)
        #     U, s, V = linalg.svd(regressor_mat)
        #     print "singular values are :\n", s
        #     cost = 100000000.
        #     #sys.exit()
        return cost
                      

        # c(x) <= 0 constraints

        # c = []
        # for i in xrange(num_dof):
        #     c.append(lambda x: get_ref_traj(x,t,num_dof)[0][i] - max_lim[i])
        #     c.append(lambda x: min_lim[i] - get_ref_traj(x,t,num_dof)[0][i])

        # for i in xrange(num_dof):
        #     c.append(lambda x: get_ref_traj(x,t,num_dof)[1][i] - radians(vh))
        #     c.append(lambda x: radians(vl) - get_ref_traj(x,t,num_dof)[1][i])


    def make_q_min_func(minimum, jt):
        def q_min_func(x):
            q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
            q_array = array(q)
            q_min = min(q_array[:,jt])
            return minimum - q_min
        return q_min_func

    def make_q_max_func(maximum, jt):
        def q_max_func(x):
            q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
            q_array = array(q)
            q_max = max(q_array[:,jt])
            return q_max - maximum
        return q_max_func
    
    def make_qd_min_func(minimum, jt):
        def qd_min_func(x):
            q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
            qd_array = array(qd)
            qd_min = min(qd_array[:,jt])
            return minimum - qd_min
        return qd_min_func

    def make_qd_max_func(maximum, jt):
        def qd_max_func(x):
            q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
            qd_array = array(qd)
            qd_max = max(qd_array[:,jt])
            return qd_max - maximum
        return qd_max_func

    def make_qdd_min_func(minimum, jt):
        def qdd_min_func(x):
            q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
            qdd_array = array(qdd)
            qdd_min = min(qdd_array[:,jt])
            return minimum - qdd_min
        return qdd_min_func

    def make_qdd_max_func(maximum, jt):
        def qdd_max_func(x):
            q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
            qdd_array = array(qdd)
            qdd_max = max(qdd_array[:,jt])
            return qdd_max - maximum
        return qdd_max_func

    def make_q_zero_func(jt):
        def q_zero(x):
            q, qd, qdd = get_ref_traj_at_time(x, 0, num_dof, N)            
            return q[jt]
        return q_zero

    def make_qd_zero_func(jt):
        def qd_zero(x):
            q, qd, qdd = get_ref_traj_at_time(x, 0, num_dof, N)            
            return qd[jt]
        return qd_zero

    def make_qdd_zero_func(jt):
        def qdd_zero(x):
            q, qd, qdd = get_ref_traj_at_time(x, 0, num_dof, N)            
            return qdd[jt]
        return qdd_zero

    # def make_last_joint_zero_func(jt):
    #     def last_jt_zero(x):
    #         q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
    #         q_array = array(qdd)
    #         q_max = float(np.max(np.abs(q_array[:,jt])))
    #         return q_max
    #     return last_jt_zero

    h = []
    for i in xrange(num_dof):
        h.append(make_q_zero_func(i))
        h.append(make_qd_zero_func(i))
        h.append(make_qdd_zero_func(i))

    c = []
    for i in xrange(num_dof):
        c.append(make_q_min_func(min_lim[i], i))
        c.append(make_q_max_func(max_lim[i], i))
        c.append(make_qd_min_func(vl, i))
        c.append(make_qd_max_func(vh, i))
        c.append(make_qdd_min_func(-2.5, i))
        c.append(make_qdd_max_func(2.5, i))
        


    if darci == True:
        import darci_arm_kinematics as dak

        robot = dak.DarciArmKinematics(arm)
        if arm == 'r':
            def fk_limits(x):
                q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
                max_y = -1.0 #robot.FK([0]+[1.54]+[0]*5, 7)[0][1][0,0]
                for i in xrange(len(q)):
                    for j in [3, 4, 5, 6]:
                        pos = robot.FK(q[i], j)[0][1][0,0]
                        #print "pos is ", pos
                        if pos > max_y:
                            max_y = pos
                return (max_y + 0.21)

        elif arm == 'l':
            def fk_limits(x):
                q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
                min_y = 1.0 #robot.FK([0]+[1.54]+[0]*5, 7)[0][1][0,0]
                #print "max_y starts at :", max_y
                for i in xrange(len(q)):
                    for j in [3, 4, 5, 6]:
                        pos = robot.FK(q[i], j)[0][1][0,0]
                        #print "pos is ", pos
                        if pos < min_y:
                            min_y = pos
                return (0.21 - min_y )

        else:
            print 'which arm are you trying to use if not right or left?'
            assert(False)
                    

    elif kreacher == True:
        print "tf is :", tf
        print "rate is :", rate

        def fk_limits(x):
            link = 0.1349
            q, qd, qdd = get_ref_traj(x, tf, rate, num_dof, N)
            min_x = (num_dof-1)*link+0.165
            for i in xrange(len(q)):
                pos_0 = link*math.cos(q[i][0])
                pos_1 = pos_0 + link*math.cos(q[i][1]+q[i][0])
                pos_2 = pos_1 + link*math.cos(q[i][2]+q[i][1]+q[i][0])
                pos_3 = pos_2 + 0.165*math.cos(q[i][3]+q[i][2]+q[i][1]+q[i][0])

                if pos_1 < min_x:
                    min_x = pos_1
                if pos_2 < min_x:
                    min_x = pos_2
                if pos_3 < min_x:
                    min_x = pos_3
                return -(min_x-0.13)  #only feasible up to 0.2698, we'll see what happens

    c.append(fk_limits)

        # h(x) = 0 constraints

        # required constraints tolerance, default for NLP is 1e-6
    contol = 1e-7

        # If you use solver algencan, NB! - it ignores xtol and ftol; using maxTime, maxCPUTime, maxIter, maxFunEvals, fEnough is recommended.
        # Note that in algencan gtol means norm of projected gradient of  the Augmented Lagrangian
        # so it should be something like 1e-3...1e-5
    gtol = 1e-6 # (default gtol = 1e-6)

        # Assign problem:
         # 1st arg - objective function
        # 2nd arg - start point
        #p = NLP(f, x0, c=c,  gtol=gtol, contol=contol, iprint = 50, maxIter = 10000, maxFunEvals = 1e7, name = 'NLP_1')
    x0 = rd.uniform(-0.8, 0.8, num_dof*(2*N+1))
    # x0[2*N] = 0
    # x0[2*(2*N)] = 0
    # x0[3*(2*N)] = 0
    #x0 = rd.uniform(-0.1, 0.1, 2*N+1).tolist() + rd.uniform(-0.01, 0.01, 2*N+1).tolist() + rd.uniform(-.001, 0.001, 2*N+1).tolist()

    #x0 = range(0, num_dof*(2*N+1))
    #p = NLP(f, x0, c=c, h=h, gtol=gtol, contol=contol, iprint = 1, maxIter = 1000, maxFunEvals = 1e7, name = 'NLP_1')
    p = NLP(f, x0, c=c, h=h, gtol=gtol, contol=contol, iprint = 1, maxIter = 700, maxFunEvals = 1e7, name = 'NLP_1')

    #p = NLP(f, x0,  gtol=gtol, contol=contol, iprint = 50, maxIter = 10000, maxFunEvals = 1e7, name = 'NLP_1')

        #optional: graphic output, requires pylab (matplotlib)
    p.plot = True

    solver = 'ralg'
    #solver = 'scipy_cobyla'
        #solver = 'algencan'
        #solver = 'ipopt'
        #solver = 'scipy_slsqp'

        # solve the problem

    r = p.solve(solver, plot=0) # string argument is solver name


        # r.xf and r.ff are optim point and optim objFun value
        # r.ff should be something like 132.05
        #print r.xf, r.ff
        
    print r.xf

    q, qd, qdd = get_ref_traj(r.xf, tf, rate, num_dof, N)

    pl.figure()
    pl.plot(q)
    pl.show()

    results = {}
    results['params'] = r.xf
    results['q'] = q
    results['qd'] = qd
    results['qdd'] = qdd
    ut.save_pickle(results, './darci_sys_id.pkl')

    # final_delta = r.xf
        
    #     print "########### Final theta values at time", t, "###############"
    #     final_q[t] = degrees(get_ref_traj(final_delta,t, num_dof)[0])
    #     print final_q[t]
    #     ut.save_pickle(final_q, robot+'_JEPs.pkl')

    #     if t == time_ls[size(time_ls)-1]:
    #         print "########### Final delta values at time", t, "###############"
    #         print final_delta




