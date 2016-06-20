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



###################### Optimization Module ######################
# This script provides an interface to optimization libraries.  Currently we only use convex optimization through open opt

# Functions: 
# cvxopt_optimization
#     1/2 x' H x  + f' x -> min
#     subjected to
#     A x <= b
#     lb <= x <= ub
#       - inputs: H (nxn matrix, symmetric, positive-definite), f (vector of length n), A (mxn matrix subjected to A * x <= b), b (vector of length m), lb (vector of length n), ub (vector of length n)
#       - outputs: optimization variable (vector of length n)
#################################################################



import numpy as np
import openopt

def cvxopt_optimization(H, f, A, b, lb, ub):
    opt_error = False
    feasible = None
    delta_phi_zero = np.matrix(np.zeros(lb.shape))
    try:
        #qp = openopt.QP(H, f, A=A, b=b, lb=lb, ub=ub)
        qp = openopt.QP(H, f, A=A, b=b)
        #qp = openopt.QP(H, f)
        qp.solve('cvxopt_qp')
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

    return delta_phi_opt

 
