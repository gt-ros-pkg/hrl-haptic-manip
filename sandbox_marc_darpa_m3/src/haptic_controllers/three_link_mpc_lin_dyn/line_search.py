## Generate a contour plot
# Import some other libraries that we'll need
# matplotlib and numpy packages must also be installed
import roslib
roslib.load_manifest('hrl_lib')
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import random
import math
import paramiko
from collections import deque
from tune_params_simulated_annealing import *
import time
import hrl_lib.util as ut


def gen_data_points(num_points, x_min, x_max):
    x = []

    min_arr = np.array(x_min)
    max_arr = np.array(x_max)
    for i in xrange(num_points+1):
        point = (max_arr-min_arr)*float(i)/num_points + min_arr
        x.append(point.tolist())
    return x

def run_line_search(x_start, x_min, x_max, cost_calc):
    #gen line search list of points then
    x_points = gen_data_points(20, x_min, x_max)

    for xi in x_points:
        cur_cost = cost_calc(xi)

    return None


if __name__ == "__main__":
    # x_start = [-8, -0.5]

    # x_min = [-10, -10]
    # x_max = [10, 10]

    #start gamma at  0.68879122,  0.99664054,  0.06036663 5.62187092,  1.22612564,  8.3765938 382.88642020,  908.25783382,  717.67807610
    #x_start   = [10., 100., 5., 0.1, 0.04, 0.2, 0.1]


    # these were pretty good values
    # x_0 = [275.19411283302003, #alpha
    #        229.43422905319315, #beta
    #        0.01, #41.49996594393528, #zeta
    #        34.843382055782946, #mu
    #        0.04, #0.04 was nominal for other trials, 0.025 for high force, high clutter 
    #        10.0, #max allowable change in u
    #        10.0, #max allowable change in force per time
    #        0.0] #gamma

    # x_max = [350.,  #alpha
    #          350.,  #beta
    #          10.,  #zeta
    #          70.,   #mu
    #          0.1,   #delta_t_impulse
    #          100.,   #u_max_nom
    #          100.,  #force_rate
    #          1.]    #was gamma now waypoint mag

    # x_min = [0.1,
    #          1.0,
    #          0.01,
    #          0.01,
    #          0.01,
    #          0.01,
    #          0.1,
    #          0.015]


    x_max = [275.19411283302003, #alpha
             229.43422905319315, #beta
             0.01, #41.49996594393528, #zeta
             34.843382055782946, #mu
             0.4, # was nominal for other trials, 0.025 for high force, high clutter 
             10.0, #max allowable change in u
             10.0, #max allowable change in force per time
             0.04] #waypoint size


    x_min = [275.19411283302003, #alpha
             229.43422905319315, #beta
             0.01, #41.49996594393528, #zeta
             34.843382055782946, #mu
             0.1, # was nominal for other trials, 0.025 for high force, high clutter 
             10.0, #max allowable change in u
             10.0, #max allowable change in force per time
             0.04] #waypoint

    x_start = x_min

    run_line_search(x_start, x_min, x_max, opt_function)

    print 'Best solution: ', x_best
    print 'Best objective: ', f_best

