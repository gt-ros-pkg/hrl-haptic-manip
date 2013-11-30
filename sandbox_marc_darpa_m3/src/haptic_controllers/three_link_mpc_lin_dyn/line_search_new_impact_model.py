import roslib
roslib.load_manifest('hrl_lib')
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
import random
import math
from collections import deque
from tune_params_line_search import *
import time
import hrl_lib.util as ut


def gen_data_points(num_points, x_min, x_max, scale='linear'):
    x = []

    min_arr = np.array(x_min)
    max_arr = np.array(x_max)

    if scale == 'linear':
        for i in xrange(num_points+1):
            point = (max_arr-min_arr)*float(i)/num_points + min_arr
            x.append(point.tolist())
        return x
    else:
        new_max = np.log10(max_arr)
        new_min = np.log10(min_arr)
            
        for i in xrange(num_points+1):
            point = 10**((new_max-new_min)*float(i)/num_points + new_min)
            x.append(point.tolist())
        return x


def run_line_search(x_min, x_max, cost_calc, scale='linear'):
    #gen line search list of points then
    x_points = gen_data_points(25, x_min, x_max, scale)

    for xi in x_points:
        cur_cost = cost_calc(xi)

    return None


if __name__ == "__main__":


    # x_max = [350.,  #alpha
    #          350.,  #beta
    #          1.,  #zeta
    #          100.,   #mu
    #          -0.0075, #0.1,   #delta_t_impulse
    #          100.,   #u_max_nom
    #          100.,  #force_rate
    #          1.]    #was gamma now waypoint mag

    # x_min = [1.0,
    #          1.0,
    #          0.001,
    #          0.1,
    #          -0.02, #0.01,
    #          0.01,
    #          0.1,
    #          0.015]


    x_max = [239., #alpha  linear
             255., #beta log 
             0.743, #zeta log - and move max force down to 1-5?
             15.0, #mu - linear (0.1 - 80)
             0.825, #t_impulse_offset in line function - linear
             94.0, #max allowable change in u - log
             35.0, #max allowable change in force per time
             0.015] #waypoint size - log 

    x_min = [239., #alpha
             255., #beta
             0.743, #zeta
             15.0, #mu
             0.825, #t_impulse_offset in line function
             94.0, #max allowable change in u
             35.0, #max allowable change in force per time
             0.015] #waypoint size

    scale = 'linear'

    run_line_search(x_min, x_max, opt_function, scale)

    print 'Best solution: ', x_best
    print 'Best objective: ', f_best

