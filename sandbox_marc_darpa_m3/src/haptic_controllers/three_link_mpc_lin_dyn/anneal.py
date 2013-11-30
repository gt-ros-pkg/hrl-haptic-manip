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


# define objective function
def f(x):
    x1 = x[0]
    x2 = x[1]
    obj = 0.2 + x1**2 + x2**2 - 0.1*math.cos(6.0*3.1415*x1) - 0.1*math.cos(6.0*3.1415*x2)
    return obj

def gen_data_point(xc, search_mag, x_min, x_max):
    xi = []
    for k in xrange(len(xc)):
        # Generate new trial points using + or - 
        perturbation = (random.random()-0.5)*(x_max[k]-x_min[k])*search_mag
        xi.append(xc[k] + perturbation)
        # Clip to upper and lower bounds
        xi[k] = max(min(xi[k], x_max[k]), x_min[k])
    return xi

def run_annealing(x_start, x_min, x_max, num_cycles, num_trials_per_cycle, cost_calc, ssh_clients = None):
    ##################################################
    # Simulated Annealing
    ##################################################
    # Number of cycles
    n = num_cycles
    # Number of trials per cycle
    m = num_trials_per_cycle
    # Number of accepted solutions
    na = 0.0
    # Probability of accepting worse solution at the start
    p1 = 0.2
    # Probability of accepting worse solution at the end
    p50 = 0.00001
    # Initial temperature
    t_s = -1.0/math.log(p1)
    # Final temperature
    t_f = -1.0/math.log(p50)
    # Fractional reduction every cycle
    frac = (t_f/t_s)**(1.0/(n-1.0))
    search_mag = 1/2.5
    # Initialize x
    x = np.zeros((n+1,len(x_start)))
    x[0] = x_start
    xi = np.zeros(len(x_start))
    xi = x_start
    na = na + 1.0
    # Current best results so far
    xc = np.zeros(2)
    xc = x[0]
    fc = cost_calc(xi)
    fs = np.zeros(n+1)
    fs[0] = fc
    # Current temperature
    t = t_s
    # DeltaE Average
    DeltaE_avg = 0.0
    for i in range(n):
        print 'Cycle: ' + str(i) + ' with Temperature: ' + str(t)

        if ssh_clients != None:
            pass
            # #pseudo code:::::#####
            # make a deque with all numbers of xrange m
            # make a deque of clients that are running
            # while deque != []: #may need a false flag too so that don't end prematurely
            #     for client in ssh_clients:
            #         pop and assign to running
            #         start it running with needed data from deque
            #     for client in running:
            #         check each output to see if finished
            #         read cost if finished
            #         update cur_cost and xi accordingly


        for j in range(m):
            xi = gen_data_point(xc, search_mag, x_min, x_max)
            cur_cost = cost_calc(xi)
            DeltaE = abs(cur_cost-fc)
            if (cur_cost>fc):
                # Initialize DeltaE_avg if a worse solution was found
                #   on the first iteration
                if (i==0 and j==0): DeltaE_avg = DeltaE
                # objective function is worse
                # generate probability of acceptance
                p = math.exp(-DeltaE/(DeltaE_avg * t))
                # determine whether to accept worse point
                if (random.random()<p):
                    # accept the worse solution
                    accept = True
                else:
                    # don't accept the worse solution
                    accept = False
            else:
                # objective function is lower, automatically accept
                accept = True

            if (accept==True):
                # update currently accepted solution
                for kk in xrange(len(xc)):
                    xc[kk] = xi[kk]
                fc = cur_cost
                # increment number of accepted solutions
                na = na + 1.0
                # update DeltaE_avg
                DeltaE_avg = (DeltaE_avg * (na-1.0) +  DeltaE) / na

        # Record the best x values at the end of every cycle
        for mm in xrange(len(xc)):
            x[i+1][mm] = xc[mm]
        fs[i+1] = fc
        # Lower the temperature for next cycle
        t = frac * t
        #search_mag = frac*search_mag


    if False:
        plt.plot(x[:,0],x[:,1],'y-o')
        plt.savefig('contour.png')

        fig = plt.figure()
        ax1 = fig.add_subplot(211)
        ax1.plot(fs,'r.-')
        ax1.legend(['Objective'])
        ax2 = fig.add_subplot(212)
        ax2.plot(x[:,0],'b.-')
        ax2.plot(x[:,1],'g--')
        ax2.legend(['x1','x2'])

        # Save the figure as a PNG
        plt.savefig('iterations.png')

        plt.show()

    ut.save_pickle({'cost_history':fs, 
                    'x_min':x},
                    './simulated_annealing_results.pkl')


    return xc, fc, x 


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


    x_max = [350.,  #alpha
             350.,  #beta
             1.,  #zeta
             100.,   #mu
             -0.0075, #0.1,   #delta_t_impulse
             100.,   #u_max_nom
             100.,  #force_rate
             1.]    #was gamma now waypoint mag

    x_min = [1.0,
             1.0,
             0.001,
             0.1,
             -0.02, #0.01,
             0.01,
             0.1,
             0.015]


    # this first one was for when we included alpha
    #x_start = (np.random.uniform(0, 1, 8)*(np.array(x_max)-np.array(x_min))+np.array(x_min)).tolist()
    x_start = (np.random.uniform(0, 1, 8)*(np.array(x_max)-np.array(x_min))+np.array(x_min)).tolist()

    #print "x_start is :", x_start
    
    # x_start = [27.335196600721698,
    #            135.7817650480842,
    #            196.99897862794683,
    #            6.9238191532083233,
    #            0.039787524685139242,
    #            0.16717244319393917,
    #            0.014330607053289039,
    #            85.574475194413267]

    # x_start = [22.686868865756026,
    #            52.189019881037055,
    #            31.575358373453049,
    #            1.0665938114245734,
    #            0.207435596408724,
    #            0.26213406102351605,
    #            0.20514291074485913,
    #            0.68879122]

    num_cycles = 15
    num_trials_per_cycle = 15

    machines = ['colossus', 'colossus2']
    users = ['mkillpack', 'dpark']
    passwords = ['makeitso', 'pidi5252']

    # clients = deque()
    # for i in xrange(len(machines)):
    #     clients.append(paramiko.SSHClient())
    #     clients[i].set_missing_host_key_policy(paramiko.AutoAddPolicy())
    #     clients[i].connect(machines[i], username=users[i], password=passwords[i])

    #x_best, f_best, x_best_hist = run_annealing(x_start, x_min, x_max, num_cycles, num_trials_per_cycle, f, clients)
    #x_best, f_best, x_best_hist = run_annealing(x_start, x_min, x_max, num_cycles, num_trials_per_cycle, f)
    x_best, f_best, x_best_hist = run_annealing(x_start, x_min, x_max, num_cycles, num_trials_per_cycle, opt_function)

    print 'Best solution: ', x_best
    print 'Best objective: ', f_best

