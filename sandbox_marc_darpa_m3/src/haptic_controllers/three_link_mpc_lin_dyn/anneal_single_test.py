#!/usr/bin/env python

## Generate a contour plot
# Import some other libraries that we'll need
# matplotlib and numpy packages must also be installed
import roslib
roslib.load_manifest('hrl_lib')
import numpy as np
import random
import math
import time
import hrl_lib.util as ut
import socket
import getpass
import os

# define objective function
def f(x):
    x1 = x[0]
    x2 = x[1]
    obj = 0.2 + x1**2 + x2**2 - 0.1*math.cos(6.0*3.1415*x1) - 0.1*math.cos(6.0*3.1415*x2)
    return obj

if __name__ == "__main__":
    username = getpass.getuser()
    print "turning now ..."

    while True:
        #path = 
        path_read = '/home/'+username+'/hrl_file_server/darpa_m3/sim_anneal_tuning/x_config_'+socket.gethostname()+'.pkl'
        path_write = '/home/'+username+'/hrl_file_server/darpa_m3/sim_anneal_tuning/cost_'+socket.gethostname()+'.pkl'
        path_lock = '/home/'+username+'/hrl_file_server/darpa_m3/sim_anneal_tuning/cost_'+socket.gethostname()+'_running.pkl'
        #hrl_file_server/darpa_m3/sim_anneal_tuning/x_config_'+socket.gethostname()+'.pkl')
        if os.path.isfile(path_read):
            print 'got a file'
            ut.save_pickle({}, path_lock)
            x_config = ut.load_pickle(path_read)
            os.remove(path_read)
            print x_config
            x_start = x_config['x_start']
            cost = f(x_start)
            ut.save_pickle({'cost':cost}, path_write)
            os.remove(path_lock)
        time.sleep(1.0)
