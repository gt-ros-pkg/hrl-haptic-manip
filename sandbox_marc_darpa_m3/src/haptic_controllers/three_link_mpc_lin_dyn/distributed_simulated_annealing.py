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
#from tune_params_simulated_annealing import *
import hrl_lib.util as ut
import socket
import getpass
import os
import time

class SimAnneal():
    def __init__(self):
        self.x_start   = [10., 100., 5., 0.1, 0.04, 0.5]
        self.x_max = [1000., 1000., 1000., 10., 3., 2.]
        self.x_min = [0.01, 0.01, 0.01, 0.001, 0.001, 0.001]


        self.x_start = [3, -8]

        self.x_min = [-10, -10]
        self.x_max = [10, 10]

        self.num_cycles = 10
        self.num_trials_per_cycle = 12

        self.machines = ['colossus2', 'colossus3', 'colossus5']
        self.clients_not_running = ['colossus2', 'colossus3', 'colossus5']
        self.run_batch = True
        #self.users = ['dpark', 'dpark']
        #self.passwords = ['pidi5252', 'pidi5252']
        self.opt_function = self.f #opt_function
        self.DeltaE_avg = None
        #self.ssh_clients = None
        #self.ssh_clients = [] #deque()
        # for i in xrange(len(self.machines)):
        #     self.ssh_clients.append(paramiko.SSHClient())
        #     self.ssh_clients[i].set_missing_host_key_policy(paramiko.AutoAddPolicy())
        #     self.ssh_clients[i].connect(self.machines[i], username=self.users[i], password=self.passwords[i])
        #     _, stdout, stderr = self.ssh_clients[i].exec_command('source ~/.bashrc')  
        #     print stderr.readlines()
        #     _, stdout, stderr = self.ssh_clients[i].exec_command('roscd')  
        #     print stderr.readlines()
        #     print stdout.readlines()
        #     assert(False)

    #x_best, f_best, x_best_hist = run_annealing(x_start, x_min, x_max, num_cycles, num_trials_per_cycle, f, clients)
    #x_best, f_best, x_best_hist = run_annealing(x_start, x_min, x_max, num_cycles, num_trials_per_cycle, f)

    def run_opt(self):
        self.x_best, self.f_best, self.x_best_hist = self.run_annealing()

        print 'Best solution: ', self.x_best
        print 'Best objective: ', self.f_best


    # define objective function
    def f(self, x):
        print "x is :", x
        x1 = x[0]
        x2 = x[1]
        obj = 0.2 + x1**2 + x2**2 - 0.1*math.cos(6.0*3.1415*x1) - 0.1*math.cos(6.0*3.1415*x2)
        return obj

    def gen_data_point(self):
        #TODO get new code using frac from other anneal.py function
        self.xi = []
        for k in xrange(len(self.xc)):
            # Generate new trial points
            perturbation = (random.random()-0.5)*(self.x_max[k] - self.x_min[k])*self.search_mag
            self.xi.append(self.xc[k] + perturbation)

            # Clip to upper and lower bounds
            self.xi[k] = max(min(self.xi[k], self.x_max[k]), self.x_min[k])

    def run_annealing(self):
        ##################################################
        # Simulated Annealing
        ##################################################
        # Number of cycles
        n = self.num_cycles
        # Number of trials per cycle
        m = self.num_trials_per_cycle
        # Number of accepted solutions
        self.na = 0.0
        # Probability of accepting worse solution at the start
        p1 = 0.8
        # Probability of accepting worse solution at the end
        p50 = 0.00001
        # Initial temperature
        t_s = -1.0/math.log(p1)
        # Final temperature
        t_f = -1.0/math.log(p50)
        # Fractional reduction every cycle
        frac = (t_f/t_s)**(1.0/(n-1.0))
        self.search_mag = 1./10. #this is a fraction of the total variable range
        # Initialize x
        self.x = np.zeros((n+1,len(self.x_start)))
        self.x[0] = self.x_start
        #xi = np.zeros(len(self.x_start))
        self.xi = self.x_start
        self.na = self.na + 1.0
        # Current best results so far
        self.xc = np.zeros(2)
        self.xc = self.x[0]
        self.fc = self.opt_function(self.xi)
        self.fs = np.zeros(n+1)
        self.fs[0] = self.fc
        # Current temperature
        self.t = t_s
        # DeltaE Average
        clients_running = [] #deque() I wanted to use deque, but can only pop from ends
        std_out_ls = [] #deque()
        #pid_ls = [] #deque()

        for i in range(n):
            print 'Cycle: ' + str(i) + ' with Temperature: ' + str(self.t)

            #if self.ssh_clients != None:
            if self.run_batch == True:
                # #pseudo code:::::#####
                trials = range(self.num_trials_per_cycle)
                trials_done = False
                while trials_done == False:
                    #for client in self.ssh_clients:
                    #for jj in xrange(len(self.clients_not_running):
                    stop = False
                    while stop == False:
                        if self.clients_not_running == []:
                            stop = True
                        elif trials == []:
                            stop = True
                        else:
                            #print "machine :", self.clients_not_running[-1]
                            # print "trials is :\n", trials
                            # print "machine is :\n", machine
                            #clients_running.append(self.ssh_clients.pop())
                            trials.pop()
                            machine = self.clients_not_running.pop()
                            clients_running.append(machine)
                            # print "clients_running is now :", clients_running
                            #_, stdout, _ = clients_running[-1].exec_command('uname -n')
                            self.gen_data_point() 
                            write_path = '/home/'+getpass.getuser()+'/hrl_file_server/darpa_m3/sim_anneal_tuning/x_config_'+machine+'.pkl'
                            #write_path = '/home/'+getpass.getuser()+'/Desktop/x_config_'+machine+'.pkl'
                            ut.save_pickle({'x_start':self.xi}, write_path)
                            #command = 'echo $$; exec ' + 'python ~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/anneal_single_test.py'
                            #command = 'exec python ~/git/hrl_haptic_manipulation_in_clutter/sandbox_marc_darpa_m3/src/haptic_controllers/three_link_mpc_lin_dyn/anneal_single_test.py'
                            #_, stdout, stderr = clients_running[-1].exec_command(command)  
                            #print "error is :", stderr.readlines()
                        #TODO update tunable... file to read from hrl_file_server and write cost to hrl
                            #pid_ls.append(int(stdout.readline()))
                            #std_out_ls.append(stdout)

                    #for i in xrange(len(clients_running)):
                    for machine in clients_running:
                        #ii in xrange(len(clients_running)):
                        #print "clients running is :", clients_running
                        path_lock = '/home/'+getpass.getuser()+'/hrl_file_server/darpa_m3/sim_anneal_tuning/cost_'+machine+'_running.pkl'
                        path_read = '/home/'+getpass.getuser()+'/hrl_file_server/darpa_m3/sim_anneal_tuning/cost_'+machine+'.pkl'
                        #'/home/'+username+'/hrl_file_server/darpa_m3/sim_anneal_tuning/x_config_'+socket.gethostname()+'.pkl') 

                        if (os.path.isfile(path_lock) == False) and os.path.isfile(path_read):
                            result = ut.load_pickle(path_read)
                            self.cur_cost = result['cost']
                            self.update()
                            os.remove(path_read)
                            #print "clients_not_running before :", self.clients_not_running
                            self.clients_not_running.append(machine)
                            #print "clients_not_running after :", self.clients_not_running
                            #print "clients_running before :", clients_running
                            clients_running.pop(clients_running.index(machine))
                            #print "clients_running after :", clients_running
                        #text_buf = std_out_ls[i].readlines()
                        #print "text_buf for ", i, " is :", text_buf
                        # for text in text_buf:
                        #     if "FINISHED SIM ANNEALING TRIAL" in text: #TODO add this to where function is called
                        #         #_,_,_ =clients_running[i].exec_command('kill '+pid_ls[i])
                        #         _, stdout, _ = clients_running[i].exec_command('uname -n')
                        #         machine_name = stdout.readline()[0:-1]
                        #         self.ssh_clients.append(clients_running.pop(i))
                        #         #pid_ls.pop(i)
                        #         std_out_ls.pop(i)
                        #         result = ut.load_pickle('/home/'+getpass.getuser()+'/hrl_file_server/darpa_m3/sim_anneal_tuning/cost_'+machine_name+'.pkl')
                        #         self.cur_cost = result['cost']
                        #         self.update()
                    if trials == [] and clients_running == []:
                        trials_done = True
                    time.sleep(0.1)
            else:
                for j in range(self.num_trials_per_cycle):
                    self.gen_data_point()
                    self.cur_cost = self.opt_function(self.xi)
                    self.update()

            # Record the best x values at the end of every cycle
            for mm in xrange(len(self.xc)):
                self.x[i+1][mm] = self.xc[mm]
            self.fs[i+1] = self.fc
            # Lower the temperature for next cycle
            self.t = frac * self.t

        if True:
            plt.plot(self.x[:,0],self.x[:,1],'y-o')
            plt.savefig('contour.png')

            fig = plt.figure()
            ax1 = fig.add_subplot(211)
            ax1.plot(self.fs,'r.-')
            ax1.legend(['Objective'])
            ax2 = fig.add_subplot(212)
            ax2.plot(self.x[:,0],'b.-')
            ax2.plot(self.x[:,1],'g--')
            ax2.legend(['x1','x2'])

            # Save the figure as a PNG
            plt.savefig('iterations.png')

            plt.show()

        return self.xc, self.fc, self.x, self.fs 

    def update(self):
        self.DeltaE = abs(self.cur_cost-self.fc)
        if (self.cur_cost>self.fc):
            # Initialize self.DeltaE_avg if a worse solution was found
            #   on the first iteration
            if self.DeltaE_avg == None: 
                self.DeltaE_avg = self.DeltaE
            # objective function is worse
            # generate probability of acceptance
            p = math.exp(-self.DeltaE/(self.DeltaE_avg * self.t))
            # determine whether to accept worse point
            if (random.random()<p):
                # accept the worse solution
                accept = True
            else:
                # don't accept the worse solution
                accept = False
        else:
            if self.DeltaE_avg == None: 
                self.DeltaE_avg = self.DeltaE
            # objective function is lower, automatically accept
            accept = True

        if (accept==True):
            # update currently accepted solution
            for kk in xrange(len(self.xc)):
                self.xc[kk] = self.xi[kk]
                self.fc = self.cur_cost
                # increment number of accepted solutions
                self.na = self.na + 1.0
                # update self.DeltaE_avg
                self.DeltaE_avg = (self.DeltaE_avg * (self.na-1.0) +  self.DeltaE) / self.na

if __name__ == "__main__":

    optimizer = SimAnneal()
    x_cur, f_cur, x_hist, f_hist = optimizer.run_annealing()

    print "x_cur is :", x_cur
    print "f_cur is :", f_cur

    # x_start = [0.8, -0.5]

    # x_min = [-100, -100]
    # x_max = [100, 100]


