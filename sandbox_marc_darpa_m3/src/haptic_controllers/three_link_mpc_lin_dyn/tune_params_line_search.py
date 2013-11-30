import roslib
roslib.load_manifest('sandbox_marc_darpa_m3')
import scipy.optimize as optimize
import subprocess
import time
import hrl_lib.util as ut
import numpy as np
import os

roscore = None
simulator = None
cvxgen_node = None
cost_history = []
x_history = []
max_force_cost_hist = [] 
force_cost_hist = []
result_cost_hist = []
time_cost_hist = []
not_converged_hist = []


def run_trial(file_name, f_thresh=None):
    roscore = subprocess.Popen(['roscore'])

    time.sleep(2.0)

    simulator = subprocess.Popen(['roslaunch',
                                  'hrl_software_simulation_darpa_m3',
                                  'gen_simulator.launch'])
    time.sleep(2.0)

    load_params = subprocess.Popen(['rosrun', 
                                    'hrl_common_code_darpa_m3', 
                                    'obstacles.py', 
                                    '--pkl='+file_name])

    # load_params = subprocess.call(['rosparam', 
    #                                'load', 
    #                                file_name])

    cvxgen_node = subprocess.Popen(['rosrun', 
                                    'sandbox_marc_darpa_m3',
                                    'three_link_mpc_lin_dyn'])


    if f_thresh == None:
        controller = subprocess.call(['python', 
                                      'basic_test_mpc_with_dynamics.py',
                                      '--tuning_params'])
    else:
        controller = subprocess.call(['python', 
                                      'basic_test_mpc_with_dynamics.py',
                                      '--tuning_params',
                                      '--f_thresh='+str(f_thresh)])
                                      #'--impulse_slope'])

    roscore.terminate()
    simulator.terminate()
    cvxgen_node.terminate()
    time.sleep(10.)


def get_cost(f_thresh):
    d_log = ut.load_pickle('./reach_log.pkl')
    d_result = ut.load_pickle('./overall_result.pkl')
    d_converged = ut.load_pickle('./converged_log.pkl')

    forces = np.array(d_log['all_forces_list'])
    times = np.array(d_log['time_stamp_list'])
    result = d_result['reached_goal']

    if forces != []:
        force_cost = np.sum(forces[np.where(forces>=f_thresh)[0]])*0.1  # was 10
        #force_cost = (len(np.where(forces>=f_thresh)[0])/float(len(forces)))*1000 #turn into a percent then scale

        if np.max(forces) >= f_thresh:
            max_force_cost = (np.max(forces) - f_thresh)*100.
        else:
            max_force_cost = 0.
    else:
        max_force_cost = 0.
        force_cost = 0.
        
    print "max force cost is :", max_force_cost

    print "force cost is :", force_cost

    time_cost = (times[-1]-times[0])*10.

    print "time cost is :", time_cost
        
    if result == True:
        result_cost = 0.
    else:
        result_cost = 25000.
        
    print "result cost is :", result_cost
    
    cost = max_force_cost + result_cost + force_cost + time_cost

    return cost, max_force_cost, force_cost, result_cost, time_cost, d_converged['not_converged']
    

def opt_function(x):
    global cost_history, x_history, max_force_cost_hist, force_cost_hist, result_cost_hist, time_cost_hist, not_converged_hist

    total_cost = 0.0
    total_force_cost = 0.0
    total_max_force_cost = 0.0
    total_time_cost = 0.0
    total_result_cost = 0.0
    total_not_converged = 0.0
    #base = '/home/mkillpack'
    base = '/home/mkillpack'

    #base2 = '/home/mkillpack/hrl_file_server/darpa_m3/sim_anneal_tuning/training_cases/'
    base2 = '/home/mkillpack/hrl_file_server/darpa_m3/sim_anneal_tuning/line_search_cases/'
    f_buf = os.listdir(base2)
    f_list = [base2+folder for folder in f_buf]

    # f_list = [base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test1.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test2.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test3.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test4.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test5.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test6.yaml']

    #f_list = ['/home/mkillpack/Desktop/interm_test.yaml', '/home/mkillpack/Desktop/new_test3.yaml']

    offset = x[4]

    f_thresh = 5.0
    x[4] = -0.5/20.*f_thresh + offset
    ut.save_pickle({'x':x}, './x.pkl')

    for f_name in f_list:
        run_trial(f_name, f_thresh)
        time.sleep(5)
        cost_buf, max_force_cost_buf, force_cost_buf, result_cost_buf, time_cost_buf, num_not_converged= get_cost(f_thresh)
        total_cost = total_cost + cost_buf
        total_force_cost  = total_force_cost + force_cost_buf
        total_max_force_cost  = total_max_force_cost + max_force_cost_buf
        total_time_cost = total_time_cost + time_cost_buf
        total_result_cost = total_result_cost + result_cost_buf
        total_not_converged = num_not_converged

    f_thresh = 15.0
    x[4] = -0.5/20.*f_thresh + offset
    ut.save_pickle({'x':x}, './x.pkl')
    for f_name in f_list:
        run_trial(f_name, f_thresh)
        time.sleep(5)
        cost_buf, max_force_cost_buf, force_cost_buf, result_cost_buf, time_cost_buf, num_not_converged= get_cost(f_thresh)
        total_cost = total_cost + cost_buf
        total_force_cost  = total_force_cost + force_cost_buf
        total_max_force_cost  = total_max_force_cost + max_force_cost_buf
        total_time_cost = total_time_cost + time_cost_buf
        total_result_cost = total_result_cost + result_cost_buf
        total_not_converged = num_not_converged

    cost_history.append(total_cost)
    x_history.append(x)
    max_force_cost_hist.append(total_max_force_cost)
    force_cost_hist.append(total_force_cost)
    result_cost_hist.append(total_result_cost)
    time_cost_hist.append(total_time_cost)
    not_converged_hist.append(total_not_converged)
    
    print "for x: \n", x
    print "cost is currently :", total_cost

    ut.save_pickle({'cost_history':cost_history,
                    'x_history':x_history,
                    'force_cost': force_cost_hist,
                    'max_force_cost': max_force_cost_hist,
                    'time_cost': time_cost_hist,
                    'result_cost': result_cost_hist,
                    'not_converged': not_converged_hist},
                    './simulated_annealing_interm_results.pkl')


    return total_cost

if __name__ == '__main__':
    print "nothing in main, exiting ..."
