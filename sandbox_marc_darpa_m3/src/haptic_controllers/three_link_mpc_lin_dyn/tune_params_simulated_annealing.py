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
    import rospy

    time.sleep(2.0)

    simulator = subprocess.Popen(['roslaunch',
                                  'hrl_software_simulation_darpa_m3',
                                  'gen_simulator.launch'])
    time.sleep(2.0)

    load_params = subprocess.Popen(['rosrun', 
                                    'hrl_common_code_darpa_m3', 
                                    'obstacles.py', 
                                    '--pkl='+file_name])

    logger_process = subprocess.Popen(['rosrun', 
                                       'sandbox_advait_darpa_m3',
                                       'log_and_monitor_node.py',
                                       '--'+sys.argv[5],
                                       '--log_taxel_msg',
                                       '__name:=ode_log_and_monitor_node'])



    var_imped = subprocess.Popen(['rosrun',
                                  'sandbox_marc_darpa_m3',
                                  'var_comp_optimize.py',
                                  '--pub_once_alpha',
                                  str(1)])



    contact_memory = subprocess.Popen(['rosrun',
                                       'sandbox_advait_darpa_m3',
                                       'contact_memory.py',
                                       '--sim3',
                                       '__name:=contact_memory_node'])

    controller = subprocess.call(['rosrun',
                                  'sandbox_advait_darpa_m3',
                                  'switch_among_controllers_node.py',
                                  '--batch', 
                                  '--sim3',
                                  '--ignore_mobile_base',
                                  '', # CONTROLLER_SWITCH,
                                  '--single_reach', # REACH_SWITCH
                                  '--acf='+str(f_thresh)

    # if f_thresh == None:
    #     controller = subprocess.call(['python', 
    #                                   'basic_test_mpc_with_dynamics.py',
    #                                   '--tuning_params'])
    # else:
    #     controller = subprocess.call(['python', 
    #                                   'basic_test_mpc_with_dynamics.py',
    #                                   '--tuning_params',
    #                                   '--f_thresh='+str(f_thresh),
    #                                   '--impulse_slope'])

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
    base2 = '/home/mkillpack/hrl_file_server/darpa_m3/sim_anneal_tuning/training_cases/'
    f_buf = os.listdir(base2)
    f_list = [base2+folder for folder in f_buf]

    # f_list = [base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test1.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test2.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test3.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test4.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test5.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test6.yaml']

    #f_list = ['/home/mkillpack/Desktop/interm_test.yaml', '/home/mkillpack/Desktop/new_test3.yaml']

    ut.save_pickle({'x':x}, './x.pkl')

    f_thresh = 5.0
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
    #this is wierd but it worked really well!!!
    #x_0 = [20.2, 341, 0.01, 6.56, 0.228, 0.543, 0.0645, 0.1] #alpha, beta, zeta, mu, t_impact, u_max_nom, f_rate 

    #x_0 = [20.2, 341, 0.01, 6.56, 0.15, 0.543, 0.0645, 0.1] #alpha, beta, zeta, mu, t_impact, u_max_nom, f_rate 

    x_0 = [27.335196600721698,
           135.7817650480842,
           196.99897862794683,
           6.9238191532083233,
           0.039787524685139242,
           0.16717244319393917,
           0.014330607053289039,
           85.574475194413267]


    #x_0 = [20.2, 341, 0.00001, 6.56, 0.128, 0.543, 0.0645] #alpha, beta, zeta, mu, t_impact, u_max_nom, f_rate
    cost = opt_function(x_0)
    print "cost is :", cost
    # try:
    #     x_0   = [10., 100., 5., 0.1, 0.04, 0.5]
    #     x_max = np.array([1000., 1000., 1000., 10., 3., 2.])
    #     x_min = np.array([0.01, 0.01, 0.01, 0.001, 0.001, 0.001])

    #     xmin, Jmin, T, feval, iters, _, retval = optimize.anneal(opt_function, x_0, schedule='boltzmann', full_output=True, T0=3000., lower=x_min, upper=x_max)

    #     ut.save_pickle({'cost_history':cost_history, 
    #                     'x_min':x_min,
    #                     'iters':iters,
    #                     'T_final':T_final,
    #                     'retval':retval},
    #                    './simulated_annealing_results.pkl')

    #     print "cost history:\n", cost_history
    #     print "xmin :", xmin
    #     print "iters :", iters
    #     print "T_final :", T
    #     print "retval :", retval
    # except KeyboardInterrupt:
    #     print "control c-d it or something else happened"
    #     cvxgen_node.terminate()
    #     simulator.terminate()
    #     roscore.terminate()
