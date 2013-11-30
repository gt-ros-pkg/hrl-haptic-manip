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

    rviz = subprocess.Popen(['rosrun', 'rviz', 'rviz', '-d',
                             '/home/mkillpack/git/hrl_haptic_manipulation_in_clutter/hrl_software_simulation_darpa_m3/viz_config/ode_simulation.vcg'])

    time.sleep(6.0)

    simulator = subprocess.Popen(['roslaunch',
                                  'hrl_software_simulation_darpa_m3',
                                  'gen_simulator.launch'])
    time.sleep(4.0)

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

    roscore.terminate()
    rviz.terminate()
    simulator.terminate()
    cvxgen_node.terminate()
    time.sleep(15.)


def move_files(save_path):
    if not os.path.isdir(save_path):
        os.system('mkdir '+save_path)        
    os.system('mv ./reach_log.pkl '+save_path+'/reach_log.pkl')
    os.system('mv ./overall_result.pkl '+save_path+'/overall_result.pkl')

def start_trials(x, f_thresh, f_list, save_path):
    #f_list = ['/home/mkillpack/Desktop/interm_test.yaml', '/home/mkillpack/Desktop/new_test3.yaml']
    ut.save_pickle({'x':x}, './x.pkl')

    for f_name in f_list:
        buf = f_name.split('/')[-1].split('_')
        buf2 = buf[3]+'_'+buf[4]+'_'+buf[5][0:3]
        path = save_path+'/'+buf2

        if  os.path.isfile(path+'/reach_log.pkl') and os.path.isfile(path+'/overall_result.pkl'):
            print "######################################\n this folder is already done \n########################"
        else:
            run_trial(f_name, f_thresh)
            time.sleep(5.)
            move_files(path)
    print "all done !! Exiting ..."

if __name__ == '__main__':
    #this is wierd but it worked really well!!!
    #x_0 = [20.2, 341, 0.01, 6.56, 0.228, 0.543, 0.0645, 0.1] #alpha, beta, zeta, mu, t_impact, u_max_nom, f_rate 
    #x_0 = [20.2, 341, 0.01, 6.56, 0.15, 0.543, 0.0645, 0.1] #alpha, beta, zeta, mu, t_impact, u_max_nom, f_rate 

    # x_0 = [27.335196600721698, 
    #        135.7817650480842,
    #        196.99897862794683,
    #        6.9238191532083233,
    #        0.09, #0.039787524685139242,
    #        0.16717244319393917,
    #        0.014330607053289039,
    #        85.574475194413267]

    # x_0 = [14.571716832414602,
    #        127.7654682669734,
    #        262.84410976771335,
    #        6.530355842174655,
    #        0.010563026016048057,
    #        0.2542218359482329,
    #        0.05544487756297957,
    #        72.12022424152676]

    # hand tuned a bit 
    # x_0 = [27.3,
    #        136.,
    #        0.001,
    #        0.1, 
    #        0.007, #0.0598,
    #        0.167,
    #        0.0143,
    #        85.6]

    # first set from latest SA
    # x_0 = [15.0,
    #        146.8319867541657,
    #        222.60773853929007,
    #        3.7402491587165887,
    #        0.019255433997685617,
    #        0.15545875486538577,
    #        0.07936848183995318,
    #        194.77361658257226]

    # second set from latest SA
    # x_0 = [22.001547240300766,
    #        140.82144701370098,
    #        162.38555715164205,
    #        5.708958642204883,
    #        0.03364636183554705,
    #        0.11890857515088922,
    #        0.09186090152077357,
    #        196.09060069157732]

    # third set from latest SA
    # x_0 = [37.11752365386822,
    #        125.63416982668834,
    #        166.43947809598313, #0.001, 166.43947809598313,
    #        6.192142012968738,
    #        0.02448741383602415,
    #        0.24364182388902358,
    #        0.029900862869186723,
    #        .2] #198.06547386301145]

    #best success rate so far (around 70), but bad forces
    # x_0 = [275.19411283302003,
    #        229.43422905319315,
    #        0.01, #41.49996594393528,
    #        34.843382055782946,
    #        0.07, #0.2
    #        10.0, #10.0
    #        10.0,
    #        4.210659702932656]

    # x_0 = [275.19411283302003,
    #        229.43422905319315,
    #        0.01, #41.49996594393528,
    #        34.843382055782946,
    #        0.06, #0.2
    #        10.0, 
    #        10.0,
    #        4.210659702932656]

    x_0 = [275.19411283302003,
           229.43422905319315,
           0.01, #41.49996594393528,
           34.843382055782946,
           0.04, #worked really well for 0.07  #0.2
           10.0, 
           10.0,
           4.210659702932656]



    base = '/home/mkillpack'

    base2 = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/test_cases/low_clutter/' #failed_trials_high_force_low_clutter/'
    f_buf = os.listdir(base2)
    f_list = [base2+folder for folder in f_buf]

    #f_list = ['/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/test_cases/high_clutter/reach_problem_dict_87_r00_t01.pkl']

    print f_list

    # f_list = [
    #           #base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test1.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test2.yaml',
    #           #base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test3.yaml',
    #           base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test4.yaml',
    #           # base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test5.yaml',
    #           # base+'/hrl_file_server/darpa_m3/sim_anneal_tuning/test6.yaml'
    #          ]

    f_thresh = 25.0

    save_path = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/visualization_tests'
    start_trials(x_0, f_thresh, f_list, save_path)

