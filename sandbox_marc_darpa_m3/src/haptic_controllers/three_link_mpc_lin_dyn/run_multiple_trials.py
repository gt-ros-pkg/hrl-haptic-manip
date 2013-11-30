import roslib
roslib.load_manifest('sandbox_marc_darpa_m3')
import scipy.optimize as optimize
import subprocess
import sys
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

def run_trial(file_name, f_thresh=None, imp_scale=None):
    roscore = subprocess.Popen(['roscore'])

    time.sleep(4.0)

    simulator = subprocess.Popen(['roslaunch',
                                  'hrl_software_simulation_darpa_m3',
                                  'gen_simulator.launch'])
    time.sleep(4.0)

    load_params = subprocess.Popen(['rosrun', 
                                    'hrl_common_code_darpa_m3', 
                                    'obstacles.py', 
                                    '--pkl='+file_name])

    print 'we loaded teh params!!!'

    # load_params = subprocess.call(['rosparam', 
    #                                'load', 
    #                                file_name])

    print "started cvxgen"
    cvxgen_node = subprocess.Popen(['rosrun', 
                                    'sandbox_marc_darpa_m3',
                                    'three_link_mpc_lin_dyn'])

    if imp_scale == None:
        var_imp = subprocess.Popen(['python',
                                    'var_comp_optimize.py',
                                    '--pub_once_alpha',
                                    str(1.0)])
    else:
        print "got into setting impedance"
        var_imp = subprocess.Popen(['python',
                                    'var_comp_optimize.py',
                                    '--pub_once_alpha',
                                    str(imp_scale)])
        time.sleep(5.)
        print "finished settting impedance"

    if f_thresh == None:
        controller = subprocess.call(['python', 
                                      'basic_test_mpc_with_dynamics.py',
                                      '--tuning_params'])
    else:
        print "called the controller code ..."
        controller = subprocess.call(['python', 
                                      'basic_test_mpc_with_dynamics.py',
                                      '--tuning_params',
                                      '--f_thresh='+str(f_thresh)])
                                      #'--impulse_slope'])



    var_imp.terminate()
    roscore.terminate()
    simulator.terminate()
    cvxgen_node.terminate()
    time.sleep(15.)


def move_files(save_path):
    os.system('mv ./reach_log.pkl '+save_path+'/reach_log.pkl')
    os.system('mv ./overall_result.pkl '+save_path+'/overall_result.pkl')

def start_trials(x, f_thresh, f_list, save_path, imp_scale=None):
    #f_list = ['/home/mkillpack/Desktop/interm_test.yaml', '/home/mkillpack/Desktop/new_test3.yaml']
    ut.save_pickle({'x':x}, './x.pkl')
    print "saved pickle"

    for f_name in f_list:
        if os.path.isfile(f_name):
            buf = f_name.split('/')[-1].split('_')
            print "buf is :", buf
            buf2 = buf[3]+'_'+buf[4]+'_'+buf[5][0:3]
            path = save_path+'/'+buf2

            print "for ", path
            if  os.path.isfile(path+'/reach_log.pkl') and os.path.isfile(path+'/overall_result.pkl'):
                print "######################################\n this folder is already done \n########################"
            elif os.path.isfile(path+'/running.txt'):
                print "######################################\n this trial is running elsewhere \n########################"
            else:
                print "... starting trial"
                if not os.path.isdir(path):
                    os.system('mkdir '+path)
                os.system('touch '+path+'/running.txt')
                run_trial(f_name, f_thresh, imp_scale)
                time.sleep(5.)
                move_files(path)
                os.system('rm '+path+'/running.txt')
        else:
            print f_name, " was a folder"
    print "all done !! Exiting ..."

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--load_path', action='store', dest='load_path',
                 type='string', help='which folder path should files come from?',
                 default=None)

    p.add_option('--save_path', action='store', dest='save_path',
                 type='string', help='which folder path should files come from?',
                 default=None)

    p.add_option('--f_thresh', action='store', dest='f_thresh',
                 default=None, help='specifiy f_thresh', type='float')

    p.add_option('--t_impulse', action='store', dest='t_impulse',
                 default=None, help='specifiy f_thresh', type='float')

    p.add_option('--imp_scale', action='store', dest='imp_scale',
                 default=1.0, help='specifiy alpha', type='float')




    opt, args = p.parse_args()

    ind_to_use = None
    params = [# [166.3149183002939, 225.15009200781367, 0.8595967825526845, 0.1, -0.02, 81.56410948346854, 50.28182804815723, 0.015],
              # [239.00258542689838, 254.87433991882577, 0.74294721341314, 1.5403789279802593, -0.01787889646955531, 94.32784550386229, 35.41297936062891, 0.015], 
              [239.00258542689838, 254.87433991882577, 0.74294721341314, 20., -0.009898989898989, 94.32784550386229, 35.41297936062891, 0.015], #this is the best so far
              [239.00258542689838, 254.87433991882577, 0.74294721341314, 15., -0.0099, 94.32784550386229, 35.41297936062891, 0.015],
              #[239.00258542689838, 254.87433991882577, 0.74294721341314, 34., -0.009898989898989, 94.32784550386229, 35.41297936062891, 0.015],
              #[239.00258542689838, 254.87433991882577, 0.74294721341314, 82., -0.0099, 94.32784550386229, 35.41297936062891, 0.015],
              [173.16530068562693, 137.96938642429478, 0.032628584486035644, 82.88024924486307, -0.013090381749167561, 77.82217447234362, 36.88632734822424, 0.9616225077782078]]
              # [173.16530068562693, 137.96938642429478, 0.032628584486035644, 82.88024924486307, -0.007373737373737374, 77.82217447234362, 36.88632734822424, 0.9616225077782078],
              # [167.06765912509087, 146.5734046538604, 0.001, 72.06457418370121, -0.012927972019495094, 70.3226170045294, 52.38552850774174, 0.8864847839620789],
              # [199.59604389040584, 271.5162589100235, 0.9747637190620502, 96.75440380770155, -0.008240991489641367, 64.29633640273283, 13.553328802314727, 0.800991814582807],
              # [239.00258542689838, 254.87433991882577, 0.74294721341314, 1.5403789279802593, -0.0098989898989899, 94.32784550386229, 35.41297936062891, 0.015]]
              # [239.00258542689838, 254.87433991882577, 0.74294721341314, 1.5403789279802593, -0.01494949494949495, 94.32784550386229, 35.41297936062891, 0.015],
              # [239.00258542689838, 254.87433991882577, 0.74294721341314, 1.5403789279802593, -0.004848484848484849, 94.32784550386229, 35.41297936062891, 0.015]]
              # [239.00258542689838, 254.87433991882577, 0.74294721341314, 1.5403789279802593, -0.007373737373737374, 94.32784550386229, 35.41297936062891, 0.015]]

    if opt.t_impulse == None:
        print "I need an t_impulse value, exiting ..."
        sys.exit()
    elif opt.t_impulse < 0:
        for i in xrange(len(params)):
            if params[i][4] == opt.t_impulse:
                ind_to_use = i
                print "using the ", i, "th set of paremeters!!!!!!!!"
                print "using the ", i, "th set of paremeters!!!!!!!!"
    else:
        t_impulse = float(opt.t_impulse)

    if opt.save_path == None or opt.load_path == None:
        print "you didn't give me save or load path, exiting ..."
        sys.exit()
    if opt.f_thresh == None:
        print "i need a force threshold, exiting ..."
        sys.exit()
    else:
        f_thresh = float(opt.f_thresh)


    #this was an original from the first Sim Anneal cost that worked well, should come back to it to check!!! (as well as cost function)
    #this is wierd but it worked really well!!! 
    #x_0 = [20.2, 341, 0.01, 6.56, 0.228, 0.543, 0.0645, 0.1] #alpha, beta, zeta, mu, t_impact, u_max_nom, f_rate 
    #x_0 = [20.2, 341, 0.01, 6.56, 0.15, 0.543, 0.0645, 0.1] #alpha, beta, zeta, mu, t_impact, u_max_nom, f_rate 

    #best success rate so far (around 70), but bad forces
    # values in comments worked well in visualization, but I don't know with stats.
    # x_0 = [275.19411283302003,
    #        229.43422905319315,
    #        41.49996594393528, #0.01
    #        34.843382055782946,
    #        0.2, #0.07
    #        10.0,
    #        10.0,
    #        4.210659702932656]

    #DEFAULT USED FOR HUMANOIDS PAPER
    # for forces and speed I think we were best so far at delta_t_impulse = 0.05 or so and waypoint_size = 0.02
    # but these are best for success and speed? ...
    # x_0 = [275.19411283302003, #alpha
    #        229.43422905319315, #beta
    #        0.01, #41.49996594393528, #zeta
    #        34.843382055782946, #mu
    #        0.02, #0.04 was nominal for other trials, 0.025 for high force, high clutter 
    #        10.0, #max allowable change in u
    #        10.0, #max allowable change in force per time
    #        0.04] #this was gamma now waypoint size 4.210659702932656] #gamma


    # new params used for new impulse test
    if ind_to_use != None:
        x_0 = params[ind_to_use]
    else:
        x_0 = [239., 
               255.,
               0.743, 
               15.0, 
               t_impulse, 
               94.0, 
               35.0, 
               0.015]
        
        # old params for old impact model that had variable location for impact
        # x_0 = [275.19411283302003, #alpha
        #        229.43422905319315, #beta
        #        0.01, #41.49996594393528, #zeta
        #        34.843382055782946, #mu
        #        t_impulse, #0.04 was nominal for other trials, 0.025 for high force, high clutter 
        #        10.0, #max allowable change in u
        #        10.0, #max allowable change in force per time
        #        0.04] #this was gamma now waypoint size 4.210659702932656] #gamma



    # 20130620 sets
    # x_0 = [132.99868923957436,
    #        500., #66.87448514586538,
    #        1.#2.858958562525118,
    #        8.85879475034745,
    #        0.02, #0.06699236386604579,
    #        100.0,
    #        74.70964758039393,
    #        0.07448942321305306]

    # #20130624 set #for low low
    # x_0 = [275.19411283302003, #alpha
    #        229.43422905319315, #beta
    #        0.01, #41.49996594393528, #zeta
    #        34.843382055782946, #mu
    #        0.04, #0.04 was nominal for other trials, 0.025 for high force, high clutter 
    #        100.0, #max allowable change in u
    #        74.0, #max allowable change in force per time
    #        0.07448942321305306]


    #20130624 set v2
    # x_0 = [275.19411283302003, #alpha
    #        229.43422905319315, #beta
    #        0.01, #41.49996594393528, #zeta
    #        15., #34.843382055782946, #mu
    #        0.02, #0.04 was nominal for other trials, 0.025 for high force, high clutter 
    #        100.0, #max allowable change in u
    #        74.0, #max allowable change in force per time
    #        0.07448942321305306]



    # base = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/test_cases/high_clutter/'

    #base = '/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed080/'
    #base = '/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable040_fixed040/'
    #base = '/home/mkillpack/hrl_file_server/darpa_m3/logs/ijrr_reach_dict_pkl_files/75_obstacle_configurations_movable000_fixed020/'

    base = opt.load_path
    f_buf = os.listdir(base)
    f_list = [base+folder for folder in f_buf]
    f_list.sort()
    
    print f_list
 
    imp_scale = opt.imp_scale

    params = {}
    params['controller_gains'] = {'alpha': x_0[0],
                                  'beta': x_0[1],
                                  'zeta': x_0[2],
                                  'mu': x_0[3],
                                  't_impulse': x_0[4],
                                  'delta_u_max': x_0[5],
                                  'delta_f_max': x_0[6],
                                  'waypoint_mag': x_0[7],
                                  'impedance_scale':imp_scale,
                                  'f_thresh':f_thresh}
    params['f_thresh'] = f_thresh


    save_path = opt.save_path
    #save_path = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed_000_movable_080_25N_threshold_with_damping_delta_t_impulse_0.02'
    #save_path = '/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/dynamic_imped_scale_5.00_fixed_080_movable_000_25N_threshold_with_damping_delta_t_impulse_0.02'
    #save_path = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/fixed_040_movable_040_25N_threshold'
    # save_path = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_low_force_high_clutter_simple_large_delta_t_impulse_0.04'
    # save_path = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_high_force_low_clutter_simple_large_delta_t_impulse_0.02'
    # save_path = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_low_force_low_clutter_simple_large'
    # save_path = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_high_force_high_clutter_20130624_v2_large_delta_t_impulse_0.02'
    #save_path = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_low_force_low_clutter_20130624_large_delta_t_impulse_0.04'
    # save_path = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_high_force_high_clutter_simple_with_force_rate'

    print "got up to making folder"

    if not os.path.isdir(save_path):
        os.system('mkdir '+save_path)
        import cPickle as pkl
        f = open(save_path+'/parameters.pkl', 'w')
        pkl.dump(params, f)
        f.close()

    print "got up to start trials"
    start_trials(x_0, f_thresh, f_list, save_path, imp_scale)

