
import os
import matplotlib.pyplot as pp

import roslib
roslib.load_manifest('hrl_experiments_ijrr_dec_2011_darpa_m3')

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu

import mid_level_control.visualize_log as vl
import hrl_common_code_darpa_m3.software_simulation_setup.viz as sssv


##
# dir_path - path to top leve directory
# trial_nm_list - list of names of the form 'F12_M00/0003'
# trial_id - the name that we gave while running the trials. e.g. mpc_ft_at_base_20120419
def plot_final_contact_states(problem_direc, log_direc, trial_nm_list, kinematics):
    for nm in trial_nm_list:
        rpd = ut.load_pickle(problem_direc+'/reach_problem_dict_'+nm+'.pkl')
        log_dict = ut.load_pickle(log_direc+'/'+nm+'/reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl')

        mpu.figure()
        sssv.draw_obstacles_from_reach_problem_dict(rpd)
        sssv.draw_goal_from_reach_problem_dict(rpd)
        vl.visualize_final_contact_state(kinematics, log_dict)
        pp.savefig(nm+'.png')

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--d1', action='store', dest='d1',
                 type='string', default = None,
                 help='top level directory for method 1.')
    p.add_option('--d2', action='store', dest='d2',
                 type='string', default = None,
                 help='top level directory for method 2.')
    p.add_option('--rpd', action='store', dest='rpd',
                 type='string', default = None,
                 help='directory with all the reach problem dict pkls.')


    opt, args = p.parse_args()

    if opt.d1 == None or opt.d2 == None:
        print 'Specify both --d1 and --d2.'
        print 'Exiting ...'
        sys.exit()

    dict1 = ut.load_pickle(os.path.join(opt.d1, 'combined_results.pkl'))
    dict2 = ut.load_pickle(os.path.join(opt.d2, 'combined_results.pkl'))

    s1 = set(dict1['successful_trials'])
    s2 = set(dict2['successful_trials'])

    success_1_but_not_2 = list(s1-s2)
    success_2_but_not_1 = list(s2-s1)
    
    print 'success_1_but_not_2:', len(success_1_but_not_2)
    print success_1_but_not_2

    print 'success_2_but_not_1:', len(success_2_but_not_1)
    print success_2_but_not_1

    # make True to do some plotting.
    if False:
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
        import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa
        kinematics = gsa.RobotSimulatorKDL(d_robot)
        plot_final_contact_states(opt.rpd, opt.d2, list(success_1_but_not_2), kinematics)






