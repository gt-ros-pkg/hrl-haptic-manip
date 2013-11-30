
import sys, os
import numpy as np, math

import single_trial as st

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import hrl_lib.util as ut
import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa



class ComputeResultStatistics():
    def __init__(self):
        self.root_path = None
        pass

    def traverse_directory(self, root_path, fn_list):
        self.root_path = root_path

        for fn in fn_list:
            fn('start')

        for root, dirs, files in os.walk(root_path):
            for d in dirs:
                full_path = os.path.join(root, d)
                for fn in fn_list:
                    fn(full_path)

        for fn in fn_list:
            fn('end')

    def compute_success_rate(self, full_path):
        if full_path == 'start':
            self.success_count = 0
            self.fail_count = 0
            self.not_completed_trial_list = []
            self.success_trials_list = []
            return

        if full_path == 'end':
            print self.success_count, 'successful trials'
            print self.fail_count, 'failed trials'
            print self.success_count * 100. / (self.success_count+self.fail_count), '% success rate.'
            print self.success_count + self.fail_count, 'total trials'

            d = {}
            d['successful_trials'] = self.success_trials_list
            d['success_count'] = self.success_count
            d['fail_count'] = self.fail_count
            ut.save_pickle(d, self.root_path + '/combined_results.pkl')
            return

        pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
        pkl_list.reverse()

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+'/overall_result.pkl')
        trial = full_path.split('/')[-1]

        if res_dict is None:
            self.not_completed_trial_list.append(trial)
            self.fail_count += 1
        elif res_dict['reached_goal']:
            self.success_count += 1
            self.success_trials_list.append(trial)
        else:
            self.fail_count += 1

    def compute_joint_contact_statistics(self, full_path):
        if full_path == 'start':
            self.non_jt_contact_state_num = 0
            self.jt_contact_state_num = 0
            self.trials_with_joint_contact_num = 0
            self.trials_without_joint_contact_num = 0
            return

        if full_path == 'end':
            print 'Number of trials with contact at a joint:', self.trials_with_joint_contact_num
            print 'Number of trials WITHOUT contact at any joint:', self.trials_without_joint_contact_num

            print 'Number of contact states with joint contacts:', self.jt_contact_state_num
            print 'Number of contact states WITHOUT joint contacts:', self.non_jt_contact_state_num
            return

        pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
        pkl_list.reverse()

        for pkl in pkl_list:
            d = ut.load_pickle(pkl)
            jt_q_l, jt_loc_l, non_jt_q_l, non_jt_loc_l = st.find_states_with_contact_at_joints(d, self.d_robot, self.kinematics)
            self.jt_contact_state_num += len(jt_q_l)
            self.non_jt_contact_state_num += len(non_jt_q_l)
            if len(jt_q_l) > 0:
                self.trials_with_joint_contact_num += 1
            else:
                self.trials_without_joint_contact_num += 1

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--dir', action='store', dest='direc',
                 type='string', default = None,
                 help='top level directory with sub-directories for each reach problem.')
    p.add_option('--sim3', action='store_true', dest='sim3',
                 help='three link planar (torso, upper arm, forearm)')
    p.add_option('--sim3_with_hand', action='store_true', dest='sim3_with_hand',
                 help='three link planar (upper arm, forearm, hand)')

    opt, args = p.parse_args()

    if opt.direc == None:
        print 'Specify a root directory.'
        print 'Exiting ...'
        sys.exit()

    if opt.sim3:
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
    elif opt.sim3_with_hand:
        import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as d_robot
    else:
        print 'Please specify --sim3 or --sim3_with_hand'
        print 'Exiting ...'
        sys.exit()

    crs = ComputeResultStatistics()

    #fn_list = [crs.success_rate_for_different_stopping_force]
    #fn_list = [crs.compute_success_rate]
    #fn_list = [crs.compute_success_rate, crs.compute_reach_in_force_statistics]

    fn_list = [crs.compute_joint_contact_statistics]
    kinematics = gsa.RobotSimulatorKDL(d_robot)
    crs.kinematics = kinematics
    crs.d_robot = d_robot

    crs.traverse_directory(opt.direc, fn_list)



