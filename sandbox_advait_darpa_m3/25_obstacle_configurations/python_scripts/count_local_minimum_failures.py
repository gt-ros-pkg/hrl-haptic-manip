
import os
import compare_vary_clutter as cvc

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')

import hrl_lib.util as ut



# for a set of 600 trials.
def count(or_success_list, single_reach_success_list):
    or_success_set = set(or_success_list)
    mpc_success_set = set(single_reach_success_list)

#    sanity_check = len(mpc_success_set - or_success_set)
#    print 'Number of times MPC succeeded but OpenRAVE failed:', sanity_check

    total_number_of_soln = len(or_success_set)
    no_local_minimum = len(or_success_set.intersection(mpc_success_set))
#    print 'Number of times both OpenRAVE and MPC succeeded:', no_local_minimum

    fail_with_soln = list(or_success_set - mpc_success_set)
    return fail_with_soln, total_number_of_soln


def test_local_minima_count(openrave_nm_list, single_reach_nm_list):
    path = '/home/advait/nfs/darpa_m3_logs/advait'
    cr_pkl_nm = 'combined_results.pkl'
    rifs_pkl_nm = 'reach_in_force_statistics.pkl'
    for i in range(len(openrave_nm_list)):
        single_nm = single_reach_nm_list[i]
        or_nm = openrave_nm_list[i]

        or_pkl = ut.load_pickle(os.path.join(path, or_nm, cr_pkl_nm))
        single_pkl = ut.load_pickle(os.path.join(path, single_nm, cr_pkl_nm))

        fail_with_soln, total_number_of_soln = count(or_pkl['successful_trials'], single_pkl['successful_trials'])

        log_pkl = 'reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl'
        local_min_count = 0
        high_force_count = 0
        for nm in fail_with_soln:
            log_dict = ut.load_pickle(os.path.join(path, single_nm, nm, log_pkl))
            if log_dict == None:
                local_min_count += 1
                continue
            if max(log_dict['max_force_list']) > 45.:
                high_force_count += 1
            else:
                local_min_count += 1

        print ''
        print 'single_nm:', single_nm
        print 'local_min_percentage:', local_min_count * 100. / total_number_of_soln
        print 'high_force_percentage:', high_force_count * 100. / total_number_of_soln
        print 'high force count:', high_force_count



if __name__ == '__main__':

    ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
                openrave_nm_list, _ = cvc.return_lists_50_percent_each()
    test_local_minima_count(openrave_nm_list, skin_nm_list)

    ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
                openrave_nm_list, _ = cvc.return_lists_75_percent_fixed()
    test_local_minima_count(openrave_nm_list, skin_nm_list)

    ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
                openrave_nm_list, _ = cvc.return_lists_25_percent_fixed()
    test_local_minima_count(openrave_nm_list, skin_nm_list)


