
import os
import matplotlib.pyplot as pp
import numpy as np, math
import scipy.stats as ss
import copy
import sys

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu

base_path = '/home/mkillpack'

def return_lists_50_percent_each():
    dyn_skin_nm_list = [
                         #'dyn_vs_qs_comparison/dyn_trials_5N',
                         #'dyn_vs_qs_comparison/dyn_trials_5N_no_norm',
                         #'dyn_vs_qs_comparison/dyn_trials_25N',
                         'dyn_vs_qs_comparison/dyn_high_force_high_clutter_simple_with_force_rate',
                         # 'dyn_vs_qs_comparison/dyn_low_force_low_clutter',
                         #'dyn_vs_qs_comparison/dyn_high_force_high_clutter',
                         #'dyn_vs_qs_comparison/dyn_high_force_low_clutter',

                         #'dyn_vs_qs_comparison/dyn_trials_25N_less_stiff',
                         #'dyn_vs_qs_comparison/dyn_trials_25N_no_norm',
                         #'fast_mpc_tests/75_obstacle_configurations_movable010_fixed010/mpc_dyn_using_skin',
                         #'75_obstacle_configurations_movable020_fixed020/mpc_qs_1_using_best_ft_sensor',
                         #'fast_mpc_tests/75_obstacle_configurations_movable040_fixed040/mpc_dyn_using_skin',
                         #'75_obstacle_configurations_movable060_fixed060/mpc_qs_1_using_best_ft_sensor',
                         #'fast_mpc_tests/75_obstacle_configurations_movable080_fixed080/mpc_dyn_using_skin'
                         #'75_obstacle_configurations_movable100_fixed100/mpc_qs_1_using_best_ft_sensor',
                       ]

    qs_skin_nm_list = [
                        #'dyn_vs_qs_comparison/qs_trials_5N/mpc_qs_1_using_skin',
                        #'dyn_vs_qs_comparison/qs_trials_25N/mpc_qs_1_using_skin',
                        #'dyn_vs_qs_comparison/qs_trials_25N_less_stiff/mpc_qs_1_using_skin',
                        #'dyn_vs_qs_comparison/20120531_25_trials_per_point_that_worked_well/dyn_high_force_high_clutter_nom_delta_t_impulse_0.04'
                        'dyn_vs_qs_comparison/qs_high_force_high_clutter',
                         # 'dyn_vs_qs_comparison/qs_low_force_low_clutter',
                         # 'dyn_vs_qs_comparison/qs_high_force_high_clutter',
                         # 'dyn_vs_qs_comparison/qs_high_force_low_clutter',

                         #'dyn_vs_qs_comparison/qs_trials_5N_less_stiff/mpc_qs_1_using_skin',
                        #'test_new_ubuntu_and_fuerte/75_obstacle_configurations_movable010_fixed010/mpc_qs_1_using_skin',
                        #'75_obstacle_configurations_movable020_fixed020/mpc_qs_1_using_skin',
                        #'test_new_ubuntu_and_fuerte/75_obstacle_configurations_movable040_fixed040/mpc_qs_1_using_skin',
                        #'75_obstacle_configurations_movable060_fixed060/mpc_qs_1_using_skin',
                        #'test_new_ubuntu_and_fuerte/75_obstacle_configurations_movable080_fixed080/mpc_qs_1_using_skin'
                        #'75_obstacle_configurations_movable100_fixed100/mpc_qs_1_using_skin',
                        ]

    openrave_nm_list = [
                        #'75_obstacle_configurations_movable010_fixed010/openrave_128_samples',
                        #'75_obstacle_configurations_movable020_fixed020/openrave_128_samples',
                        #'75_obstacle_configurations_movable040_fixed040/openrave_128_samples',
                        #'75_obstacle_configurations_movable060_fixed060/openrave_128_samples',
                        #'75_obstacle_configurations_movable080_fixed080/openrave_128_samples',
                        #'75_obstacle_configurations_movable100_fixed100/openrave_128_samples'
                       ]

    multiple_reach_nm_list = []
                       #        '75_obstacle_configurations_movable010_fixed010/mpc_qs_1_multiple_reaches_using_skin',
                       #        '75_obstacle_configurations_movable020_fixed020/mpc_qs_1_multiple_reaches_using_skin',
                       #        '75_obstacle_configurations_movable040_fixed040/mpc_qs_1_multiple_reaches_using_skin',
                       #        '75_obstacle_configurations_movable060_fixed060/mpc_qs_1_multiple_reaches_using_skin',
                       #        '75_obstacle_configurations_movable080_fixed080/mpc_qs_1_multiple_reaches_using_skin',
                       #        '75_obstacle_configurations_movable100_fixed100/mpc_qs_1_multiple_reaches_using_skin',
                       # ]
    
    return dyn_skin_nm_list, qs_skin_nm_list, openrave_nm_list, multiple_reach_nm_list

def return_lists_0_percent_fixed():
    ft_sensor_nm_list = [
                         '75_obstacle_configurations_movable020_fixed000/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable040_fixed000/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable080_fixed000/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable120_fixed000/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable160_fixed000/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable200_fixed000/mpc_qs_1_using_best_ft_sensor',
                        ]

    skin_nm_list = [
                     '75_obstacle_configurations_movable020_fixed000/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable040_fixed000/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable080_fixed000/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable120_fixed000/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable160_fixed000/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable200_fixed000/mpc_qs_1_using_skin',
                   ]

    baseline_nm_list = [
                         '75_obstacle_configurations_movable020_fixed000/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable040_fixed000/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable080_fixed000/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable120_fixed000/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable160_fixed000/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable200_fixed000/mpc_qs_1_ignore_skin',
                       ]

    openrave_nm_list = []

    multiple_reach_nm_list = []

    return ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
           openrave_nm_list, multiple_reach_nm_list


def return_lists_100_percent_fixed():
    ft_sensor_nm_list = ['75_obstacle_configurations_movable000_fixed020/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable000_fixed040/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable000_fixed120/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable000_fixed160/mpc_qs_1_using_best_ft_sensor',
                         '75_obstacle_configurations_movable000_fixed200/mpc_qs_1_using_best_ft_sensor',
                        ]

    skin_nm_list = [        
                     '75_obstacle_configurations_movable000_fixed020/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable000_fixed040/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable000_fixed120/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable000_fixed160/mpc_qs_1_using_skin',
                     '75_obstacle_configurations_movable000_fixed200/mpc_qs_1_using_skin',
                   ]

    baseline_nm_list = [
                         '75_obstacle_configurations_movable000_fixed020/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable000_fixed040/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable000_fixed080/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable000_fixed120/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable000_fixed160/mpc_qs_1_ignore_skin',
                         '75_obstacle_configurations_movable000_fixed200/mpc_qs_1_ignore_skin',
                       ]

    openrave_nm_list = [
                         '75_obstacle_configurations_movable000_fixed020/openrave_128_samples',
                         '75_obstacle_configurations_movable000_fixed040/openrave_128_samples',
                         '75_obstacle_configurations_movable000_fixed080/openrave_128_samples',
                         '75_obstacle_configurations_movable000_fixed120/openrave_128_samples',
                         '75_obstacle_configurations_movable000_fixed160/openrave_128_samples',
                         '75_obstacle_configurations_movable000_fixed200/openrave_128_samples',
                       ]

    multiple_reach_nm_list = [
                   ]

    return ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
           openrave_nm_list, multiple_reach_nm_list

def compute_success_rate(path, pkl_nm, nm_list):
    l = []
    for nm in nm_list:
        d = ut.load_pickle(os.path.join(path, nm, pkl_nm))
        l.append(d['success_count']*100. / (d['success_count']+d['fail_count']))
    return l

def compute_success_rate_and_perc_improv(path, pkl_nm, 
                                         dyn_skin_nm_list,
                                         qs_skin_nm_list,
                                         openrave_nm_list):
    dyn_skin_success_percent = []
    qs_skin_success_percent = []
    openrave_success_percent = []
    #perc_improv_skin_ft = []
    for dyn_skin_nm, qs_skin_nm in zip(dyn_skin_nm_list, qs_skin_nm_list):
        dyn_dict = ut.load_pickle(os.path.join(path, dyn_skin_nm, pkl_nm))
        qs_dict = ut.load_pickle(os.path.join(path, qs_skin_nm, pkl_nm))
        dyn_skin_success_percent.append(dyn_dict['success_count']*100./(dyn_dict['success_count']+dyn_dict['fail_count']))
        qs_skin_success_percent.append(qs_dict['success_count']*100./(qs_dict['success_count']+qs_dict['fail_count']))
        #perc_improv_skin_ft.append((skin_success_percent[-1] * 1. - ft_success_percent[-1]) / ft_success_percent[-1] *100) 
    
    # for bs_nm, or_nm in zip(baseline_nm_list, openrave_nm_list):
    #     base_dict = ut.load_pickle(os.path.join(path, bs_nm, pkl_nm))
    #     or_dict = ut.load_pickle(os.path.join(path, or_nm, pkl_nm))
    #     baseline_success_percent.append(base_dict['success_count']*100./(base_dict['success_count']+base_dict['fail_count']))
    #     openrave_success_percent.append(or_dict['success_count']*100./(or_dict['success_count']+or_dict['fail_count']))
    
    return dyn_skin_success_percent, qs_skin_success_percent, openrave_success_percent

# def fixed_0_percent():
#     path = base_path+'/hrl_file_server/darpa_m3/final_ijrr_results_with_10_second_timeout/'
#     pkl_nm = 'combined_results.pkl'

#     ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
#                 openrave_nm_list, _ = return_lists_0_percent_fixed()

#     obj_count_list = [20, 40, 80, 120, 160, 200]
#     ft_success_percent, skin_success_percent, baseline_success_percent,\
#     openrave_success_percent, perc_improv_skin_ft = compute_success_rate_and_perc_improv(path, pkl_nm, obj_count_list,
#                                                                      ft_sensor_nm_list, skin_nm_list,
#                                                                      baseline_nm_list, skin_nm_list)

#     openrave_success_percent = [100 for o in openrave_success_percent]
#     pp.plot(obj_count_list, openrave_success_percent, 'r-o', mew=0, linewidth=2.0, markersize=5.0)
#     pp.plot(None, None, 'r-o', mew=0, label='Estimated optimal')
#     pp.plot(obj_count_list, baseline_success_percent, 'y--^', mew=0, linewidth=1.5)
#     pp.plot(None, None, 'y--^', mew=0, label='MPC without tactile\n or FT sensors')
#     pp.plot(obj_count_list, skin_success_percent, 'b-.s', mew=0, label='MPC with tactile sensing')
#     pp.plot(obj_count_list, ft_success_percent, 'g:D', mew=0, label='MPC with FT sensors')
#     #pp.plot([20, 40, 80, 120, 160], [100]*5, 'k-o', mew=0, label='MPC with proximity sensors')
#     pp.xlabel('Total number of cylinders', labelpad=2)
#     pp.ylim((-5,105))
#     pp.xlim((10,210))
#     pp.title('100\% movable, 0\% fixed')
#     mpu.legend(draw_frame=False, numpoints=20 )   #display_mode='less_space')
#     pp.ylabel('Success percent', labelpad=-2)

#     return obj_count_list, perc_improv_skin_ft, ft_success_percent, skin_success_percent


def fixed_50_percent():
    #path = base_path+'/hrl_file_server/darpa_m3/marc_final_ijrr_results_data_new_ft_sensor/'
    #path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
    #path = base_path+'/hrl_file_server/darpa_m3/final_ijrr_results_with_10_second_timeout/'
    path = base_path+'/hrl_file_server/darpa_m3/'
    #path = base_path+'/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
    #path = '/home/advait/nfs/darpa_m3_logs/advait'
    pkl_nm = 'combined_results.pkl'
    
    dyn_skin_nm_list, qs_skin_nm_list, openrave_nm_list, _ = return_lists_50_percent_each()


    #obj_count_list = [20, 40, 60, 80]
    #obj_count_list = [20, 40, 80, 120, 160, 200]
    obj_count_list = [80]
    #obj_count_list = [20]#, 80, 160]
    dyn_skin_success_percent, qs_skin_success_percent, \
        openrave_success_percent = compute_success_rate_and_perc_improv(path, pkl_nm, 
                                                                        dyn_skin_nm_list, qs_skin_nm_list,
                                                                        openrave_nm_list)
    #mpu.set_figure_size(8, 8.0)
    #pp.figure()

    #pp.plot(obj_count_list, openrave_success_percent, 'r-o', mew=0, label='Estimated optimal')
    pp.plot(obj_count_list, qs_skin_success_percent, 'b-.s', mew=0, label='MPC with tactile sensing')
    pp.plot(obj_count_list, dyn_skin_success_percent, 'g:D', mew=0, label='MPC with FT sensors')
    #pp.plot(obj_count_list, baseline_success_percent, 'y--^', mew=0,
    #label='MPC without tactile\n\t\t or FT sensors')

    #pp.ylabel('success percent')
    #pp.xlabel('\# of fixed and movable cylinders')
    pp.xlabel('Total number of cylinders', labelpad=2)
    pp.ylim((-5,105))
    pp.xlim((10,210))
    mpu.legend(display_mode='less_space', draw_frame=False)
    #mpu.reduce_figure_margins(left=0.15, bottom=0.18, right=0.97, top=0.85)
    pp.title('50\% movable, 50\% fixed')
    #pp.savefig('vary_both_success_rate.pdf')

    return obj_count_list, dyn_skin_success_percent, qs_skin_success_percent

# def fixed_100_percent():
#     #path = base_path+'/hrl_file_server/darpa_m3/marc_final_ijrr_results_data_new_ft_sensor/'
#     #path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
#     path = base_path+'/hrl_file_server/darpa_m3/final_ijrr_results_with_10_second_timeout/'
#     #path = base_path+'/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
#     pkl_nm = 'combined_results.pkl'

#     ft_sensor_nm_list, skin_nm_list, baseline_nm_list,\
#                 openrave_nm_list, _ = return_lists_100_percent_fixed()

#     obj_count_list = [20, 40, 80, 120, 160, 200]
#     #obj_count_list = [40, 120, 160]
#     ft_success_percent, skin_success_percent, baseline_success_percent,\
#     openrave_success_percent, perc_improv_skin_ft = compute_success_rate_and_perc_improv(path, pkl_nm, obj_count_list,
#                                                                      ft_sensor_nm_list, skin_nm_list,
#                                                                      baseline_nm_list, openrave_nm_list)

#     pp.plot(obj_count_list, openrave_success_percent, 'r-o', mew=0, label='Estimated optimal')
#     pp.plot(obj_count_list, skin_success_percent, 'b-.s', mew=0,
#             label='MPC with tactile sensing')
#     pp.plot(obj_count_list, ft_success_percent, 'g:D', mew=0, label='MPC with FT sensors')
#     pp.plot(obj_count_list, baseline_success_percent, 'y--^', mew=0,
#             label='MPC without tactile\n or FT sensors')
#     #pp.plot([20, 40, 80, 120, 160], [52.42, 24.75, 6.58, 0.75, 0.67], 'k-o', mew=0, label='MPC with proximity sensors')
#     pp.xlabel('Total number of cylinders', labelpad=2)
#     pp.ylim((-5,105))
#     pp.xlim((10,210))
#     pp.title('0\% movable, 100\% fixed')
#     #pp.ylabel('Success percent', labelpad=-2)

#     return obj_count_list, perc_improv_skin_ft, ft_success_percent, skin_success_percent

def percentile_force_statistics(pth, pkl_nm, l):
    avg_l = []
    percentile_95_l = []
    for nm in l:
        d = ut.load_pickle(os.path.join(pth,nm,pkl_nm))
        avg_l.append(np.mean(d['percentile_95_force_list']))
        percentile_95_l.append(ss.scoreatpercentile(d['percentile_95_force_list'], 95))
    return avg_l, percentile_95_l

def max_force_statistics(pth, pkl_nm, l):
    avg_l = []
    max_l = []
    percentile_l = []
    for nm in l:
        d = ut.load_pickle(os.path.join(pth,nm,pkl_nm))
        avg_l.append(d['avg_contact_force'])
        max_l.append(d['avg_max_force'])
        max_force_list = d['max_force_list']
        percentile_l.append(ss.scoreatpercentile(max_force_list, 95))
    return avg_l, max_l, percentile_l

def print_computed_force_statistics(path, pkl_nm, ft_sensor_nm_list,
                                    skin_nm_list, baseline_nm_list):
    avg_ft_l, max_ft_l, tq_ft_l = max_force_statistics(path, pkl_nm, ft_sensor_nm_list)
    avg_skin_l, max_skin_l, tq_skin_l = max_force_statistics(path, pkl_nm, skin_nm_list)
    avg_baseline_l, max_baseline_l, tq_baseline_l = max_force_statistics(path, pkl_nm, baseline_nm_list)
    print 'Avg. Max Force:'
    print 'Skin -', max_skin_l
    print 'FT -', max_ft_l
    print 'Baseline -', max_baseline_l
    print
    print '95 percentile max force:'
    print 'Skin -', tq_skin_l
    print 'FT -', tq_ft_l
    print 'Baseline -', tq_baseline_l
    print

    avg_ft_l, perc_ft_l = percentile_force_statistics(path, pkl_nm, ft_sensor_nm_list)
    avg_skin_l, perc_skin_l = percentile_force_statistics(path, pkl_nm, skin_nm_list)
    avg_baseline_l, perc_baseline_l = percentile_force_statistics(path, pkl_nm, baseline_nm_list)
    print 'Avg. 95 percentile force:'
    print 'Skin -', avg_skin_l
    print 'FT -', avg_ft_l
    print 'Baseline -', avg_baseline_l
    print
    print '95 percentile of 95 percentile force:'
    print 'Skin -', perc_skin_l
    print 'FT -', perc_ft_l
    print 'Baseline -', perc_baseline_l

def compute_force_statistics():
    path = base_path+'/hrl_file_server/darpa_m3/marc_final_ijrr_results_data_new_ft_sensor/'
    #path = base_path+'/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
    # path = '/home/advait/nfs/darpa_m3_logs/advait'
    pkl_nm = 'reach_in_force_statistics.pkl'

    obj_count_list = [20, 40, 80, 120, 160]
    print 'Total Obstacles:', obj_count_list
    print

    print '========================================================='
    print '50% fixed'
    print '========================================================='
    ft_sensor_nm_list, skin_nm_list, baseline_nm_list, _, _ = return_lists_50_percent_each()
    print_computed_force_statistics(path, pkl_nm, ft_sensor_nm_list,
                                    skin_nm_list, baseline_nm_list)

    print '========================================================='
    print '75% fixed'
    print '========================================================='
    ft_sensor_nm_list, skin_nm_list, baseline_nm_list, _, _ = return_lists_75_percent_fixed()
    print_computed_force_statistics(path, pkl_nm, ft_sensor_nm_list,
                                    skin_nm_list, baseline_nm_list)

    print '========================================================='
    print '25% fixed'
    print '========================================================='
    ft_sensor_nm_list, skin_nm_list, baseline_nm_list, _, _ = return_lists_25_percent_fixed()
    print_computed_force_statistics(path, pkl_nm, ft_sensor_nm_list,
                                    skin_nm_list, baseline_nm_list)

def compute_n_reaches_for_success(path, pkl_nm, nm_list):
    l = []
    for nm in nm_list:
        d = ut.load_pickle(os.path.join(path, nm, pkl_nm))
        l.extend(d['n_reaches_for_success_list'])

    arr = np.array(l)
    min_val = np.min(arr)
    max_val = np.max(arr)
    res_dict = {}
    for i in range(min_val, max_val+1):
        res_dict[i] = np.where(arr == i)[0].shape[0]
    return res_dict

def compare_average_velocity(path, pkl_nm, qs_nm_list, dyn_nm_list):
    dist_l_dyn = []
    time_l_dyn = []
    dist_l_qs = []
    time_l_qs = []
    exec_t_dyn = []
    exec_t_qs = []
    exec_t_diff = []

    for qs_nm, dyn_nm in zip(qs_nm_list, dyn_nm_list):
        d_dyn = ut.load_pickle(os.path.join(path, dyn_nm, pkl_nm))
        d_qs = ut.load_pickle(os.path.join(path, qs_nm, pkl_nm))

        for key in d_qs['success_vel_dict'].keys():
            # if d_skin['success_vel_dict'][key]['success'] == d_ft['success_vel_dict'][key]['success'] and d_skin['success_vel_dict'][key]['success'] == True:
            #     dist_l_skin.append(d_skin['success_vel_dict'][key]['dist'])
            #     dist_l_ft.append(d_ft['success_vel_dict'][key]['dist'])
            #     time_l_skin.append(d_skin['success_vel_dict'][key]['time'])
            #     time_l_ft.append(d_ft['success_vel_dict'][key]['time'])
            if d_qs['success_vel_dict'][key]['success'] == True:
                dist_l_qs.append(d_qs['success_vel_dict'][key]['dist'])
                time_l_qs.append(d_qs['success_vel_dict'][key]['time'])
            if d_dyn['success_vel_dict'][key]['success'] == True:
                dist_l_dyn.append(d_dyn['success_vel_dict'][key]['dist'])
                time_l_dyn.append(d_dyn['success_vel_dict'][key]['time'])

            if d_qs['success_vel_dict'][key]['success'] == True and d_dyn['success_vel_dict'][key]['success'] == True:
                exec_t_dyn.append(d_dyn['success_vel_dict'][key]['time'])
                exec_t_qs.append(d_qs['success_vel_dict'][key]['time'])
                exec_t_diff.append(exec_t_qs[-1] - exec_t_dyn[-1])

    t_statistic, p_value = ss.ttest_1samp(exec_t_diff, 0) 

    print "p-value for paired test is :", p_value
    print "average diff is :", np.mean(exec_t_diff)
    print "average time for qs :", np.mean(exec_t_qs)
    print "average time for dyn :", np.mean(exec_t_dyn)

    qs_vel_list = np.array(dist_l_qs)/np.array(time_l_qs)
    dyn_vel_list = np.array(dist_l_dyn)/np.array(time_l_dyn)

    total_dist_qs = np.sum(dist_l_qs)
    total_time_qs = np.sum(time_l_qs)
    total_dist_dyn = np.sum(dist_l_dyn)
    total_time_dyn = np.sum(time_l_dyn)

    print "OTHER WAY TO DO:"
    print "dyn :", np.mean(dyn_vel_list), np.std(dyn_vel_list)
    print "qs :", np.mean(qs_vel_list), np.std(qs_vel_list)
    print "ttest between dyn and qs", ss.ttest_ind(dyn_vel_list, qs_vel_list)
    print "#################################################################"
    print "#################################################################"
    print "#################################################################"

    return total_dist_qs / total_time_qs, total_dist_dyn / total_time_dyn

def compute_average_velocity(path, pkl_nm, nm_list):
    dist_l = []
    time_l = []

    for nm in nm_list:
        d = ut.load_pickle(os.path.join(path, nm, pkl_nm))
        try:
            dist_l.extend(d['ee_distance_list'])
            time_l.extend(d['execution_time_list'])
        except:
            print "DIED HERE:"
            print os.path.join(path, nm, pkl_nm), "\n"
    total_dist = np.sum(dist_l)
    total_time = np.sum(time_l)
    return total_dist / total_time

def trials_above_force_value(max_force_list, starting_force, ending_force):
    l = copy.copy(max_force_list)
    l.sort()
    n = len(l)
    y = np.array(range(n))
    max_force_arr = np.array(l)
    start_idx_buf = np.where(max_force_arr > starting_force)
    start_idx = start_idx_buf[0][0]
    return max_force_arr[start_idx:], (n - y[start_idx:])/ (n*1.)

def plot_perc_contact_forces_above_force_threshold(perc_dyn, perc_qs, x):
    mpu.set_figure_size(8,7)
    pp.figure()
    mpu.reduce_figure_margins(left=0.2, bottom=0.2, top=0.80)
    #mpu.reduce_figure_margins(left=0.14, bottom=0.2, top=0.85)

    #pp.plot(x, perc_ft, color='g',
    pp.plot(x, perc_dyn, 'g:',
            label='MPC with dynamic model')
    pp.plot(x, perc_qs, color='b',
            label='MPC with quasi-static model')
    #pp.xlim(5.5, 20.5)
    #pp.ylim(0, max(np.max(perc_dyn), np.max(perc_qs))+1)
    #pp.axhline(0.05, c='k', ls=':')
    #pp.xlabel('95 percentile force magnitude (N)')
    pp.xlabel('force magnitude (N)')
    pp.ylabel('contact forces above \n force magnitude (\%)')
    #pp.ylim((0,8.0))

    mpu.legend(display_mode='less_space', draw_frame=False)



def plot_number_above_force_threshold(max_force_ft, max_force_skin):
    mpu.set_figure_size(8,5)
    pp.figure()
    mpu.reduce_figure_margins(left=0.17, bottom=0.2, top=0.85)

    #starting_force = 5.
    starting_force = 5.
    ending_force = 35.
    ft_force_arr, ft_count_arr = trials_above_force_value(max_force_ft,
                                                          starting_force,
                                                          ending_force)
    skin_force_arr, skin_count_arr = trials_above_force_value(max_force_skin,
                                                              starting_force,
                                                              ending_force)

    pp.plot(ft_force_arr, ft_count_arr, color='g',
            label='MPC with FT sensors')
    pp.plot(skin_force_arr, skin_count_arr, color='b',
            label='MPC with tactile sensing')
    pp.xlim(starting_force, ending_force)
    pp.ylim(0, max(ft_count_arr.max(), skin_count_arr.max()))
    #pp.axhline(0.05, c='k', ls=':')
    #pp.xlabel('95 percentile force magnitude (N)')
    pp.xlabel('95th percentile force magnitude (N)')
    pp.ylabel('Fraction of trials')
    pp.ylim((0,0.2))

    mpu.legend(display_mode='less_space', draw_frame=False)

def plot_force_histogram(f_mag_l):
    max_force = max(f_mag_l)
    bin_width = 0.5
    bins = np.arange(0.-bin_width/2., max_force+2*bin_width, bin_width)
    hist, bin_edges = np.histogram(np.array(f_mag_l), bins)
    mpu.plot_histogram(bin_edges[:-1]+bin_width/2., hist,
                       width=bin_width*0.8)
    pp.xlabel('max contact force (N)')
    #pp.xlabel('95 percentile contact force (N)')
    pp.ylabel('bin count')


def statistical_analysis_success_percent(skin_succ_perc, ft_succ_perc, n1, n2=0):
    success_perc_diff_l = []
    conf_95_interval_l = []
    p_value_l = []
    for p1hat_perc, p2hat_perc in zip(skin_succ_perc, ft_succ_perc):
        p1hat, p2hat = p1hat_perc/100., p2hat_perc/100.
        if n2 == 0:
            phat = (p1hat + p2hat) / 2 # each case has the same number of samples/trials
            se = math.sqrt(p1hat*(1-p1hat)/n1 + p2hat*(1-p2hat)/n1)
        else:
            phat = (p1hat*n1 + p2hat*n2) / (n1+n2) # if each case has a different # of samples
            se = math.sqrt(p1hat*(1-p1hat)/n1 + p2hat*(1-p2hat)/n2)            

            

        success_perc_diff_l.append(p2hat_perc - p1hat_perc)
        conf_95_interval_l.append(1.96 * se * 100.)

        se_h0 = math.sqrt(phat*(1-phat)*2./(n1+n2))
        if se_h0 != 0:
            z = (p1hat - p2hat) / se_h0
        else:
            z = 10000.
        # print "phat is :\t", phat
        # print "se_h0 is :\t", se_h0

        print "z is :\t", z
        if z < 0.:
            z = -z

        p_value_l.append(2 * ss.norm.sf(z))

    return success_perc_diff_l, conf_95_interval_l, p_value_l


def get_percentages_above_thresh(direc, nm, x_values):
    dyn_total = 0
    dyn_above = [0]*len(x_values)

    qs_total = 0
    qs_above = [0]*len(x_values)

    for name in nm:
        #print direc+'../marc_test_final_ft_sensor_2_second_timeout/'+name+'/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl'
        #data_ft = ut.load_pickle(direc+'../marc_test_ft_off_link_2_sec_timeout/'+name+'/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
        # dyn_data = ut.load_pickle(direc+'fast_mpc_tests/'+name+'/mpc_dyn_using_skin/reach_in_force_statistics.pkl')
        # qs_data = ut.load_pickle(direc+'test_new_ubuntu_and_fuerte/'+name+'/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')


        dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_high_force_high_clutter_simple_with_force_rate/reach_in_force_statistics.pkl')
        #dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_low_force_low_clutter/reach_in_force_statistics.pkl')
        #dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_high_force_high_clutter/reach_in_force_statistics.pkl')
        #dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_high_force_low_clutter/reach_in_force_statistics.pkl')
                        
        #qs_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/20120531_25_trials_per_point_that_worked_well/dyn_high_force_high_clutter_nom_delta_t_impulse_0.04/reach_in_force_statistics.pkl')
        #qs_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/qs_low_force_low_clutter/reach_in_force_statistics.pkl')
        qs_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/qs_high_force_high_clutter/reach_in_force_statistics.pkl')
        #qs_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/qs_high_force_low_clutter/reach_in_force_statistics.pkl')

        #dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_trials_5N/reach_in_force_statistics.pkl')
        # dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_trials_5N_no_norm/reach_in_force_statistics.pkl')
        #dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_trials_25N/reach_in_force_statistics.pkl')
        #dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_trials_25N_less_stiff/reach_in_force_statistics.pkl')
        #dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_trials_5N_less_stiff/reach_in_force_statistics.pkl')

        # dyn_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/dyn_trials_25N_no_norm/reach_in_force_statistics.pkl')

        #qs_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/qs_trials_5N/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
        #qs_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/qs_trials_25N/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
        #qs_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/qs_trials_5N_less_stiff/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
        #qs_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/qs_high_force_high_clutter/reach_in_force_statistics.pkl')
        #qs_data = ut.load_pickle(direc+'dyn_vs_qs_comparison/qs_trials_25N_less_stiff/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')

        
        # if qs_data == None: 
        #     print "NO DATA AT :", direc+'test_new_ubuntu_and_fuerte/'+name+'/mpc_qs_1_using_skin/reach_in_force_statistics.pkl'
        #     sys.exit()
        # elif dyn_data == None:
        #     print "NO DATA AT :", direc+'fast_mpc_tests/'+name+'_v3/mpc_dyn_using_skin/reach_in_force_statistics.pkl'
        #     sys.exit()
  
        dyn_total = dyn_total + len(dyn_data['all_forces_mag_list'])
        qs_total = qs_total + len(qs_data['all_forces_mag_list'])

        for i in xrange(len(x_values)):
            qs_above[i] = qs_above[i] + len(np.where(np.array(qs_data['all_forces_mag_list']) > x_values[i])[0])
            dyn_above[i] = dyn_above[i] + len(np.where(np.array(dyn_data['all_forces_mag_list']) > x_values[i])[0])
            
    dyn_result = []
    qs_result = []

    for i in xrange(len(x_values)):
        dyn_result.append(float(dyn_above[i])/float(dyn_total)*100)
        qs_result.append(float(qs_above[i])/float(qs_total)*100)

    return dyn_result, qs_result, dyn_total, qs_total


if __name__ == '__main__':
    success_rate_plots = True 
    force_statistics = False #this function is broken but may be useful if repaired - marc Dec 2012
    average_velocity = True
    n_reaches_for_success = False
    plot_percent_above_threshold = True

    if plot_percent_above_threshold:
        print "got in"

        # nm_0 = ['75_obstacle_configurations_movable020_fixed000',
        #         '75_obstacle_configurations_movable040_fixed000',
        #         '75_obstacle_configurations_movable080_fixed000',
        #         '75_obstacle_configurations_movable120_fixed000',
        #         '75_obstacle_configurations_movable160_fixed000',
        #         '75_obstacle_configurations_movable200_fixed000']


        nm_50 = ['75_obstacle_configurations_movable010_fixed010',]
                 #'75_obstacle_configurations_movable020_fixed020',
                 #'75_obstacle_configurations_movable040_fixed040',
                 #'75_obstacle_configurations_movable060_fixed060',
                 #'75_obstacle_configurations_movable080_fixed080']
                 #'75_obstacle_configurations_movable100_fixed100']
                 #]

        # nm_100 = ['75_obstacle_configurations_movable000_fixed020',
        #           '75_obstacle_configurations_movable000_fixed040',
        #           '75_obstacle_configurations_movable000_fixed080',
        #           '75_obstacle_configurations_movable000_fixed120',
        #           '75_obstacle_configurations_movable000_fixed160',
        #           '75_obstacle_configurations_movable000_fixed200']

        direc = '/home/mkillpack/hrl_file_server/darpa_m3/'

        #x_values = [5+i*0.125 for i in xrange(89)]
        x_values = [25+i*0.125 for i in xrange(89)]

        
        print "getting values from pkl files, will take a while ..."

        ########## same stats as below for zero percent fixed object environments ########## 
        ########## same stats as below for zero percent fixed object environments ########## 
        # perc_skin_0, perc_ft_0, skin_total_0, ft_total_0 = get_percentages_above_thresh(direc, nm_0, x_values)

        # print "perc_skin_0 :\n", perc_skin_0
        # print "perc_ft_0 :\n", perc_ft_0

        # print "p-values for 0:\n", statistical_analysis_success_percent(perc_skin_0, perc_ft_0, skin_total_0, ft_total_0)[2], "\n"

        # print "95 perc conf intervals for 0: \n", statistical_analysis_success_percent(perc_skin_0, perc_ft_0, skin_total_0, ft_total_0)[1], "\n"

        # plot_perc_contact_forces_above_force_threshold(perc_ft_0, perc_skin_0, x_values)
        # pp.title('0\% fixed')
        # pp.ylim((0, 20.))
        # pp.savefig('contact_force_percent_above_threshold_0_perc_fixed.pdf')

        ########## same stats as below for zero percent fixed object environments ########## 
        ########## same stats as below for zero percent fixed object environments ########## 

        perc_dyn_50, perc_qs_50, dyn_total_50, qs_total_50 = get_percentages_above_thresh(direc, nm_50, x_values)

        x_values2 = [5*1.1]
        perc_dyn_50_one, perc_qs_50_one, _, _ = get_percentages_above_thresh(direc, nm_50, x_values2)
        print "perc_dyn_50 :\n", perc_dyn_50
        print "perc_qs_50 :\n", perc_qs_50

        print "for diff calc dyn:", perc_dyn_50_one
        print "for diff calc qs:", perc_qs_50_one


        plot_perc_contact_forces_above_force_threshold(perc_dyn_50, perc_qs_50, x_values)
        pp.title('80 fixed obstacles \n (25N force threshold)')
        pp.ylim((0, 30.))
        pp.xlim((24, 37.))
        pp.savefig('contact_force_stats_high_force_high_clutter.pdf')

        # perc_skin_100, perc_ft_100, _, skin_total_100, ft_total_100, _ = get_percentages_above_thresh(direc, nm_100, x_values)

        # print "perc_skin_100 :\n", perc_skin_100
        # print "perc_ft_100 :\n", perc_ft_100

        # print "perc_100 skin :", 1./(np.array(perc_skin_100)/100.), "\n"
        # print "perc_100 ft :", 1./(np.array(perc_ft_100)/100.)

        # plot_perc_contact_forces_above_force_threshold(perc_ft_100, perc_skin_100, x_values)
        # pp.title('100\% fixed')
        # pp.ylim((0, 12.))
        # pp.xlim((5, 17.))
        # pp.savefig('contact_force_percent_above_threshold_100_perc_fixed.pdf')

        pp.show()

        print "p-values for 50:\n", statistical_analysis_success_percent(perc_dyn_50, perc_qs_50, dyn_total_50, qs_total_50)[2], "\n"
        # print "p-values for 100:\n", statistical_analysis_success_percent(perc_skin_100, perc_ft_100, skin_total_100, ft_total_100)[2], "\n"        

        print "95 perc conf intervals for 50: \n", statistical_analysis_success_percent(perc_dyn_50, perc_qs_50, dyn_total_50, qs_total_50)[1], "\n"
        # print "95 perc conf intervals for 100:\n", statistical_analysis_success_percent(perc_skin_100, perc_ft_100, skin_total_100, ft_total_100)[1], "\n"        


    if force_statistics:
        compute_force_statistics()

    if average_velocity:
        path = base_path+'/hrl_file_server/darpa_m3/'
        #path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
        #path = '/hrl_file_server/darpa_m3/marc_ijrr_revision_2_all_data'
        pkl_nm = 'reach_in_kinematics_statistics.pkl'

        dyn_l, qs_l, openrave_l, _ = return_lists_50_percent_each()
        # ft2_l, skin2_l, baseline2_l, _, _ = return_lists_100_percent_fixed()
        # ft3_l, skin3_l, baseline3_l, _, _ = return_lists_0_percent_fixed()
        #obj_count_list = [20, 40, 80, 120, 160, 200]
        obj_count_list = [80]

        print "path is :", path
        print "pkl_nm is :", pkl_nm

        print 'Average velocity with dynamic model:', compute_average_velocity(path, pkl_nm, dyn_l)
        print 'Average velocity with quasi-static model:', compute_average_velocity(path, pkl_nm, qs_l)

        qs_avg_vel, dyn_avg_vel = compare_average_velocity(path, pkl_nm, qs_l, dyn_l)
        print 'Average velocity for successful trials only 50 50 :', "\n qs :", qs_avg_vel, "\n dyn :", dyn_avg_vel

        # ft_avg_vel, skin_avg_vel = compare_average_velocity_skin_ft(path, pkl_nm, ft1_l, skin1_l, baseline1_l)
        # print 'Average velocity for successful trials only 50 50 :', "\n ft :", ft_avg_vel, "\n skin :", skin_avg_vel
        # ft_avg_vel, skin_avg_vel = compare_average_velocity_skin_ft(path, pkl_nm, ft2_l, skin2_l)
        # print 'Average velocity for successful trials only 100 fixed :', "\n ft :", ft_avg_vel, "\n skin :", skin_avg_vel
        # ft_avg_vel, skin_avg_vel = compare_average_velocity_skin_ft(path, pkl_nm, ft3_l, skin3_l)
        # print 'Average velocity for successful trials only 0 fixed :', "\n ft :", ft_avg_vel, "\n skin :", skin_avg_vel



    if n_reaches_for_success:
        # for 10 seconds
        # estimated_optimal_success = (1183+1133+983+783+555) ##update this as well from # of files in each folder  
        # d={0:0, 1:1080+920+663+463+270, 2:60+119+131+130+106, 3:15+21+57+52+41, 4:2+7+17+13+21, 5:1+8+6+4+6, -1:25+58+109+113+111}


        #for 2 seconds
        #######this is the one we used for the final IJRR paper - marc Dec. 2012
        estimated_optimal_success = (1183+1133+983+783+555+446) ##update this as well from # of files in each folder  
        d={0:0, 1:1054+907+624+442+260+199 , 2:86+131+160+175+118+78 , 3:16+35+76+50+48+39, 4:3+13+28+27+21+14 , 5:1+10+10+6+6+10, -1:40+104+500}
        #######this is the one we used for the final IJRR paper - marc Dec. 2012

        mpu.set_figure_size(8, 5)
        fig = pp.figure()

        n_reach_list = []
        count_l = []
        for k in d.keys():
            if k == -1 or k==0:
                continue
            n_reach_list.append(k)
            count_l.append(d[k])

        total_success = np.sum(count_l)
        frac_optimal = np.cumsum(count_l)*1./estimated_optimal_success  #estimated_optimal_success
        
        print "frac_optimal is :", frac_optimal

        pp.plot(n_reach_list, frac_optimal, '-o', mew=0)
        pp.ylim((0.65, 1.))
        pp.xlim((0,6))
        pp.xlabel('Number of greedy MPC reaches')
        pp.ylabel('Fraction of \nestimated optimal', labelpad=14,
                horizontalalignment='center')
        mpu.reduce_figure_margins(left=0.22, bottom=0.20, right=0.98, top=0.97)
        pp.savefig('success_with_retries_50_50_only.pdf')
        pp.show()


    if success_rate_plots:
        mpu.set_figure_size(8*3, 8.0)
        #fig = pp.figure()
        #fig.subplots_adjust(hspace=0.350, wspace=0.0001, bottom=0.17,
                            #top=0.84, left=0.045, right=0.99)

        # sp1 = pp.subplot(1,3,1)
        # obj_l_all_move, pi_all_move, ft_0, skin_0 = fixed_0_percent()

        # print "ft_0 :\n", ft_0
        # print "skin_0 :\n", skin_0
        # print "p-values :\n", statistical_analysis_success_percent(skin_0, ft_0, 1200)[2]

        #sp2 = pp.subplot(1,3,2)
        fi2 = pp.figure()
        obj_l_both, dyn_50, qs_50 = fixed_50_percent()

        print "dyn_50 :\n", dyn_50
        print "qs_50 :\n", qs_50
        print "p-values :\n", statistical_analysis_success_percent(dyn_50, qs_50, 1200)[2]

        # sp3 = pp.subplot(1,3,3)
        # obj_l_all_fixed, pi_all_fixed, ft_100, skin_100 = fixed_100_percent()

        # print "ft_100 :\n", ft_100
        # print "skin_100 :\n", skin_100
        # print "p-values :\n", statistical_analysis_success_percent(skin_100, ft_100, 1200)[2]

        #pp.setp(sp2.get_yticklabels(), visible=True)
        #pp.setp(sp3.get_yticklabels(), visible=False)
        pp.savefig('fast_mpc_success_rate_with_clutter.pdf')

        if False:
            mpu.set_figure_size(7.4, 7.0)
            pp.figure()

            pp.plot(obj_l_all_fixed, pi_all_fixed, 'c:s', mew=0)#, label='0\% movable, 100\% fixed')
            pp.plot(np.NaN, np.NaN, 'c:s', mew=0, ms=4., label='100\% fixed')
            print "percent improvement over ft sensors for 100\% fixed", pi_all_fixed

            pp.plot(obj_l_both, pi_both, 'm--^', mew=0)#, label='50\% movable, 50\% fixed')
            pp.plot(np.NaN, np.NaN, 'm--^', mew=0, ms=4., label='50\% fixed')
            print "percent improvement over ft sensors for 50\% fixed", pi_both

            pp.plot(obj_l_all_move, pi_all_move, 'k-o', mew=0)#, label='100\% movable, 0\% fixed')
            pp.plot(np.NaN, np.NaN, 'k-o', mew=0, ms=4., label='0\% fixed')
            print "percent improvement over ft sensors for 0\% fixed", pi_all_move

            pp.ylabel('percent improvement \n in success rate',
                      horizontalalignment='center', labelpad=14)
            print "all fixed improvement", pi_all_fixed
            pp.xlabel('Total number of cylinders', labelpad=2)
            pp.xlim((10,210))
            pp.ylim((-5,60))
            pp.yticks([0, 10, 20, 30, 40, 50, 60])
            #mpu.legend(display_mode='less_space', draw_frame=False)
            mpu.legend(draw_frame=False)
            mpu.reduce_figure_margins(left=0.205, bottom=0.13, right=0.97, top=0.90)
            pp.title('Tactile sensing vs FT sensors')
            # params = {'legend.markerscale': 0.01}
            # pp.rcParams.update(params)
            pp.savefig('percent_improvement.pdf')

        pp.show()






















################################OLD FUNCTIONS THAT WE NO LONGER USE##########################
################################OLD FUNCTIONS THAT WE NO LONGER USE##########################
################################OLD FUNCTIONS THAT WE NO LONGER USE##########################


#     if high_force_count_plot_for_50_perc_fixed:
#         print "FOR 50 PERCENT FIXED"
#         path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
#         d = ut.load_pickle(path+'75_obstacle_configurations_movable080_fixed080/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
#         #d = ut.load_pickle('160_ft.pkl')
#         #max_force_ft = d['max_force_list']
#         max_force_ft = d['percentile_95_force_list']

#         d = ut.load_pickle(path+'75_obstacle_configurations_movable080_fixed080/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
#         #d = ut.load_pickle('160_skin.pkl')
#         #max_force_skin = d['max_force_list']
#         max_force_skin = d['percentile_95_force_list']

#         print '----------------------------------------------'
#         #print '160 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
#         #print '160 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
#         print '160 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
#         print '160 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
#         #print 'Max U:', len(max_force_skin) * len(max_force_ft)

#         #print '160 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
#         #print '160 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

#         plot_number_above_force_threshold(max_force_ft, max_force_skin)
#         pp.title('160 cylinders. 50\% fixed')
#         #pp.ylim((0, 0.6))
#         pp.savefig('trials_above_force_160_for_50_perc_fixed.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_ft)
#         pp.savefig('ft_histogram_160_50_perc_fixed.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_skin)
#         pp.savefig('skin_histogram_160_50_perc_fixed.pdf')


# ############################
# #         d = ut.load_pickle('120_ft.pkl')
# #         max_force_ft = d['max_force_list']
# #         #max_force_ft = d['percentile_95_force_list']

# #         d = ut.load_pickle('120_skin.pkl')
# #         max_force_skin = d['max_force_list']
# #         #max_force_skin = d['percentile_95_force_list']

# #         print '----------------------------------------------'
# #         print '120 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
# #         print '120 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)

# #         pp.figure()
# #         plot_force_histogram(max_force_ft)
# #         pp.savefig('ft_histogram_120.pdf')

# #         pp.figure()
# #         plot_force_histogram(max_force_skin)
# #         pp.savefig('skin_histogram_120.pdf')

# # ############################

#         d = ut.load_pickle(path+'75_obstacle_configurations_movable040_fixed040/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
#         # d = ut.load_pickle('80_ft.pkl')
#         #max_force_ft = d['max_force_list']
#         max_force_ft = d['percentile_95_force_list']

#         d = ut.load_pickle(path+'75_obstacle_configurations_movable040_fixed040/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
#         #d = ut.load_pickle('80_skin.pkl')
#         #max_force_skin = d['max_force_list']
#         max_force_skin = d['percentile_95_force_list']

#         print '----------------------------------------------'
#         #print '80 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
#         #print '80 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
#         print '80 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
#         print '80 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
#         #print 'Max U:', len(max_force_skin) * len(max_force_ft)

#         #print '80 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
#         #print '80 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

#         plot_number_above_force_threshold(max_force_ft, max_force_skin)
#         pp.title('80 cylinders. 50\% fixed')
#         #pp.ylim((0, 0.2))
#         pp.savefig('trials_above_force_80_for_50_perc_fixed.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_ft)
#         pp.savefig('ft_histogram_80_50_perc_fixed.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_skin)
#         pp.savefig('skin_histogram_80_50_perc_fixed.pdf')

# # ###########################
# #         d = ut.load_pickle('40_ft.pkl')
# #         max_force_ft = d['max_force_list']
# #         #max_force_ft = d['percentile_95_force_list']

# #         d = ut.load_pickle('40_skin.pkl')
# #         max_force_skin = d['max_force_list']
# #         #max_force_skin = d['percentile_95_force_list']

# #         print '----------------------------------------------'
# #         print '40 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
# #         print '40 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
# # ############################

# #         pp.figure()
# #         plot_force_histogram(max_force_ft)
# #         pp.savefig('ft_histogram_40.pdf')

# #         pp.figure()
# #         plot_force_histogram(max_force_skin)
# #         pp.savefig('skin_histogram_40.pdf')

#         pp.show()

#     if high_force_count_plot_for_0_perc_fixed:
#         print "FOR 0 PERCENT FIXED"
#         path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
#         d = ut.load_pickle(path+'75_obstacle_configurations_movable160_fixed000/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
#         #d = ut.load_pickle('160_ft.pkl')
#         #max_force_ft = d['max_force_list']
#         max_force_ft = d['percentile_95_force_list']

#         d = ut.load_pickle(path+'75_obstacle_configurations_movable160_fixed000/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
#         #d = ut.load_pickle('160_skin.pkl')
#         #max_force_skin = d['max_force_list']
#         max_force_skin = d['percentile_95_force_list']

#         print '----------------------------------------------'
#         #print '160 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
#         #print '160 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
#         print '160 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
#         print '160 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
#         #print 'Max U:', len(max_force_skin) * len(max_force_ft)

#         #print '160 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
#         #print '160 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

#         try:
#             plot_number_above_force_threshold(max_force_ft, max_force_skin)
#             pp.title('160 cylinders. 0\% fixed')
#             #pp.ylim((0, 0.6))
#             pp.savefig('trials_above_force_160_for_0_perc_fixed.pdf')

#             pp.figure()
#             plot_force_histogram(max_force_ft)
#             pp.savefig('ft_histogram_160_0_perc_fixed.pdf')

#             pp.figure()
#             plot_force_histogram(max_force_skin)
#             pp.savefig('skin_histogram_160_0_perc_fixed.pdf')
#         except:
#             print "No trials with forces above threshold"

#         d = ut.load_pickle(path+'75_obstacle_configurations_movable080_fixed000/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
#         # d = ut.load_pickle('80_ft.pkl')
#         #max_force_ft = d['max_force_list']
#         max_force_ft = d['percentile_95_force_list']

#         d = ut.load_pickle(path+'75_obstacle_configurations_movable080_fixed000/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
#         #d = ut.load_pickle('80_skin.pkl')
#         #max_force_skin = d['max_force_list']
#         max_force_skin = d['percentile_95_force_list']

#         print '----------------------------------------------'
#         #print '80 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
#         #print '80 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
#         print '80 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
#         print '80 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
#         #print 'Max U:', len(max_force_skin) * len(max_force_ft)

#         #print '80 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
#         #print '80 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

#         try:
#             plot_number_above_force_threshold(max_force_ft, max_force_skin)
#             pp.title('80 cylinders. 0\% fixed')
#             #pp.ylim((0, 0.2))
#             pp.savefig('trials_above_force_80_for_0_perc_fixed.pdf')

#             pp.figure()
#             plot_force_histogram(max_force_ft)
#             pp.savefig('ft_histogram_80_0_perc_fixed.pdf')

#             pp.figure()
#             plot_force_histogram(max_force_skin)
#             pp.savefig('skin_histogram_80_0_perc_fixed.pdf')
#         except:
#             print "No trials with forces above threshold"



#         pp.show()

#     if high_force_count_plot_for_100_perc_fixed:
#         print "FOR 100 PERCENT FIXED"
#         path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
#         d = ut.load_pickle(path+'75_obstacle_configurations_movable000_fixed160/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
#         #d = ut.load_pickle('160_ft.pkl')
#         #max_force_ft = d['max_force_list']
#         max_force_ft = d['percentile_95_force_list']

#         d = ut.load_pickle(path+'75_obstacle_configurations_movable000_fixed160/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
#         #d = ut.load_pickle('160_skin.pkl')
#         #max_force_skin = d['max_force_list']
#         max_force_skin = d['percentile_95_force_list']

#         print '----------------------------------------------'
#         #print '160 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
#         #print '160 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
#         print '160 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
#         print '160 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
#         #print 'Max U:', len(max_force_skin) * len(max_force_ft)

#         #print '160 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
#         #print '160 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

#         plot_number_above_force_threshold(max_force_ft, max_force_skin)
#         pp.title('160 cylinders. 100\% fixed')
#         #pp.ylim((0, 0.6))
#         pp.savefig('trials_above_force_160_for_100_perc_fixed.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_ft)
#         pp.savefig('ft_histogram_160_100_perc_fixed.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_skin)
#         pp.savefig('skin_histogram_160_100_perc_fixed.pdf')

#         d = ut.load_pickle(path+'75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_ft_sensor/reach_in_force_statistics.pkl')
#         # d = ut.load_pickle('80_ft.pkl')
#         #max_force_ft = d['max_force_list']
#         max_force_ft = d['percentile_95_force_list']

#         d = ut.load_pickle(path+'75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_skin/reach_in_force_statistics.pkl')
#         #d = ut.load_pickle('80_skin.pkl')
#         #max_force_skin = d['max_force_list']
#         max_force_skin = d['percentile_95_force_list']

#         print '----------------------------------------------'
#         #print '80 cylinders (FT) shapiro:', ss.shapiro(max_force_ft)
#         #print '80 cylinders (skin) shapiro:', ss.shapiro(max_force_skin)
#         print '80 cylinders (ttest_ind):', ss.ttest_ind(max_force_ft, max_force_skin)
#         print '80 cylinders (mann whitneyu):', ss.mannwhitneyu(max_force_ft, max_force_skin)
#         #print 'Max U:', len(max_force_skin) * len(max_force_ft)

#         #print '80 cylinders (ttest_rel):', ss.ttest_rel(max_force_ft, max_force_skin)
#         #print '80 cylinders (wilcoxon):', ss.wilcoxon(max_force_ft, max_force_skin)

#         plot_number_above_force_threshold(max_force_ft, max_force_skin)
#         pp.title('80 cylinders. 100\% fixed')
#         #pp.ylim((0, 0.2))
#         pp.savefig('trials_above_force_80_for_100_perc_fixed.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_ft)
#         pp.savefig('ft_histogram_80_100_perc_fixed.pdf')

#         pp.figure()
#         plot_force_histogram(max_force_skin)
#         pp.savefig('skin_histogram_80_100_perc_fixed.pdf')

#         pp.show()






        





    # if regulate_forces_plot:
    #     #path = base_path+'/hrl_file_server/darpa_m3/marc_rerun_all_trials_ijrr_revision_2_data/'
    #     path = '/home/advait/nfs/darpa_m3_logs/advait/25_obstacle_configurations_may_26_movable20_fixed20'
    #     pkl_nm = 'reach_in_force_statistics.pkl'
    #     trial_nm_l = ['mpc_qs_1_20120526_using_skin_5N',
    #                   'mpc_qs_1_20120526_using_skin_10N',
    #                   'mpc_qs_1_20120526_using_skin_15N',
    #                   'mpc_qs_1_20120526_using_skin_20N',
    #                   'mpc_qs_1_20120526_using_skin_25N']
        
    #     #for trial_nm in trial_nm_l:
    #     #    d = ut.load_pickle(os.path.join(path, trial_nm, pkl_nm))
    #     #    all_forces = d['all_forces_mag_list']
    #     #    print '-----------------------'
    #     #    print 'trial_nm:', trial_nm
    #     #    print '-----------------------'
    #     #    #print 'Median force:', d['median_contact_force']
    #     #    #print 'First quartile contact force:', d['first_quartile_contact_force']
    #     #    #print 'Third quartile contact force:', d['third_quartile_contact_force']
    #     #    print '75 percentile force:', ss.scoreatpercentile(all_forces, 75)
    #     #    print '90 percentile force:', ss.scoreatpercentile(all_forces, 90)
    #     #    print '95 percentile force:', ss.scoreatpercentile(all_forces, 95)
    #     #    print '99 percentile force:', ss.scoreatpercentile(all_forces, 99)

    #     perc_75_a = np.array([4.65661773347, 7.46382457071, 8.36974924057, 8.81591794446, 9.31266569258])
    #     perc_90_a = np.array([4.92841734977, 9.85172087923, 14.2601314264, 14.8729333516, 16.5725414726])
    #     perc_95_a = np.array([4.97980083, 9.94200627435, 14.8729220772, 19.5636655556, 23.9172521823])
    #     perc_99_a = np.array([5.54570756397, 10.0423941129, 15.0507787333, 20.0175772605, 25.0097461856])

    #     f_thresh_l = [5, 10, 15, 20, 25]
        
    #     print 'correlation:', ss.pearsonr(perc_95_a, f_thresh_l)

    #     mpu.set_figure_size(8,8)
    #     pp.figure()
    #     yerr = np.row_stack([perc_95_a-perc_75_a, perc_99_a-perc_95_a])
    #     pp.errorbar(f_thresh_l, perc_95_a, yerr, linewidth=0,
    #                 marker='o', ms=5, elinewidth=1, capsize=10)
    #     pp.plot([0,30], [0,30], 'r-')
    #     pp.axis('equal')
    #     #pp.xlim(0,30)
    #     pp.ylim(0,30)
    #     pp.ylabel('Contact force magnitude (N)')
    #     pp.xlabel('Don\'t care force threshold (N)')
    #     mpu.reduce_figure_margins(left=0.15, bottom=0.15, right=0.98, top=0.98)
    #     pp.show()









        ############this was looking for trends in max forces as clutter increased####################
        ############this was looking for trends in max forces as clutter increased####################
        ############this was looking for trends in max forces as clutter increased####################
        # x_values = [5, 6, 7, 10, 15]
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # for only thresholds of 5 and some ...
        # x_values : [5, 6, 7, 10, 15]
        # perc_skin_50 :
        # [13.152016570404795, 2.4868944233961869, 1.6648494141656009, 0.7072536691845327, 0.25496171785739069]
        # perc_ft_50 :
        # [15.205903020971187, 5.36684379563847, 3.0464240283462969, 0.90815380633646037, 0.2849722282641946]
        # perc_none_50 :
        # [51.246747803280634, 49.440747381257452, 47.752007575807504, 42.888903294500892, 35.438420621955771]
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        # # perc_skin_50, perc_ft_50, perc_none_50,  skin_total_50, ft_total_50, none_total_50 = get_percentages_above_thresh(direc, nm_50, x_values)
        # # print "for only thresholds of 5 and some ..."
        # # print "x_values :", x_values
        # # print "perc_skin_50 :\n", perc_skin_50
        # # print "perc_ft_50 :\n", perc_ft_50
        # # print "perc_none_50 :\n", perc_none_50
        # # print "p-values comparing skin to ft:\n", statistical_analysis_success_percent(perc_skin_50, perc_ft_50, skin_total_50, ft_total_50)[2], "\n"
        # # print "p-values comparing skin to none:\n", statistical_analysis_success_percent(perc_skin_50, perc_none_50, skin_total_50, none_total_50)[2], "\n"


        # print "for 20:"
        # perc_skin_50, perc_ft_50, perc_none_50,  skin_total_50, ft_total_50, none_total_50 = get_percentages_above_thresh(direc,['75_obstacle_configurations_movable000_fixed020'], x_values)
        # print "for only thresholds of 5 and some ..."
        # print "x_values :", x_values
        # print "perc_skin_50 :\n", perc_skin_50
        # print "perc_ft_50 :\n", perc_ft_50
        # print "perc_none_50 :\n", perc_none_50
        # print "p-values comparing skin to ft:\n", statistical_analysis_success_percent(perc_skin_50, perc_ft_50, skin_total_50, ft_total_50)[2], "\n"
        # print "p-values comparing skin to none:\n", statistical_analysis_success_percent(perc_skin_50, perc_none_50, skin_total_50, none_total_50)[2], "\n"


        # print "for 40:"
        # perc_skin_50, perc_ft_50, perc_none_50,  skin_total_50, ft_total_50, none_total_50 = get_percentages_above_thresh(direc,['75_obstacle_configurations_movable000_fixed040'], x_values)
        # print "for only thresholds of 5 and some ..."
        # print "x_values :", x_values
        # print "perc_skin_50 :\n", perc_skin_50
        # print "perc_ft_50 :\n", perc_ft_50
        # print "perc_none_50 :\n", perc_none_50
        # print "p-values comparing skin to ft:\n", statistical_analysis_success_percent(perc_skin_50, perc_ft_50, skin_total_50, ft_total_50)[2], "\n"
        # print "p-values comparing skin to none:\n", statistical_analysis_success_percent(perc_skin_50, perc_none_50, skin_total_50, none_total_50)[2], "\n"


        # print "for 80:"
        # perc_skin_50, perc_ft_50, perc_none_50,  skin_total_50, ft_total_50, none_total_50 = get_percentages_above_thresh(direc,['75_obstacle_configurations_movable000_fixed080'], x_values)
        # print "for only thresholds of 5 and some ..."
        # print "x_values :", x_values
        # print "perc_skin_50 :\n", perc_skin_50
        # print "perc_ft_50 :\n", perc_ft_50
        # print "perc_none_50 :\n", perc_none_50
        # print "p-values comparing skin to ft:\n", statistical_analysis_success_percent(perc_skin_50, perc_ft_50, skin_total_50, ft_total_50)[2], "\n"
        # print "p-values comparing skin to none:\n", statistical_analysis_success_percent(perc_skin_50, perc_none_50, skin_total_50, none_total_50)[2], "\n"


        # print "for 120:"
        # perc_skin_50, perc_ft_50, perc_none_50,  skin_total_50, ft_total_50, none_total_50 = get_percentages_above_thresh(direc,['75_obstacle_configurations_movable000_fixed120'], x_values)
        # print "for only thresholds of 5 and some ..."
        # print "x_values :", x_values
        # print "perc_skin_50 :\n", perc_skin_50
        # print "perc_ft_50 :\n", perc_ft_50
        # print "perc_none_50 :\n", perc_none_50
        # print "p-values comparing skin to ft:\n", statistical_analysis_success_percent(perc_skin_50, perc_ft_50, skin_total_50, ft_total_50)[2], "\n"
        # print "p-values comparing skin to none:\n", statistical_analysis_success_percent(perc_skin_50, perc_none_50, skin_total_50, none_total_50)[2], "\n"


        # print "for 160:"
        # perc_skin_50, perc_ft_50, perc_none_50,  skin_total_50, ft_total_50, none_total_50 = get_percentages_above_thresh(direc,['75_obstacle_configurations_movable000_fixed160'], x_values)
        # print "for only thresholds of 5 and some ..."
        # print "x_values :", x_values
        # print "perc_skin_50 :\n", perc_skin_50
        # print "perc_ft_50 :\n", perc_ft_50
        # print "perc_none_50 :\n", perc_none_50
        # print "p-values comparing skin to ft:\n", statistical_analysis_success_percent(perc_skin_50, perc_ft_50, skin_total_50, ft_total_50)[2], "\n"
        # print "p-values comparing skin to none:\n", statistical_analysis_success_percent(perc_skin_50, perc_none_50, skin_total_50, none_total_50)[2], "\n"


        # print "for 200:"
        # perc_skin_50, perc_ft_50, perc_none_50,  skin_total_50, ft_total_50, none_total_50 = get_percentages_above_thresh(direc,['75_obstacle_configurations_movable000_fixed200'], x_values)
        # print "for only thresholds of 5 and some ..."
        # print "x_values :", x_values
        # print "perc_skin_50 :\n", perc_skin_50
        # print "perc_ft_50 :\n", perc_ft_50
        # print "perc_none_50 :\n", perc_none_50
        # print "p-values comparing skin to ft:\n", statistical_analysis_success_percent(perc_skin_50, perc_ft_50, skin_total_50, ft_total_50)[2], "\n"
        # print "p-values comparing skin to none:\n", statistical_analysis_success_percent(perc_skin_50, perc_none_50, skin_total_50, none_total_50)[2], "\n"
        ############this was looking for trends in max forces as clutter increased####################
        ############this was looking for trends in max forces as clutter increased####################
        ############this was looking for trends in max forces as clutter increased####################
