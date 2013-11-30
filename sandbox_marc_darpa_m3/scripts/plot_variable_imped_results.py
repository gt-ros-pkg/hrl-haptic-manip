
import os
import matplotlib.pyplot as pl
import numpy as np, math
import scipy.stats as ss
import copy
import sys

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu
import scipy.stats as ss

base_path = '/home/mkillpack'


def compute_success_rate(path, pkl_nm, nm_list):
    l = []
    for nm in nm_list:

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

def get_success_rate(base, path, pkl_nm):
    data = ut.load_pickle(os.path.join(base, path, pkl_nm))
    return data['success_count']/float(data['success_count']+data['fail_count'])

def get_force_stats(base, path, pkl_nm, thresh):
    data = ut.load_pickle(os.path.join(base, path, pkl_nm))
    force_99th = ss.scoreatpercentile(data['all_forces_mag_list'], 99)
    median_of_max = np.median(data['max_force_list'])
    perc_above_thresh = float(len(np.where(np.array(data['all_forces_mag_list']) > float(thresh))[0]))/float(len(data['all_forces_mag_list']))

    return force_99th, median_of_max, perc_above_thresh

def compare_average_velocity(path, pkl_nm, qs_nm_list, dyn_nm_list):
    dist_l_dyn = []
    time_l_dyn = []
    dist_l_qs = []
    time_l_qs = []
    exec_t_dyn = []
    exec_t_qs = []
    exec_t_diff = []

    if True:
        qs_nm = qs_nm_list
        dyn_nm = dyn_nm_list
    #for qs_nm, dyn_nm in zip(qs_nm_list, dyn_nm_list):
        print "os.path.join(path, dyn_nm, pkl_nm :",  os.path.join(path, dyn_nm, pkl_nm)
        print "os.path.join(path, qs_nm, pkl_nm :",  os.path.join(path, qs_nm, pkl_nm)
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

    return np.mean(exec_t_qs), np.mean(exec_t_dyn), np.mean(exec_t_diff), p_value  #total_dist_qs / total_time_qs, total_dist_dyn / total_time_dyn

if __name__ == '__main__':
    base = '/home/mkillpack/hrl_file_server/darpa_m3/varying_impedance_across_trials/'

    thresh = '05'
    impulse_time = '0.04'

    alpha_list = [0.1, 0.33, 0.5, 0.66, 1.00, 2.00, 3.00, 4.00, 5.00, 6.00]

    alpha_list_str = ['0.10', '0.33', '0.50', '0.66', '1.00', '2.00', '3.00', '4.00', '5.00', '6.00']

    dyn_list = ['dynamic_imped_scale_'+alpha+'_fixed_080_movable_000_'+thresh+'N_threshold_with_damping_delta_t_impulse_'+impulse_time+'/' for alpha in alpha_list_str]
    qs_list = ['qs_imped_scale_'+alpha+'_fixed_080_movable_000_'+thresh+'N_threshold/' for alpha in alpha_list_str]

    dyn_list[4] = 'dynamic_imped_scale_1.00_fixed_080_movable_000_'+thresh+'N_threshold_delta_t_impulse_'+impulse_time+'/'

    vel_pkl_nm = 'reach_in_kinematics_statistics.pkl'
    force_pkl_nm = 'reach_in_force_statistics.pkl'

    qs_time_to_complete_ls = []
    dyn_time_to_complete_ls = []
    qs_99_force = []
    dyn_99_force = []
    qs_perc_above_thresh = []
    dyn_perc_above_thresh = []
    qs_med_max = []
    dyn_med_max = []
    qs_success = []
    dyn_success = []

    for i in xrange(len(dyn_list)):
        qs_time, dyn_time, diff_time, p_value = compare_average_velocity(base, vel_pkl_nm, qs_list[i], dyn_list[i])
        qs_time_to_complete_ls.append(qs_time)
        dyn_time_to_complete_ls.append(dyn_time)
        
        force_99th, median_of_max, perc_above_thresh = get_force_stats(base, qs_list[i], force_pkl_nm, thresh)
        qs_99_force.append(force_99th)
        qs_perc_above_thresh.append(perc_above_thresh)
        qs_med_max.append(median_of_max)

        force_99th, median_of_max, perc_above_thresh = get_force_stats(base, dyn_list[i], force_pkl_nm,  thresh)
        dyn_99_force.append(force_99th)
        dyn_perc_above_thresh.append(perc_above_thresh)
        dyn_med_max.append(median_of_max)

        qs_success.append(get_success_rate(base, qs_list[i], 'combined_results.pkl'))
        dyn_success.append(get_success_rate(base, dyn_list[i], 'combined_results.pkl'))
        

    print "qs time is :", qs_time_to_complete_ls
    print "dyn time is :", dyn_time_to_complete_ls

    mpu.set_figure_size(14, 9)
    pl.figure()
    pl.plot(alpha_list, qs_time_to_complete_ls, 'b-.s', mew=0, label='MPC with Quasi-static Model')
    pl.plot(alpha_list, dyn_time_to_complete_ls, 'g:D', mew=0, label='MPC with Dynamic Model')
    pl.ylabel('average time to complete (s)')
    pl.xlabel(r'$\alpha$ (ratio of original stiffness)', labelpad=2)
    pl.title('Time to Complete as Stiffness Varies')
    pl.savefig('var_imped_time_to_complete_'+thresh+'N_fixed080.pdf')
    # pp.ylim((-5,105))
    # pp.xlim((10,210))
    mpu.legend(display_mode='less_space', draw_frame=False)



    pl.figure()
    pl.plot(alpha_list, qs_success, 'b-.s', mew=0, label='MPC with Quasi-static Model')
    pl.plot(alpha_list, dyn_success, 'g:D', mew=0, label='MPC with Dynamic Model')
    pl.ylabel('success rate (\%)')
    pl.xlabel(r'$\alpha$ (ratio of original stiffness)', labelpad=2)
    pl.title('Success Rate as Stiffness Varies')
    # pp.ylim((-5,105))
    # pp.xlim((10,210))
    pl.savefig('var_imped_succes_rate_'+thresh+'N_fixed080.pdf')
    mpu.legend(display_mode='less_space', draw_frame=False)


    pl.figure()
    pl.plot(alpha_list, qs_99_force, 'b-.s', mew=0, label='MPC with Quasi-static Model')
    pl.plot(alpha_list, dyn_99_force, 'g:D', mew=0, label='MPC with Dynamic Model')
    pl.ylabel('99th percentile contact force (N)')
    pl.xlabel(r'$\alpha$ (ratio of original stiffness)', labelpad=2)
    pl.title('99th Percentile Contact Force as Stiffness Varies')
    pl.savefig('var_imped_99_percentile_'+thresh+'N_fixed080.pdf')
    # pp.ylim((-5,105))
    # pp.xlim((10,210))
    mpu.legend(display_mode='less_space', draw_frame=False)



    pl.figure()
    pl.plot(alpha_list, qs_perc_above_thresh, 'b-.s', mew=0, label='MPC with Quasi-static Model')
    pl.plot(alpha_list, dyn_perc_above_thresh, 'g:D', mew=0, label='MPC with Dynamic Model')
    pl.ylabel('\% of all contact forces above threshold')
    pl.xlabel(r'$\alpha$ (ratio of original stiffness)', labelpad=2)
    pl.title(r'Percent of Forces Above $f_{threshold}$ as Stiffness Varies')
    pl.savefig('var_imped_perc_above_'+thresh+'N_fixed080.pdf')
    # pp.ylim((-5,105))
    # pp.xlim((10,210))
    mpu.legend(display_mode='less_space', draw_frame=False)

    pl.figure()
    pl.plot(alpha_list, qs_med_max, 'b-.s', mew=0, label='MPC with Quasi-static Model')
    pl.plot(alpha_list, dyn_med_max, 'g:D', mew=0, label='MPC with Dynamic Model')
    pl.ylabel('median of max contact force (N)')
    pl.xlabel(r'$\alpha$ (ratio of original stiffness)', labelpad=2)
    pl.title('Median of the Max Forces per Trial as Stiffness Varies')
    pl.savefig('var_imped_median_of_max_forces_'+thresh+'N_fixed080.pdf')
    # pp.ylim((-5,105))
    # pp.xlim((10,210))
    mpu.legend(display_mode='less_space', draw_frame=False)

    pl.show()
