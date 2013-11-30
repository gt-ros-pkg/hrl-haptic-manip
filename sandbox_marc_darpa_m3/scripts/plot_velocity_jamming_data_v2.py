import cPickle as pkl
import copy
import numpy as np
import matplotlib.pyplot as pl
import roslib
roslib.load_manifest('sandbox_marc_darpa_m3')

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu


def get_histograms(qs_data, dyn_data, ind_over, threshold, num_objects, prefix='', num_bins=30 ):
    #data = pkl.load(file(f+'/stats_on_impact_velocity_and_jamming.pkl'))
    
    vel_dyn_mag = []
    vel_qs_mag = []

    vel_dyn_mag = [np.linalg.norm(vel) for vel in dyn_data['contact_vel']]
    vel_qs_mag = [np.linalg.norm(vel) for vel in qs_data['contact_vel']]

    dyn_ind_less = np.where(np.array(dyn_data['contact_force']) < threshold)
    qs_ind_less = np.where(np.array(qs_data['contact_force']) < threshold)

    dyn_indices_over = []
    qs_indices_over = []

    for ind in ind_over:
        dyn_indices_over.append(np.where(np.array(dyn_data['contact_force']) > ind))
        qs_indices_over.append(np.where(np.array(qs_data['contact_force']) > ind))

    bins  = np.arange(num_bins)*0.01

    #TODO get max y_max and use it !!! for both plots will need to use gcf or something...

    # start here to get figures and make ylim the same for both!
    mpu.set_figure_size(14, 9)
    #mpu.reduce_figure_margins(left=0.2, bottom=0.2, top=0.85)
    fig1 = pl.figure()
    pl.hist(np.array(vel_qs_mag)[qs_ind_less], bins)
    pl.xlabel('velocity at contact point (m/s)')
    pl.ylabel('number of contacts below '+str(threshold)+' N')
    pl.title('Quasi-static Velocities for Forces Over '+str(threshold)+' Newtons')
    qs_ymin, qs_ymax = pl.ylim()

    fig2 = pl.figure()
    pl.hist(np.array(vel_dyn_mag)[dyn_ind_less], bins)
    pl.xlabel('velocity at contact point (m/s)')
    pl.ylabel('number of contacts below '+str(threshold)+' N')
    pl.title('Dynamic Velocities for Forces Over '+str(threshold)+' Newtons')
    dyn_ymin, dyn_ymax = pl.ylim()


    if dyn_ymax > qs_ymax:
        pl.figure(fig1.number)
        pl.ylim((0, dyn_ymax))
    else:
        pl.figure(fig2.number)
        pl.ylim((0, qs_ymax))
 
    pl.figure(fig1.number)
    pl.savefig('/home/mkillpack/Desktop/'+prefix+'qs_velocity_hist_at_forces_below_'+str(threshold)+'N_with_'+str(threshold)+'N_threshold_'+str(num_objects)+'_fixed.pdf')
    pl.figure(fig2.number)
    pl.savefig('/home/mkillpack/Desktop/'+prefix+'dyn_velocity_hist_at_forces_below_'+str(threshold)+'N_with_'+str(threshold)+'N_threshold_'+str(num_objects)+'_fixed.pdf')

    #pl.hist(np.array(vel_mag)[ind_less_25], bins)


    for i in xrange(len(ind_over)):
        mpu.set_figure_size(14, 9)
        fig1 = pl.figure()
        pl.hist(np.array(vel_qs_mag)[qs_indices_over[i]], bins)
        pl.title('Quasi-static Velocities for Forces Over '+str(ind_over[i])+' Newtons')
        pl.xlabel('velocity at contact point (m/s)')
        pl.ylabel('number of contacts above '+str(ind_over[i])+' N')
        qs_ymin,qs_ymax = pl.ylim()

        mpu.set_figure_size(14, 9)
        fig2 = pl.figure()
        pl.hist(np.array(vel_dyn_mag)[dyn_indices_over[i]], bins)
        pl.title('Dynamic Velocities for Forces Over '+str(ind_over[i])+' Newtons')
        pl.xlabel('velocity at contact point (m/s)')
        pl.ylabel('number of contacts above '+str(ind_over[i])+' N')
        dyn_ymin, dyn_ymax = pl.ylim()

        if dyn_ymax > qs_ymax:
            pl.figure(fig1.number)
            pl.ylim((0, dyn_ymax))
        else:
            pl.figure(fig2.number)
            pl.ylim((0, qs_ymax))


        pl.figure(fig1.number)
        pl.savefig('/home/mkillpack/Desktop/'+prefix+'qs_velocity_hist_at_forces_below_'+str(ind_over[i])+'N_with_'+str(threshold)+'N_threshold_'+str(num_objects)+'_fixed.pdf')
        pl.figure(fig2.number)
        pl.savefig('/home/mkillpack/Desktop/'+prefix+'dyn_velocity_hist_at_forces_below_'+str(ind_over[i])+'N_with_'+str(threshold)+'N_threshold_'+str(num_objects)+'_fixed.pdf')


def get_other_stats(data, thresh_list, file_name):
    vel_list = [0.025, 0.05, 0.10, 0.2]

    vel_mag = [np.linalg.norm(vel) for vel in data['contact_vel']]

    for thresh in thresh_list:
        try:
            ind = np.where(np.array(data['contact_force']) > thresh)[0]
            num_total = len(np.array(data['contact_type'])[ind])
            num_jamming = len(np.where(np.array(data['contact_type'])[ind] == 'jamming')[0])
            print >> file_name, "\n\n\n\n"
            print >> file_name, "thresh is :"+str(thresh)
            print >> file_name, "total num of contacts :"+str(num_total)
            print >> file_name, "Perc of jammming :"+str(num_jamming/float(num_total))
            print >> file_name, "num of jamming :"+str(num_jamming)

            ind_other = np.where(np.array(data['contact_type'])[ind] == "other")[0].tolist()
            for vel in vel_list:
                vel_ind = list(set(ind).intersection(np.where(np.array(data['contact_vel']) > vel)[0].tolist()))
                print >> file_name, "Perc of contacts with velocity above "+str(vel)+" is:"+str(len(vel_ind)/float(num_total))
                print >> file_name, "num of contacts with velocity above "+str(vel)+" is:"+str(len(vel_ind))
            
        except ZeroDivisionError:
            print >> file_name, "NO CONTACT FOR THIS THRESHOLD"
        

if __name__ == '__main__':
    force_thresh = 25
    num_obst = 80

    if force_thresh == 5:
        import analyze_multiple_trial_results_5N as amtr
        ind_over = [5, 10, 20, 30, 40, 50]
        thresh_list = [6, 10, 20, 30, 40, 50]
        qs_obj = amtr.ComputeResultStatistics(threshold = 5, contact_vel_save = False)
    else:
        import analyze_multiple_trial_results_25N as amtr
        ind_over = [25, 30, 35, 40, 45, 50]
        thresh_list = [30, 35, 40, 45, 50]
        qs_obj = amtr.ComputeResultStatistics(log_name = '/reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl', threshold = 25, contact_vel_save = False)

    if force_thresh == 5 and num_obst == 20:
        path_qs = '/home/mkillpack/hrl_file_server/darpa_m3/final_ijrr_results_with_10_second_timeout/75_obstacle_configurations_movable000_fixed020/mpc_qs_1_using_skin/'
        path_dyn = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_low_force_low_clutter_new_delta_t_impulse_sim_anneal_search_set2_mu_15_t_impulse_0.7/'
    elif force_thresh == 5 and num_obst == 80:
        path_qs = '/home/mkillpack/hrl_file_server/darpa_m3/final_ijrr_results_with_10_second_timeout/75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_skin/'
        path_dyn = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_low_force_high_clutter_new_delta_t_impulse_sim_anneal_search_set2_mu_15_t_impulse_0.7/'
    elif force_thresh == 25 and num_obst == 20:
        path_dyn = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_high_force_low_clutter_new_delta_t_impulse_sim_anneal_search_set2_mu_15/'
        path_qs = '/home/mkillpack/hrl_file_server/darpa_m3/final_ijrr_results_with_10_second_timeout/75_obstacle_configurations_movable000_fixed020_25N_thresh/mpc_qs_1_using_skin/'
    elif force_thresh == 25 and num_obst == 80:
        path_dyn = '/home/mkillpack/hrl_file_server/darpa_m3/dyn_vs_qs_comparison/dyn_high_force_high_clutter_new_delta_t_impulse_sim_anneal_search_set2_mu_15/'
        path_qs = '/home/mkillpack/hrl_file_server/darpa_m3/final_ijrr_results_with_10_second_timeout/75_obstacle_configurations_movable000_fixed080_25N_thresh/mpc_qs_1_using_skin/'


    import analyze_multiple_trial_results_dyn as amtrd
    dyn_obj = amtrd.ComputeResultStatistics(contact_vel_save = False)

    fn_list_dyn = []
    fn_list_dyn.append(dyn_obj.get_impact_and_jamming_data)

    fn_list_qs = []
    fn_list_qs.append(qs_obj.get_impact_and_jamming_data)

    qs_obj.traverse_directory(path_qs, fn_list_qs)
    data_qs = copy.copy(qs_obj.contact_data)  
    del qs_obj

    dyn_obj.traverse_directory(path_dyn, fn_list_dyn)
    data_dyn =  copy.copy(dyn_obj.contact_data)
    del dyn_obj

    qs_file = file('./dyn_stats.txt', 'w')
    print >> qs_file, "########################################"
    print >> qs_file, "contacts stats for QS"
    print >> qs_file, "########################################"

    get_other_stats(data_qs, ind_over, qs_file)

    qs_file.close()

    dyn_file = file('./qs_stats.txt', 'w')
    print >> dyn_file, "\n\n\n\n\n########################################"
    print >> dyn_file, "contacts stats for DYN"
    print >> dyn_file, "########################################"

    get_other_stats(data_dyn, ind_over, dyn_file)

    dyn_file.close()

    import plot_contact_loc_data as pcld

    base = '/home/mkillpack/Desktop/'

    pcld.plot_stuff(data_dyn['contact_force'], data_dyn['contact_locs'],  
                    data_qs['contact_force'], data_qs['contact_locs'], 
                    base+str(force_thresh).zfill(2)+'N_movable000_fixed0'+str(num_obst), thresh_list)

    get_histograms(data_qs, data_dyn, ind_over, force_thresh, num_obst)  #can add prefix and num_bins



