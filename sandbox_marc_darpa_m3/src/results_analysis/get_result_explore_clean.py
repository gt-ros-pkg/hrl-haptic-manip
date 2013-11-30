
import numpy as np, math
import sys, os

import roslib; roslib.load_manifest('sandbox_marc_darpa_m3')
#import analyze_results_clean as arc

#import software_simulation.ode_sim_arms as osa
import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu
import matplotlib.pyplot as pp
from scipy import *
from scipy import signal as signal

#used to plot
from numpy import *
import pylab as pl
#import matplotlib.axes3d as p3
import mpl_toolkits.mplot3d.axes3d as p3
import scipy.stats as ss



class ComputeResultStatistics():
    def __init__(self, controller_list):
        self.root_path = None
        self.controller_list = controller_list

    def traverse_directory(self, root_path, fn_list):
        self.root_path = root_path

        for fn in fn_list:
            fn('start')

        for root, dirs, files in os.walk(root_path):
            for d in dirs:
                full_path = os.path.join(root, d)
                print full_path
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
            ut.save_pickle(d, self.root_path + '/combined_success_results.pkl')
            return

        full_path = full_path.rstrip('/')

        res_dict = ut.load_pickle(full_path+'/'+self.controller_list[0]+'_controller.pkl')
        trial = full_path.split('/')[-1]

        if res_dict is None:
            self.not_completed_trial_list.append(trial)
            self.fail_count += 1
        elif res_dict['result']== "Reached":
            self.success_count += 1
            self.success_trials_list.append(trial)
        else:
            self.fail_count += 1

    # # what would the success rate for a single reach have been if the
    # # stopping force were different.
    # def success_rate_for_different_stopping_force(self, full_path):
    #     stopping_force_list = [5, 10, 15, 20, 25, 30, 40, 50, 100]

    #     if full_path == 'start':
    #         self.success_count_list = [0 for f in stopping_force_list]
    #         self.fail_count_list = [0 for f in stopping_force_list]
    #         return

    #     if full_path == 'end':
    #         d = {}
    #         for i in range(len(stopping_force_list)):
    #             print '==== Stopping force :%.1fN ======'%stopping_force_list[i]
    #             print self.success_count_list[i], 'successful trials'
    #             print self.fail_count_list[i], 'failed trials'
    #             print self.success_count_list[i] * 1. / (self.success_count_list[i]+self.fail_count_list[i]), '% success rate.'
    #             print self.success_count_list[i] + self.fail_count_list[i], 'total trials'
    #             d[i] = {}
    #             d[i]['force'] = stopping_force_list[i]
    #             d[i]['success_count'] = self.success_count_list[i]
    #             d[i]['fail_count'] = self.fail_count_list[i]
    #         ut.save_pickle(d, self.root_path + '/success_rate_diff_stopping_force.pkl')
    #         return

    #     param_pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*param*.pkl')
    #     param_pkl_list.reverse()
    #     pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
    #     pkl_list.reverse()

    #     if pkl_list != []:
    #         pd = ut.load_pickle(param_pkl_list[0])
    #         stopping_dist_to_goal = pd['stopping_dist_to_goal'] + 0.002

    #         d = ut.load_pickle(pkl_list[0])
    #         max_f_l = d['max_force_list']
    #         max_f_arr = np.array(max_f_l)
    #         goal = d['local_goal']

    #         for i,f in enumerate(stopping_force_list):
    #             idx_arr = np.where(max_f_arr > f)[0]
    #             if len(idx_arr) == 0:
    #                 idx = -1
    #             else:
    #                 idx = idx_arr[0]

    #             try:
    #                 ee = np.matrix(d['ee_pos_list'][idx]).T
    #                 dist = np.linalg.norm(ee - goal)
    #             except IndexError:
    #                 print '-----------------------------------'
    #                 print full_path
    #                 print 'idx:', idx
    #                 print 'len(d[\'ee_pos_list\']):', len(d['ee_pos_list'])
    #                 print 'len(d[\'max_force_list\']):', len(d['max_force_list'])
    #                 dist = 200.

    #             if dist <= stopping_dist_to_goal:
    #                 self.success_count_list[i] += 1
    #             else:
    #                 self.fail_count_list[i] += 1

    def compute_reach_in_force_statistics(self, full_path):
        if full_path == 'start':
            self.single_reach_percentile_99_force_list = []
            self.single_reach_percentile_95_force_list = []
            self.single_reach_percentile_90_force_list = []
    #         self.single_reach_percentile_75_force_list = []
            self.single_reach_max_force_list = []
            self.single_reach_avg_force_list = []
            self.single_reach_std_force_list = []
            self.single_reach_med_force_list = []
            self.single_reach_force_count_list_for_avg = []

    #         self.percentile_99_force_list = []
    #         self.percentile_95_force_list = []
    #         self.percentile_90_force_list = []
    #         self.percentile_75_force_list = []
    #         self.max_force_list = []
    #         self.avg_force_list = []
    #         self.force_count_list_for_avg = []
            return

        if full_path == 'end':
            # def compute_overall_avg(avg_list, count_list):
            #     return np.sum(np.multiply(avg_list, count_list)) / np.sum(count_list)

            # avg_max_force = np.mean(self.max_force_list)
            # avg_contact_force = compute_overall_avg(self.avg_force_list, self.force_count_list_for_avg)

            single_reach_avg_max_force = np.mean(self.single_reach_max_force_list)
            single_reach_avg_contact_force = np.mean(self.single_reach_avg_force_list)
            single_reach_median_contact_force = np.mean(self.single_reach_med_force_list)

            print ''
            print 'Avg. max force (Single Reach):', single_reach_avg_max_force
            print 'Avg. contact force (Single Reach):', single_reach_avg_contact_force
            print 'Avg. median force (Single Reach):', single_reach_median_contact_force
            print ''
            print ''

            d = {}
            d['avg_max_force_single_reach'] = single_reach_avg_max_force
            d['avg_contact_force_single_reach'] = single_reach_avg_contact_force
            d['med_contact_force_single_reach'] = single_reach_median_contact_force
            d['max_force_list_single_reach'] = self.single_reach_max_force_list
            d['99_of_99_forces_single_reach'] = ss.scoreatpercentile(self.single_reach_percentile_99_force_list, 99)
            d['95_of_95_forces_single_reach'] = ss.scoreatpercentile(self.single_reach_percentile_95_force_list, 95)
            d['90_of_90_forces_single_reach'] = ss.scoreatpercentile(self.single_reach_percentile_90_force_list, 90)
            d['single_reach_percentile_99_force_list'] = self.single_reach_percentile_99_force_list
            d['single_reach_percentile_95_force_list'] = self.single_reach_percentile_95_force_list
            d['single_reach_percentile_90_force_list'] = self.single_reach_percentile_90_force_list
            d['max_force_list'] = self.single_reach_max_force_list
            d['avg_force_list'] = self.single_reach_avg_force_list
            d['med_force_list'] = self.single_reach_med_force_list
            d['std_force_list'] = self.single_reach_std_force_list

            ut.save_pickle(d, self.root_path + '/single_reach_in_force_statistics.pkl')
            return

        #pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
        #pkl_list.reverse()

    #     # ONLY FOR SINGLE REACH. Advait felt like the ls -t was
    #     # making nfs real slow.
        pkl = full_path+'/'+self.controller_list[0]+'_logger.pkl'
        pkl_list = [pkl]

        for i, pkl in enumerate(pkl_list):
            d = ut.load_pickle(pkl)
            if d == None:
                continue

            f_mag_l = []
            for force in d['forces']:
                f_mag_l.append(np.linalg.norm(force))
            if len(f_mag_l) == 0.:
                continue

            if i == 0:
                self.single_reach_max_force_list.append(d['max_force'])
                self.single_reach_avg_force_list.append(d['avg_force'])
                self.single_reach_med_force_list.append(d['median_force'])
                self.single_reach_std_force_list.append(d['std_force'])

                self.single_reach_percentile_99_force_list.append(ss.scoreatpercentile(f_mag_l, 99))
                self.single_reach_percentile_95_force_list.append(ss.scoreatpercentile(f_mag_l, 95))
                self.single_reach_percentile_90_force_list.append(ss.scoreatpercentile(f_mag_l, 90))
    #             self.single_reach_percentile_75_force_list.append(ss.scoreatpercentile(f_mag_l, 75))
    #             self.single_reach_max_force_list.append(max(f_mag_l))
    #             self.single_reach_avg_force_list.append(np.mean(f_mag_l))
    #             self.single_reach_force_count_list_for_avg.append(len(f_mag_l))

    #         self.percentile_99_force_list.append(ss.scoreatpercentile(f_mag_l, 99))
    #         self.percentile_95_force_list.append(ss.scoreatpercentile(f_mag_l, 95))
    #         self.percentile_90_force_list.append(ss.scoreatpercentile(f_mag_l, 90))
    #         self.percentile_75_force_list.append(ss.scoreatpercentile(f_mag_l, 75))
    #         self.max_force_list.append(max(f_mag_l))
    #         self.avg_force_list.append(np.mean(f_mag_l))
    #         self.force_count_list_for_avg.append(len(f_mag_l))

    def compute_kinematic_statistics(self, full_path):
        if full_path == 'start':
            self.ee_distance = []
            self.execution_time = []
            self.avg_velocity = []
            self.min_distance_to_goal = []
            self.max_velocity_list = []
            return

        if full_path == 'end':
            print ''
            print 'Avg. velocity:', np.mean(self.avg_velocity)
            print 'Avg. distance:', np.mean(self.ee_distance)
            print 'Avg. execution time:', np.mean(self.execution_time)
            print ''

            d = {}
            d['avg_velocity_list'] = self.avg_velocity
            d['execution_time_list'] = self.execution_time
            d['ee_distance_list'] = self.ee_distance
            d['min_distance_to_goal_list'] = self.min_distance_to_goal
            d['max_velocity_list'] = self.max_velocity_list
            ut.save_pickle(d, self.root_path + '/reach_in_kinematics_statistics.pkl')
            return

        pkl = full_path+'/'+self.controller_list[0]+'_logger.pkl'
        pkl_list = [pkl]

        for i, pkl in enumerate(pkl_list):
            d = ut.load_pickle(pkl)
            if d == None:
                continue
            
            ee_arr = np.array(d['ee_position'])
            motion_arr = ee_arr[1:] - ee_arr[0:-1]
            ee_dist = np.sum(ut.norm(motion_arr.T))

            self.min_distance_to_goal.append(np.min(d['d_to_goal']))

            time_l = d['dist_and_angles_time']
            exec_time = time_l[-1]-time_l[0]

            ##START HERE
            self.max_velocity_list.append(0)
            self.ee_distance.append(ee_dist)
            self.execution_time.append(exec_time)
            self.avg_velocity.append(ee_dist/exec_time)

    # def compute_reaches_for_success(self, full_path):
    #     if full_path == 'start':
    #         self.n_reaches_for_success_list = []
    #         return

    #     if full_path == 'end':
    #         d = {}
    #         d['n_reaches_for_success_list'] = self.n_reaches_for_success_list
    #         ut.save_pickle(d, self.root_path + '/count_reaches_for_success.pkl')
    #         return


    #     full_path = full_path.rstrip('/')
    #     res_dict = ut.load_pickle(full_path+'/overall_result.pkl')
    #     trial = full_path.split('/')[-1]

    #     if res_dict is None:
    #         self.n_reaches_for_success_list.append(-1)
    #     elif res_dict['reached_goal']:
    #         pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
    #         if pkl_list != []:
    #             self.n_reaches_for_success_list.append(len(pkl_list))
    #     else:
    #         self.n_reaches_for_success_list.append(-1)











# def get_data(controller_list, logger_list, num_trials):
#     x = []
#     y = []
 
#     z_dict = {}
#     res_dict = {}
    
#     for c in logger_list:
#         res_dict[c] = {'time': [], 'max_force': [], 'avg_force': [],
#                        'success': [], 'mag_contact_forces': [], 
#                        'time_to_complete_list': [], 'max_forces': [], 
#                        'median_forces' : [], 'results' : [], 'max_force_trial': [],
#                        'avg_vel': [], 'rms_error': [] }
#     res_dict["success_intersection"] = []

#     for c in controller_list:
#         z_dict[c] = []
#     z_dict['buf'] = {}
#     z_dict['success'] = {}
#     z_dict['image'] = {}

#     for jj in xrange(11):
#         x.append(jj*2)
#         y.append(jj*2)
        
#         for c in controller_list:
#             z_dict['buf'][c] = []

#         for kk in xrange(11):
#             for c in controller_list:
#                 z_dict['success'][c] = 0
#             total_trials = 0
#             print "starting analysis of results..."
#             path = './F'+str(jj*2).zfill(2)+'_M'+str(kk*2).zfill(2)+'/'
#             res_dict = arc.build_logger_result_dict(path, logger_list, 1, num_trials, res_dict)
#             for i in xrange(num_trials):
#                 folder_name = path+str(i+1).zfill(4)
#                 total_trials=total_trials+1
#                 for c in controller_list:
#                     try:
#                         res_pkl = ut.load_pickle(folder_name+'/'+c)
#                         if res_pkl['result'] == 'Reached':
#                             z_dict['success'][c] = z_dict['success'][c] + 1
#                     except:
#                         print "tried to get result in folder where there was none"
#                         print folder_name

#             if total_trials > 0:
#                 for c in controller_list:
#                     z_dict['buf'][c].append(z_dict['success'][c]/float(total_trials))
#             else:
#                 for c in controller_list:
#                     z_dict['buf'][c].append(0)

#         for c in controller_list:
#             z_dict[c].append(z_dict['buf'][c])

#     ut.save_pickle(z_dict, './compiled_res_dict_explore.pkl')
#     return z_dict, res_dict


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--use_common', action='store_true', dest='use_common',
                 default=False, help='use previously generated common pickle file to make plots')
    p.add_option('--make_common', action='store_true', dest='make_common',
                 default=False, help='make common pickle file to make plots')
    p.add_option('--base_dir', action='store', dest='base_direc',
                 type='string', default = None,
                 help='top level directory with sub-directories for each reach problem.')
    p.add_option('--cont_name', action='store', dest='controller_name',
                 type='string', default = None,
                 help='give base name of controller for desired results')


    opt, args = p.parse_args()

    # if opt.direc == None:
    #     print 'Specify a root directory.'
    #     print 'Exiting ...'
    #     sys.exit()

    #crs = ComputeResultStatistics(['qs_slow_mpc_multi_link_eight'])


    #fn_list.append(crs.success_rate_for_different_stopping_force)

    #fn_list.append(crs.compute_reaches_for_success)
    #fn_list.append(crs.compute_kinematic_statistics)


    directory_list = ['75_obstacle_config_movable120_fixed40',
                      '75_obstacle_config_movable60_fixed60',
                      '75_obstacle_config_movable80_fixed80',
                      '75_obstacle_config_movable40_fixed120',
                      '75_obstacle_config_movable20_fixed20',
                      '75_obstacle_config_movable40_fixed40']

    for direc in directory_list :
        crs = ComputeResultStatistics([opt.controller_name])
        fn_list = []
        # fn_list.append(crs.compute_success_rate)
        fn_list.append(crs.compute_reach_in_force_statistics)
        crs.traverse_directory(opt.base_direc+direc, fn_list)

    #crs.traverse_directory(opt.direc, fn_list)







    # title_list = ['Success Rate for OpenRAVE planning algorithm \n with movable and fixed obstacles treated as fixed',
    #               'Success Rate for OpenRAVE planning algorithm \n without movable obstacles',
    #               'Success Rate for Reach in and out 100 Taxels',
    #               'Success rate for Reach in and out 1 Taxel']
                  # 'Success Rate for QP-based feedback controller',
                  # 'Success Rate for QP-based feedback controller \n with global reach-in/out behavior']

    
    # if opt.use_common == True:
    #     print 'BE AWARE, you are attempting to use a pkl file that was'
    #     print 'previously compiled from old results, is this correct?' 
    #     inp = raw_input('type y for yes \n')
    #     if inp != 'y':
    #         exit
            
    # for l in logger_list:
    #     med_forces = []
    #     med_time = []
    #     med_max_forces = []
    #     max_max_forces = []
        
    #     x = [i*2 for i in range(11)]

    #     for i in xrange(11):
    #         med_forces.append(np.median([res_dict[l]['median_forces'][r] for r in range(i*opt.num_trials*11, (i+1)*opt.num_trials*11)])) 
    #         med_time.append(np.median([res_dict[l]['time_to_complete_list'][r] for r in range(i*opt.num_trials*11, (i+1)*opt.num_trials*11)])) 
    #         med_max_forces.append(np.median([res_dict[l]['max_force'][r] for r in range(i*opt.num_trials*11, (i+1)*opt.num_trials*11)])) 
    #         max_max_forces.append(np.max([res_dict[l]['max_force'][r] for r in range(i*opt.num_trials*11, (i+1)*opt.num_trials*11)])) 

    #     pl.figure()
    #     #pl.plot(res_dict[l]['median_forces'])
    #     pl.plot(x, med_forces)
    #     pl.title('median_forces for \n'+l)

    #     pl.figure()
    #     #pl.plot(res_dict[l]['median_forces'])
    #     pl.plot(x, med_max_forces)
    #     pl.title('median of max forces for \n'+l)

    #     pl.figure()
    #     #pl.plot(res_dict[l]['median_forces'])
    #     pl.plot(x, max_max_forces)
    #     pl.title('max of max forces for \n'+l)
        
    #     pl.figure()
    #     #pl.plot(res_dict[l]['time_to_complete_list'])
    #     pl.plot(x, med_time)
    #     pl.title('median time to complete for \n'+l)
 







# OLD code that was for plotting heat map grid
    #mpu.set_figure_size(10, 7)
#     title_ind = 0
#     for c in controller_list:
#         z_dict['image'][c] = blur_image(np.array(z_dict[c]),1)
#         pl.figure()
#         C = pl.imshow(z_dict['image'][c]*100, origin='lower')
#         cbar = pl.colorbar(C)
#         pl.clim(50,100)
#         cbar.ax.set_ylabel('%Success Rate')
#         pl.title(title_list[title_ind])
#         pl.xlabel('Number of Movable Obstacles')
#         pl.xticks( (0, 2, 4, 6, 8), ('0', '5', '10', '15', '20'))
#         pl.yticks( (0, 2, 4, 6, 8), ('0', '5', '10', '15', '20'))
#         pl.ylabel('Number of Fixed Obstacles')
#         title_ind = title_ind + 1
#     pl.show()


# def gauss_kern(size, sizey=None):
#     """ Returns a normalized 2D gauss kernel array for convolutions """
#     size = int(size)
#     if not sizey:
#         sizey = size
#     else:
#         sizey = int(sizey)
#     x, y = mgrid[-size:size+1, -sizey:sizey+1]
#     g = exp(-(x**2/float(size)+y**2/float(sizey)))
#     return g / g.sum()

# def blur_image(im, n, ny=None) :
#     """ blurs the image by convolving with a gaussian kernel of typical
#         size n. The optional keyword argument ny allows for a different
#         size in the y direction.
#     """
#     g = gauss_kern(n, sizey=ny)
#     improc = signal.convolve(im,g, mode='valid')
#     return(improc)

