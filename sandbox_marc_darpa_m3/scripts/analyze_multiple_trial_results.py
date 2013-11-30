
import sys, os
import numpy as np, math

import scipy.stats as ss

import roslib; roslib.load_manifest('sandbox_marc_darpa_m3')
#roslib.load_manifest('hrl_software_simulation_darpa_m3')
import hrl_lib.util as ut
import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as sim_robot_config
import hrl_software_simulation_darpa_m3.gen_sim_arms as sim_robot

class ComputeResultStatistics():
    def __init__(self, robot= None, log_name = None, threshold = 5):

        self.root_path = None
        self.threshold = threshold

        if robot == None:
            self.robot = sim_robot.ODESimArm(sim_robot_config)
        else:
            self.robot = robot
        if log_name == None:
            #self.log_name = '/reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl' #
            self.log_name = '/reach_log.pkl'
        else:
            self.log_name = log_name

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
                    # try:
                    #     fn(full_path)
                    # except:
                    #     print "function failed at :", full_path
        for fn in fn_list:
            fn('end')

    def get_sum_taxel_hist(self, full_path):
        if full_path == 'start':
            self.taxel_hist = None
            return

        if full_path == 'end':
            d = {}
            d['taxel_hist'] = self.taxel_hist.tolist()
            ut.save_pickle(d, self.root_path + '/taxel_histogram.pkl')
            print "here's taxel hist :\n", self.taxel_hist

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+self.log_name)
        trial = full_path.split('/')[-1]

        if self.taxel_hist == None:
            try:
                self.taxel_hist = np.array([0]*len(res_dict['taxel_hist']))
            except:
                print "no taxel_hist in this logging files"
        if res_dict is None:
            pass
        else: 
            self.taxel_hist = self.taxel_hist + res_dict['taxel_hist']


    def get_contact_loc_hist(self, full_path):
        if full_path == 'start':
            self.contact_hist = None
            self.contact_hist_5N = None
            self.contact_hist_10N = None
            self.contact_hist_25N = None
            self.contact_hist_40N= None
            self.multi_contact_hist = None
            self.multi_contact_hist_5N = None
            self.multi_contact_hist_10N = None
            self.multi_contact_hist_25N = None
            self.multi_contact_hist_40N = None
            return

        if full_path == 'end':
            d = {}
            d['contact_hist'] = self.contact_hist
            d['contact_hist_5N'] = self.contact_hist_5N
            d['contact_hist_10N'] = self.contact_hist_10N
            d['contact_hist_25N'] = self.contact_hist_25N
            d['contact_hist_40N'] = self.contact_hist_40N
            d['multi_contact_hist'] = self.multi_contact_hist
            d['multi_contact_hist_5N'] = self.multi_contact_hist_5N
            d['multi_contact_hist_10N'] = self.multi_contact_hist_10N
            d['multi_contact_hist_25N'] = self.multi_contact_hist_25N
            d['multi_contact_hist_40N'] = self.multi_contact_hist_40N

            ut.save_pickle(d, self.root_path + '/contact_histograms.pkl')

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+self.log_name)
        trial = full_path.split('/')[-1]

        if self.multi_contact_hist == None:
            try:
                self.contact_hist = {'right':[], 'left':[]}
                self.contact_hist_5N = {'right':[], 'left':[]}
                self.contact_hist_10N = {'right':[], 'left':[]}
                self.contact_hist_25N = {'right':[], 'left':[]}
                self.contact_hist_40N = {'right':[], 'left':[]}
                self.multi_contact_hist = {'right':[], 'left':[]}
                self.multi_contact_hist_5N = {'right':[], 'left':[]}
                self.multi_contact_hist_10N = {'right':[], 'left':[]}
                self.multi_contact_hist_25N = {'right':[], 'left':[]}
                self.multi_contact_hist_40N = {'right':[], 'left':[]}
            except:
                print "no contact in this logging files or other error ..."

        if res_dict is None:
            pass
        else: 
            try:
                num_contacts_ind = np.where(np.array(res_dict['num_contacts_at_time_instant_list']) > 0)[0]
                q_s = np.array(res_dict['q_list'])[num_contacts_ind]
                num_contacts = np.array(res_dict['num_contacts_at_time_instant_list'])[num_contacts_ind]
                for j in xrange(len(num_contacts)):
                    ind_nom = int(np.sum(num_contacts[0:j]) - num_contacts[j])

                    for i in xrange(int(num_contacts[j])):
                        ind = ind_nom + i
                        jt_pos, _ = self.robot.kinematics.FK(q_s[j], res_dict['all_forces_jts_list'][ind])
                        next_jt_pos, _ = self.robot.kinematics.FK(q_s[j], res_dict['all_forces_jts_list'][ind]+1)
                        jt_vec = next_jt_pos - jt_pos
                        cont_loc = np.matrix(res_dict['all_forces_locs_list'][ind]).reshape(3,1)
                        cont_vec = cont_loc-jt_pos
                        dist_cont = cont_vec.T*(jt_vec/np.linalg.norm(jt_vec))
                        zero_jt_pos, _ = self.robot.kinematics.FK([0,0,0], res_dict['all_forces_jts_list'][ind])
                        cont_hist_loc = abs(zero_jt_pos[1,0]) + dist_cont[0,0]

                        
                        local_y_dir = np.cross(jt_vec.A1, np.array([0, 0, 1]))
                        offset_dir = np.dot(cont_vec.A1, local_y_dir/np.linalg.norm(local_y_dir))

                        if offset_dir > 0:
                            side = 'right'
                        else:
                            side = 'left'
                            
                        # if cont_hist_loc > 1.00:
                        #     print "jt is :", res_dict['all_forces_jts_list'][ind]
                        #     print "cont_loc is :", cont_loc
                        #     print "force is :", res_dict['all_forces_list'][ind]
                        #     pl.figure()
                        #     pl.plot([jt_pos[0,0], next_jt_pos[0,0]], [jt_pos[1,0], next_jt_pos[1,0]], 'b')
                        #     pl.hold(True)
                        #     pl.plot([jt_pos[0,0]], [jt_pos[1,0]], 'ko')
                        #     pl.hold(True)
                        #     pl.plot(cont_loc[0,0], cont_loc[1, 0], 'r*')
                        #     pl.hold(True)
                        #     check_dist = dist_cont[0,0]*jt_vec/np.linalg.norm(jt_vec) + jt_pos
                        #     pl.plot(check_dist[0], check_dist[1], 'g*')
                        #     pl.axis('equal')
                        #     pl.show()

                        self.contact_hist[side].append(cont_hist_loc)
                        if res_dict['all_forces_list'][ind] > 5:
                            self.contact_hist_5N[side].append(cont_hist_loc)
                        if res_dict['all_forces_list'][ind] > 10:
                            self.contact_hist_10N[side].append(cont_hist_loc)
                        if res_dict['all_forces_list'][ind] > 25:
                            self.contact_hist_25N[side].append(cont_hist_loc)
                        if res_dict['all_forces_list'][ind] > 40:
                            self.contact_hist_40N[side].append(cont_hist_loc)

                        if num_contacts[j] > 1:
                            self.multi_contact_hist[side].append(cont_hist_loc)
                        if num_contacts[j] > 1 and res_dict['all_forces_list'][ind] > 5:
                            self.multi_contact_hist_5N[side].append(cont_hist_loc)
                        if num_contacts[j] > 1 and res_dict['all_forces_list'][ind] > 10:
                            self.multi_contact_hist_10N[side].append(cont_hist_loc)
                        if num_contacts[j] > 1 and res_dict['all_forces_list'][ind] > 25:
                            self.multi_contact_hist_25N[side].append(cont_hist_loc)
                        if num_contacts[j] > 1 and res_dict['all_forces_list'][ind] > 40:
                            self.multi_contact_hist_40N[side].append(cont_hist_loc)
            except:
                print "dying probably because force vectors not the same length .."
    



    def get_impact_and_jamming_data(self, full_path):
        if full_path == 'start':
            self.contact_vel = []
            self.contact_force = []
            self.contact_ind = []
            self.contact_locs = []
            self.jamming_forces = []
            self.jamming_normals = []
            self.jamming_vel = []
            self.total_num_cont_jamming = 0
            self.contact_type = []
            return

        if full_path == 'end':
            d = {}
            d['total_num_cont_jamming'] = self.total_num_cont_jamming
            d['jamming_force'] = self.jamming_forces
            d['jamming_vel'] = self.jamming_vel
            d['jamming_normals'] = self.jamming_normals
            d['contact_vel'] = self.contact_vel
            d['contact_force'] = self.contact_force
            d['contact_locs'] = self.contact_locs
            d['contact_type'] = self.contact_type
            ut.save_pickle(d, self.root_path + '/stats_on_impact_velocity_and_jamming.pkl')

        full_path = full_path.rstrip('/')
        data = ut.load_pickle(full_path+self.log_name)
        trial = full_path.split('/')[-1]

        if data is None:
            pass
        elif len(data['q_list']) != len(data['num_contacts_at_time_instant_list']):
            pass
        elif len(data['q_list']) != len(data['qdot_list']):
            pass
        elif np.sum(data['num_contacts_at_time_instant_list']) != len(data['all_forces_locs_list']):
            pass
        elif len(data['all_forces_list']) != len(data['all_forces_locs_list']):
            pass
        elif len(data['all_forces_list']) != len(data['all_forces_jts_list']):
            pass
        else:
            counter_jamming = 0
            counter = 0
            for i in xrange(len(data['q_list'])):
                flag = False
                if data['num_contacts_at_time_instant_list'][i] > 0:
                    #check if any of the nrmls for high forces are opposite directions

                    for j in xrange(int(data['num_contacts_at_time_instant_list'][i])):
                        jac = self.robot.kinematics.jacobian(data['q_list'][i], np.matrix(data['all_forces_locs_list'][counter]).reshape(3,1))
                        jac = jac[0:2, 0:data['all_forces_jts_list'][counter]+1]
                        vel = jac*(np.matrix(data['qdot_list'][i]).reshape(3,1))[0:data['all_forces_jts_list'][counter]+1,:]
                        self.contact_vel.append(vel)
                        self.contact_locs.append(data['all_forces_locs_list'][counter])
                        self.contact_force.append(data['all_forces_list'][counter])
                        counter = counter + 1

                if data['num_contacts_at_time_instant_list'][i] > 1:
                    force_buf = []
                    nrml_buf = []
                    ind_buf = []
                    vel_buf = []
                    for j in xrange(int(data['num_contacts_at_time_instant_list'][i])):
                        for k in xrange(j+1, int(data['num_contacts_at_time_instant_list'][i])):
                            if np.dot(data['all_forces_nrmls_list'][counter_jamming], data['all_forces_nrmls_list'][counter_jamming+k]) < 0:
                                flag = True
                        jac = self.robot.kinematics.jacobian(data['q_list'][i], np.matrix(data['all_forces_locs_list'][counter_jamming]).reshape(3,1))
                        jac = jac[0:2, 0:data['all_forces_jts_list'][counter_jamming]+1]
                        vel = jac*(np.matrix(data['qdot_list'][i]).reshape(3,1))[0:data['all_forces_jts_list'][counter_jamming]+1,:]
                        force_buf.append(data['all_forces_list'][counter_jamming])
                        nrml_buf.append(data['all_forces_nrmls_list'][counter_jamming])
                        vel_buf.append(vel)
                        counter_jamming = counter_jamming + 1
                    if flag == True: # and np.max(force_buf) > self.threshold:
                        self.jamming_vel.append(vel_buf)
                        self.jamming_forces.append(force_buf)
                        self.jamming_normals.append(nrml_buf)
                        self.total_num_cont_jamming = self.total_num_cont_jamming + data['num_contacts_at_time_instant_list'][i]

                if flag == True:
                    self.contact_type =self.contact_type +  ['jamming']*int(data['num_contacts_at_time_instant_list'][i])
                else:
                    self.contact_type =self.contact_type +  ['other']*int(data['num_contacts_at_time_instant_list'][i])


    def get_trials_over_50N(self, full_path):
        if full_path == 'start':
            self.trial_list = []
            self.high_force_list = []
            return

        if full_path == 'end':
            d = {}
            d['trials'] = self.trial_list
            d['forces'] = self.high_force_list
            ut.save_pickle(d, self.root_path + '/trials_with_forces_over_50N.pkl')

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+self.log_name)
        trial = full_path.split('/')[-1]

        if res_dict is None:
            pass
        elif res_dict['all_forces_list'] != []: 
            if np.max(res_dict['all_forces_list']) > 50.:
                self.trial_list.append(trial)
                self.high_force_list.append(np.max(res_dict['all_forces_list']))

    def combine_controller_rates(self, full_path):
        if full_path == 'start':
            self.controller_rate = []
            return

        if full_path == 'end':
            d = {}
            d['all_controller_rates'] = np.hstack(self.controller_rate).tolist()
            ut.save_pickle(d, self.root_path + '/controller_rates.pkl')

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+self.log_name)
        trial = full_path.split('/')[-1]

        if res_dict is None:
            pass
        else: 
            self.controller_rate.append(res_dict['jep_send_rate'])

    def trials_where_mean_stopped_moving(self, full_path):
        if full_path == 'start':
            self.trial_list = []
            return

        if full_path == 'end':
            d = {}
            d['trial_list'] = self.trial_list
            d['num_trials'] = len(self.trial_list)
            ut.save_pickle(d, self.root_path + '/trials_where_ee_stopped_moving.pkl')

        full_path = full_path.rstrip('/')
        log_dict = ut.load_pickle(full_path+self.log_name)
        res_dict = ut.load_pickle(full_path+'/overall_result.pkl')

        trial = full_path.split('/')[-1]

        if res_dict is None or log_dict['mean_motion_list'] == []:
            pass
        elif log_dict['mean_motion_list'][-1] < 0.001 and res_dict['reached_goal'] != True:
            self.trial_list.append(full_path)


    def count_max_number_of_contacts_per_trial(self, full_path):
        if full_path == 'start':
            self.max_contact_list = []
            return

        if full_path == 'end':
            d = {}
            d['max_contact_list'] = self.max_contact_list
            ut.save_pickle(d, self.root_path + '/max_contact_list.pkl')

        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+self.log_name)
        trial = full_path.split('/')[-1]

        if res_dict is None:
            pass
        else: 
            self.max_contact_list.append(np.max(res_dict['num_contacts_at_time_instant_list']))

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

    # what would the success rate for a single reach have been if the
    # stopping force were different.
    def success_rate_for_different_stopping_force(self, full_path):
        stopping_force_list = [5, 10, 15, 20, 25, 30, 40, 50, 100]

        if full_path == 'start':
            self.success_count_list = [0 for f in stopping_force_list]
            self.fail_count_list = [0 for f in stopping_force_list]
            return

        if full_path == 'end':
            d = {}
            for i in range(len(stopping_force_list)):
                print '==== Stopping force :%.1fN ======'%stopping_force_list[i]
                print self.success_count_list[i], 'successful trials'
                print self.fail_count_list[i], 'failed trials'
                print self.success_count_list[i] * 1. / (self.success_count_list[i]+self.fail_count_list[i]), '% success rate.'
                print self.success_count_list[i] + self.fail_count_list[i], 'total trials'
                d[i] = {}
                d[i]['force'] = stopping_force_list[i]
                d[i]['success_count'] = self.success_count_list[i]
                d[i]['fail_count'] = self.fail_count_list[i]
            ut.save_pickle(d, self.root_path + '/success_rate_diff_stopping_force.pkl')
            return

        param_pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*param*.pkl')
        param_pkl_list.reverse()
        pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/reach*log*.pkl')
        pkl_list.reverse()

        if pkl_list != []:
            pd = ut.load_pickle(param_pkl_list[0])
            stopping_dist_to_goal = pd['stopping_dist_to_goal'] + 0.002

            d = ut.load_pickle(pkl_list[0])
            max_f_l = d['max_force_list']
            max_f_arr = np.array(max_f_l)
            goal = d['local_goal']

            for i,f in enumerate(stopping_force_list):
                idx_arr = np.where(max_f_arr > f)[0]
                if len(idx_arr) == 0:
                    idx = -1
                else:
                    idx = idx_arr[0]

                try:
                    ee = np.matrix(d['ee_pos_list'][idx]).T
                    dist = np.linalg.norm(ee - goal)
                except IndexError:
                    print '-----------------------------------'
                    print full_path
                    print 'idx:', idx
                    print 'len(d[\'ee_pos_list\']):', len(d['ee_pos_list'])
                    print 'len(d[\'max_force_list\']):', len(d['max_force_list'])
                    dist = 200.

                if dist <= stopping_dist_to_goal:
                    self.success_count_list[i] += 1
                else:
                    self.fail_count_list[i] += 1

    def compute_reach_in_force_statistics(self, full_path):
        if full_path == 'start':
            self.single_reach_max_force_list = []
            self.single_reach_95_percentile_force_list = []
            self.single_reach_avg_force_list = []
            self.single_reach_force_count_list_for_avg = []

            self.all_forces_mag_list = []
            self.max_force_list = []
            self.avg_force_list = []
            self.force_count_list_for_avg = []
            return

        if full_path == 'end':
            def compute_overall_avg(avg_list, count_list):
                return np.sum(np.multiply(avg_list, count_list)) / np.sum(count_list)

            d = {}

            d['all_forces_mag_list'] = self.all_forces_mag_list
            d['median_contact_force'] = ss.scoreatpercentile(self.all_forces_mag_list, 50)
            d['first_quartile_contact_force'] = ss.scoreatpercentile(self.all_forces_mag_list, 25)
            d['third_quartile_contact_force'] = ss.scoreatpercentile(self.all_forces_mag_list, 75)
            print ''
            print 'Median contact force:', d['median_contact_force']
            print 'First quartile contact force:', d['first_quartile_contact_force']
            print 'Third quartile contact force', d['third_quartile_contact_force']
            print ''

            avg_max_force = np.mean(self.max_force_list)
            avg_contact_force = compute_overall_avg(self.avg_force_list, self.force_count_list_for_avg)
            print ''
            print 'Avg. max force:', avg_max_force
            print 'Avg. contact force:', avg_contact_force
            print ''

            single_reach_avg_max_force = np.mean(self.single_reach_max_force_list)
            single_reach_avg_contact_force = compute_overall_avg(self.single_reach_avg_force_list,
                                                       self.single_reach_force_count_list_for_avg)
            print ''
            print 'Avg. max force (Single Reach):', single_reach_avg_max_force
            print 'Avg. contact force (Single Reach):', single_reach_avg_contact_force
            print ''
            print ''
            d['avg_max_force'] = avg_max_force
            d['avg_contact_force'] = avg_contact_force
            d['avg_max_force_single_reach'] = single_reach_avg_max_force
            d['avg_contact_force_single_reach'] = single_reach_avg_contact_force
            d['max_force_list'] = self.max_force_list
            d['percentile_95_force_list'] = self.single_reach_95_percentile_force_list
            ut.save_pickle(d, self.root_path + '/reach_in_force_statistics.pkl')
            return

        #pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/*reach_from*to_goal*log*.pkl')
        #pkl_list.reverse()

        # ONLY FOR SINGLE REACH. Advait felt like the ls -t was
        # making nfs real slow.
        pkl = full_path+self.log_name
        pkl_list = [pkl]

        for i, pkl in enumerate(pkl_list):
            d = ut.load_pickle(pkl)
            if d == None:
                continue
            f_mag_l = d['all_forces_list']
            if len(f_mag_l) == 0.:
                continue

            if i == 0:
                self.single_reach_max_force_list.append(max(f_mag_l))
                self.single_reach_95_percentile_force_list.append(ss.scoreatpercentile(f_mag_l, 95))
                self.single_reach_avg_force_list.append(np.mean(f_mag_l))
                self.single_reach_force_count_list_for_avg.append(len(f_mag_l))

            self.all_forces_mag_list.extend(f_mag_l)
            self.max_force_list.append(max(f_mag_l))
            self.avg_force_list.append(np.mean(f_mag_l))
            self.force_count_list_for_avg.append(len(f_mag_l))

    def compute_kinematic_statistics(self, full_path):
        if full_path == 'start':
            self.ee_distance = []
            self.execution_time = []
            self.avg_velocity = []
            self.success_velocity = {}
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
            d['success_vel_dict'] = self.success_velocity
            ut.save_pickle(d, self.root_path + '/reach_in_kinematics_statistics.pkl')
            return

        pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/reach*log*.pkl')
        pkl_list.reverse()

        res_dict = ut.load_pickle(full_path+'/overall_result.pkl')
        trial = full_path.split('/')[-1]

        total_dist = 0.
        total_time = 0.

        for i, pkl in enumerate(pkl_list):
            d = ut.load_pickle(pkl)
            if d == None:
                continue
            
            ee_arr = np.array(d['ee_pos_list'])
            motion_arr = ee_arr[1:] - ee_arr[0:-1]
            ee_dist = np.sum(ut.norm(motion_arr.T))
            total_dist = total_dist + ee_dist

            time_l = d['time_stamp_list']
            exec_time = time_l[-1]-time_l[0]
            total_time = total_time + exec_time

            self.ee_distance.append(ee_dist)
            self.execution_time.append(exec_time)
            self.avg_velocity.append(ee_dist/exec_time)

        if res_dict['reached_goal']:
            success = True
        else:
            success = False

        self.success_velocity[trial] = {'dist':total_dist, 'success':success, 'time':total_time}


    def compute_reaches_for_success(self, full_path):
        if full_path == 'start':
            self.n_reaches_for_success_list = []
            return

        if full_path == 'end':
            d = {}
            d['n_reaches_for_success_list'] = self.n_reaches_for_success_list
            ut.save_pickle(d, self.root_path + '/count_reaches_for_success.pkl')
            return


        full_path = full_path.rstrip('/')
        res_dict = ut.load_pickle(full_path+'/overall_result.pkl')
        trial = full_path.split('/')[-1]

        if res_dict is None:
            self.n_reaches_for_success_list.append(-1)
        elif res_dict['reached_goal']:
            pkl_list = ut.get_bash_command_output('ls -t '+full_path+'/reach*log*.pkl')
            if pkl_list != []:
                self.n_reaches_for_success_list.append(len(pkl_list))
        else:
            self.n_reaches_for_success_list.append(-1)


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--dir', action='store', dest='direc',
                 type='string', default = None,
                 help='top level directory with sub-directories for each reach problem.')

    opt, args = p.parse_args()

    if opt.direc == None:
        print 'Specify a root directory.'
        print 'Exiting ...'
        sys.exit()

    #crs = ComputeResultStatistics(log_name = '/reach_log.pkl', threshold = 5)
    crs = ComputeResultStatistics(log_name = '/reach_from_reach_in_dy_0.00_retry_right=to_goal_1_log.pkl', threshold = 25)

    fn_list = []

    #fn_list.append(crs.get_impact_and_jamming_data)

    fn_list.append(crs.get_contact_loc_hist)


    # core # of pkl files that we use for analysis
    # fn_list.append(crs.compute_success_rate)
    # fn_list.append(crs.compute_kinematic_statistics)
    # fn_list.append(crs.compute_reach_in_force_statistics)
    # fn_list.append(crs.get_trials_over_50N)
    # fn_list.append(crs.combine_controller_rates)
    # fn_list.append(crs.trials_where_mean_stopped_moving)
    # fn_list.append(crs.count_max_number_of_contacts_per_trial)

    # # fn_list.append(crs.get_sum_taxel_hist)
    # # fn_list.append(crs.success_rate_for_different_stopping_force)
    # # fn_list.append(crs.compute_reaches_for_success)

    crs.traverse_directory(opt.direc, fn_list)





