#
#
# Copyright (c) 2013, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \authors: Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
# \adviser: Charles Kemp (Healthcare Robotics Lab, Georgia Tech.)

import subprocess
import roslib
roslib.load_manifest('hrl_dynamic_mpc')
import sys
import time
import hrl_lib.util as ut
import numpy as np
import os
from hrl_dynamic_mpc.srv import LogData
import threading
import rospy
import signal
import darci_client as dc


class BatchRunner():
    def __init__(self, num_trials, f_threshes, delta_t_s, goal_reset=None):
        self.num_trials = num_trials
        self.lock = threading.RLock()
        self.first_reach = True
        goal_reset[2] = goal_reset[2] - 0.15
        self.goal_reset = goal_reset
        rospy.init_node('batch_trials_reaching')

        rospy.wait_for_service('rosbag_data')
        rospy.wait_for_service('log_skin_data')
        rospy.wait_for_service('log_data')

        self.rosbag_srv = rospy.ServiceProxy('rosbag_data', LogData)
        self.ft_and_humanoid_record_srv = rospy.ServiceProxy('log_data', LogData)
        self.skin_record_srv = rospy.ServiceProxy('log_skin_data', LogData)
        self.robot_state = dc.DarciClient() 
        self.f_threshes = f_threshes
        self.delta_t_s = delta_t_s
        self.reaching_left_results = []
        self.reaching_right_results = []


    def run_trial(self, i, side, f_thresh, t_impulse, goal):
        self.rosbag_srv('first_impact_'+str(i).zfill(3)+'_'+side+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')
        self.skin_record_srv('first_impact_'+str(i).zfill(3)+'_'+side+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')
        self.ft_and_humanoid_record_srv('first_impact_'+str(i).zfill(3)+'_'+side+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')

        goal_ls = goal[0].A1.tolist()

        goal_str_buf = [str(goal_ls[0])+', '+str(goal_ls[1])+', '+str(goal_ls[2])]
        goal_str = ''.join(goal_str_buf)
        controller = subprocess.call(['python', 
                                      'run_controller_debug.py',
                                      '--darci',
                                      '--t_impulse='+str(t_impulse),
                                      '--f_thresh='+str(f_thresh),
                                      "--goal="+goal_str])

        time.sleep(1.0)
        
        self.rosbag_srv('')
        self.skin_record_srv('')
        self.ft_and_humanoid_record_srv('')

        data = ut.load_pickle('./result.pkl')
        if side == 'left':
            self.reaching_left_results.append(data['result'])
        else:
            self.reaching_right_results.append(data['result'])
        return data['result']


    def run_slip_trial(self, i, f_thresh, t_impulse, goal):
        self.rosbag_srv('slip_impact_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')
        self.skin_record_srv('slip_impact_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')
        self.ft_and_humanoid_record_srv('slip_impact_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')

        goal_ls = goal[0].A1.tolist()

        goal_str_buf = [str(goal_ls[0])+', '+str(goal_ls[1])+', '+str(goal_ls[2])]
        goal_str = ''.join(goal_str_buf)
        controller = subprocess.call(['python', 
                                      'run_controller_debug.py',
                                      '--darci',
                                      '--t_impulse='+str(t_impulse),
                                      '--f_thresh='+str(f_thresh),
                                      "--goal="+goal_str])

        time.sleep(1.0)
        
        self.rosbag_srv('')
        self.skin_record_srv('')
        self.ft_and_humanoid_record_srv('')

        data = ut.load_pickle('./result.pkl')
        self.reaching_right_results.append(data['result'])
        return data['result']

    
    def run_slip_trial(self, i, f_thresh, t_impulse, goal):
        self.rosbag_srv('slip_impact_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')
        self.skin_record_srv('slip_impact_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')
        self.ft_and_humanoid_record_srv('slip_impact_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')

        goal_ls = goal[0].A1.tolist()

        goal_str_buf = [str(goal_ls[0])+', '+str(goal_ls[1])+', '+str(goal_ls[2])]
        goal_str = ''.join(goal_str_buf)
        controller = subprocess.call(['python', 
                                      'run_controller_debug.py',
                                      '--darci',
                                      '--t_impulse='+str(t_impulse),
                                      '--f_thresh='+str(f_thresh),
                                      "--goal="+goal_str])

        time.sleep(1.0)
        
        self.rosbag_srv('')
        self.skin_record_srv('')
        self.ft_and_humanoid_record_srv('')

        data = ut.load_pickle('./result.pkl')
        self.reaching_right_results.append(data['result'])
        return data['result']

    def run_canonical_trial(self, i, f_thresh, t_impulse, goal, num_can):
        self.rosbag_srv('canonical_'+str(num_can).zfill(2)+'_trial_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3))
        self.skin_record_srv('canonical_'+str(num_can).zfill(2)+'_trial_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')
        self.ft_and_humanoid_record_srv('canonical_'+str(num_can).zfill(2)+'_trial_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')

        goal_ls = goal[0].A1.tolist()

        goal_str_buf = [str(goal_ls[0])+', '+str(goal_ls[1])+', '+str(goal_ls[2])]
        goal_str = ''.join(goal_str_buf)
        controller = subprocess.call(['python', 
                                      'run_controller_debug.py',
                                      '--darci',
                                      '--t_impulse='+str(t_impulse),
                                      '--f_thresh='+str(f_thresh),
                                      "--goal="+goal_str])

        time.sleep(1.0)
        
        self.rosbag_srv('')
        self.skin_record_srv('')
        self.ft_and_humanoid_record_srv('')

        data = ut.load_pickle('./result.pkl')
        self.reaching_right_results.append(data['result'])
        return data['result']


    def run_canonical(self, goals, q_configs, num_canonical):
        for cmd in q_configs['start']:
            self.robot_state.setDesiredJointAngles(list(cmd))
            self.robot_state.updateSendCmd()
            time.sleep(2.)

        for f_thresh in self.f_threshes:
            for t_impulse in self.delta_t_s:
                for i in xrange(self.num_trials):
                    self.robot_state.setDesiredJointAngles(list(q_configs['right_start'][0]))
                    self.robot_state.updateSendCmd()
                    time.sleep(1.)
                    
                    result = self.run_canonical_trial(i, f_thresh, t_impulse, goals, num_canonical)

                    for cmd in q_configs['restart']:
                        self.robot_state.setDesiredJointAngles(list(cmd))
                        self.robot_state.updateSendCmd()
                        time.sleep(2.)
                    self.robot_state.setDesiredJointAngles(list(q_configs['right_start'][0]))
                    self.robot_state.updateSendCmd()
                    time.sleep(1.)
        data2 = {}
        data2['reaching_straight'] = self.reaching_right_results 
        ut.save_pickle(data, './combined_results_for_canonical'+str(num_canonical)+'.pkl')


    def run_foliage_trial(self, i, f_thresh, t_impulse, goal, num_reach, record = True):
        if record == True:
            self.rosbag_srv('foliage_goal_'+str(num_reach).zfill(3)+'_trial_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')
            self.skin_record_srv('foliage_goal_'+str(num_reach).zfill(3)+'_trial_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')
            self.ft_and_humanoid_record_srv('foliage_goal_'+str(num_reach).zfill(3)+'_trial_'+str(i).zfill(3)+'_f_thresh_'+str(f_thresh).zfill(2)+'_delta_t_impulse_'+str(t_impulse).zfill(3)+'_')

        goal_ls = goal

        goal_str_buf = [str(goal_ls[0])+', '+str(goal_ls[1])+', '+str(goal_ls[2])]
        goal_str = ''.join(goal_str_buf)
        controller = subprocess.call(['python', 
                                      'run_controller_debug.py',
                                      '--darci',
                                      '--t_impulse='+str(t_impulse),
                                      '--f_thresh='+str(f_thresh),
                                      "--goal="+goal_str])

        time.sleep(1.0)
        
        if record == True:
            self.rosbag_srv('')
            self.skin_record_srv('')
            self.ft_and_humanoid_record_srv('')

        data = ut.load_pickle('./result.pkl')
        return data['result']



    def run_foliage_reach(self, goals, q_configs, num_reach):
        if self.first_reach == True:
            self.first_reach = False
            for cmd in q_configs['start']:
                self.robot_state.setDesiredJointAngles(list(cmd))
                self.robot_state.updateSendCmd()
                time.sleep(2.)

        for f_thresh in self.f_threshes:
            for t_impulse in self.delta_t_s:
                for i in xrange(self.num_trials):
                    self.robot_state.setDesiredJointAngles(list(q_configs['trial_start'][0]))
                    self.robot_state.updateSendCmd()
                    time.sleep(1.)
                    
                    result = self.run_foliage_trial(i, f_thresh, t_impulse, goals, num_reach)
                    offset = 0.20 - goals[2]
                    goals[2] = goals[2]+offset
                    counter = (str(i)+'_up').zfill(6)
                    reset_result = self.run_foliage_trial(counter, f_thresh, t_impulse, goals, num_reach)
                    reset_result = self.run_foliage_trial(i, f_thresh, t_impulse, self.goal_reset, num_reach, record = False)
                    
                    if result != 'success':
                        raw_input('Help me a bit please ..')

                    for cmd in q_configs['restart']:
                        self.robot_state.setDesiredJointAngles(list(cmd))
                        self.robot_state.updateSendCmd()
                        time.sleep(2.)
                    self.robot_state.setDesiredJointAngles(list(q_configs['trial_start'][0]))
                    self.robot_state.updateSendCmd()
                    time.sleep(1.)
        data2 = {}
        data2['reaching_straight'] = self.reaching_right_results 
        ut.save_pickle(data, './combined_results_for_foliage.pkl')



    def run_first_impact(self, goals, q_configs):

        for cmd in q_configs['start']:
            self.robot_state.setDesiredJointAngles(list(cmd))
            self.robot_state.updateSendCmd()
            time.sleep(2.)

        for f_thresh in self.f_threshes:
            for t_impulse in self.delta_t_s:
                for i in xrange(self.num_trials):
                    self.robot_state.setDesiredJointAngles(list(q_configs['left_start'][0]))
                    self.robot_state.updateSendCmd()
                    time.sleep(1.)

                    side = 'left'
                    result = self.run_trial(i, side, f_thresh, t_impulse, goals[side])

                    if result == 'success':
                        self.robot_state.setDesiredJointAngles(list(q_configs['right_start'][0]))
                        self.robot_state.updateSendCmd()
                        time.sleep(2.)
                    else:
                        for cmd in q_configs['left_to_right_restart']:
                            self.robot_state.setDesiredJointAngles(list(cmd))
                            self.robot_state.updateSendCmd()
                            time.sleep(2.)
                        self.robot_state.setDesiredJointAngles(list(q_configs['right_start'][0]))
                        self.robot_state.updateSendCmd()
                        time.sleep(1.)

                    side = 'right'
                    result = self.run_trial(i, side, f_thresh, t_impulse, goals[side])

                    if result == 'success':
                        self.robot_state.setDesiredJointAngles(list(q_configs['left_start'][0]))
                        self.robot_state.updateSendCmd()
                        time.sleep(2.)
                    else:
                        for cmd in q_configs['right_to_left_restart']:
                            self.robot_state.setDesiredJointAngles(list(cmd))
                            self.robot_state.updateSendCmd()
                            time.sleep(2.)
                        self.robot_state.setDesiredJointAngles(list(q_configs['left_start'][0]))
                        self.robot_state.updateSendCmd()
                        time.sleep(1.)

        data2 = {}
        data2['reaching_left'] = self.reaching_left_results
        data2['reaching_right'] = self.reaching_right_results 
        ut.save_pickle(data, './combined_results_for_first_impact.pkl')

    def in_hull(self, p, hull):
        """
        Test if points in `p` are in `hull`

        `p` should be a `NxK` coordinates of `N` points in `K` dimension
        `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
        coordinates of `M` points in `K`dimension for which a Delaunay triangulation
        will be computed
        """
        from scipy.spatial import Delaunay
        if not isinstance(hull,Delaunay):
            hull = Delaunay(hull)

        return hull.find_simplex(p)>=0

if __name__ == '__main__':

    num_trials = 1

    #f_threshes = [10.] #, 15.]
    f_threshes = [5.]
    #delta_t_s = [2., 4., 16., 48.]
    delta_t_s = [8.]
    #delta_t_s = [16., 48.]
    data = ut.load_pickle('./joint_and_ee_data.pkl')

    goal = data['ee_positions']['restart']
    goal_reset = goal[0].A1.tolist()

    runner = BatchRunner(num_trials, f_threshes, delta_t_s, goal_reset)
    
    # goals = {'left':data['ee_positions']['right_start'],
    #          'right':data['ee_positions']['left_start']}

    # runner.run_first_impact(goals, data['q_configs'])

    # data = ut.load_pickle('./starting_configs.pkl')
    # goals = data['ee_positions']['goal']
    # #runner.run_slip_impact(goals, data['q_configs'])

    # runner.run_canonical(data['ee_positions']['goal'], data['q_configs'], 5)

    range_pos = np.array(data['ee_positions']['range']).reshape(7,3)
    
    z_max = -0.05
    z_min = -0.25
    x_max = np.max(range_pos[:,0])
    x_min = np.min(range_pos[:,0])
    y_max = np.max(range_pos[:,1])
    y_min = np.min(range_pos[:,1])


    goals = []

    for i in xrange(120):
        
        flag = False
        while flag == False:
            x_rand, y_rand, z_rand = np.random.rand(3)
            x = x_rand*(x_max-x_min)+x_min
            y = y_rand*(y_max-y_min)+y_min
            z = z_rand*(z_max-z_min)+z_min

            flag = runner.in_hull(np.array([x, y]), range_pos[:, 0:2].reshape(7,2))

            if np.sqrt(x**2+(y-0.185)**2) < 0.30:
                flag = False

        goal_ls = [x, y, z]
        goals.append(goal_ls)

        ut.save_pickle(goals, './goal_positions.pkl')
        runner.run_foliage_reach(goal_ls, data['q_configs'], i)


