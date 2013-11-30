#!/usr/local/bin/python

import sys
import subprocess
import os
import time
import glob
import roslib
roslib.load_manifest('hrl_lib')
import rospy
import hrl_lib.util as ut

def check_files(folder):
    if not os.path.isfile(folder+'/overall_result.pkl'):
        print "no overall results file"
        return False
    elif glob.glob(folder+'/*reach_from*to_goal*log*.pkl') == []:
        print "no controller logging files"
        return False
    else:
        file_list = glob.glob(folder+'/*pkl')
        for pkl_file in file_list:
            try:
                data = ut.load_pickle(pkl_file)
            except:
                print "failed to load pickle, corrupted? ..."
                return False
            if data == None:
                print "failed to load pickle, none there? ..."
                return False
        print "file exists"
        return True

def check_running(folder):
    if not os.path.isfile(folder+'/running.txt'):
        print "no other machine is running this trial"
        return False
    else:
        print "another machine IS running this trial"
        return True


if __name__ == '__main__':
    if sys.argv[1] == "":
        print "Give name of reach_problem_dict"
        sys.exit()

    if sys.argv[2] == "use_skin":
        CONTROLLER_SWITCH=''
        SENSING="taxels"        
    elif sys.argv[2] == "use_ft_sensor":
        CONTROLLER_SWITCH=''
        SENSING="ft_at_base"
    elif sys.argv[2] == "ignore_skin":
        CONTROLLER_SWITCH='--is'
        SENSING="taxels"
    else:
        print 'Specify either use_skin or use_ft_sensor or ignore_skin'
        sys.exit()

    if sys.argv[3] == "single_reach":
        REACH_SWITCH='--single_reach'
    elif sys.argv[3] == "multiple_reaches":
        REACH_SWITCH=''
    else:
        print 'Specify either single_reach or multiple_reaches'
        sys.exit()

    if sys.argv[4] == "":
        print 'Specify an allowable force'
        sys.exit()

    if sys.argv[5] == "":
        print "need to specify which simulation arm to use"
        sys.exit()
    elif sys.argv[5] == "sim3_with_hand":
        d_robot = '--three_link_with_hand'
    elif sys.argv[5] == "sim3":
        d_robot = '--planar_three_link_capsule'

    if sys.argv[6] == "":
        imp_scale = 1
    else:
        imp_scale = sys.argv[6]

    print "trying to make directory"

    intermed_name = (os.path.split(sys.argv[1])[1]).split('_')[3:6]

    dir_name = intermed_name[0]+'_'+intermed_name[1]+'_'+intermed_name[-1].split('.')[0]

    print "name is :", dir_name

    print os.system('pwd')
    os.system('mkdir '+dir_name) 
    os.chdir(dir_name) 

    check = False
    count = 0

    if check_files('./'):
        print "#############################################################################"
        print "Result pkl exists. Ignoring this trial :" + dir_name  
        print "#############################################################################"
        sys.exit()

    if check_running('./'):
        print "#############################################################################"
        print "Running already on another machine, ignoring this trial :" + dir_name  
        print "#############################################################################"
        sys.exit()

    os.system('touch running.txt')

    not_done = True

    while not_done == True:
        count = count + 1

        try:
            rospy.delete_param('/launch')
        except KeyError:
            print "value not set"
        try:
            rospy.delete_param('/m3')
        except KeyError:
            print "value not set"

        rospy.set_param('use_sim_time', True)
        rospy.set_param('/m3/software_testbed/resolution', 100)

        print sys.argv[1]
        subprocess.Popen(['rosrun', 
                          'hrl_common_code_darpa_m3', 
                          'obstacles.py', 
                          '--pkl='+sys.argv[1]])

        subprocess.Popen(['rosrun', 
                          'hrl_software_simulation_darpa_m3', 
                          'sim_arm_param_upload.py', 
                          d_robot])

        if SENSING == "taxels":
            simulator = subprocess.Popen(['rosrun',
                                          'hrl_software_simulation_darpa_m3',
                                          'simulator',
                                          '/skin/contacts:=/skin/contacts_unused',
                                          '__name:=software_simulation_node'])

            time.sleep(3)

            skin_msg = subprocess.Popen(['rosrun', 
                                         'hrl_common_code_darpa_m3',
                                         'taxel_array_to_skin_contact.py',
                                         '__name:=taxel_array_to_skin_contact'])

            logger_process = subprocess.Popen(['rosrun', 
                                               'sandbox_advait_darpa_m3',
                                               'log_and_monitor_node.py',
                                               '--'+sys.argv[5],
                                               '--log_taxel_msg',
                                               '__name:=ode_log_and_monitor_node'])
            
        elif SENSING == "ft_at_base":
            simulator = subprocess.Popen(['rosrun',                 
                                          'hrl_software_simulation_darpa_m3',
                                          'simulator',
                                          '/skin/contacts:=/skin/contacts_all',
                                          '/skin/taxel_array:=/skin/taxel_array_unused',
                                          '__name:=software_simulation_node'])

            time.sleep(3)

            skin_msg = subprocess.Popen(['rosrun',
                                         'hrl_common_code_darpa_m3',
                                         'skin_contact_to_resultant_force.py',
                                         '__name:=skin_contact_to_resultant_force'])

            logger_process = subprocess.Popen(['rosrun',
                                               'sandbox_advait_darpa_m3',
                                               'log_and_monitor_node.py',
                                               '--'+sys.argv[5],
                                               '--log_taxel_msg',
                                               '/skin/contacts:=/skin/contacts_all',
                                               '__name:=ode_log_and_monitor_node'])


        time.sleep(2)

        var_imped = subprocess.Popen(['rosrun',
                                      'sandbox_marc_darpa_m3',
                                      'var_comp_optimize.py',
                                      '--pub_once_alpha',
                                      str(imp_scale)])

        time.sleep(2)

        contact_memory = subprocess.Popen(['rosrun',
                                           'sandbox_advait_darpa_m3',
                                           'contact_memory.py',
                                           '--'+sys.argv[5],
                                           '__name:=contact_memory_node'])

        controller = subprocess.call(['rosrun',
                                      'sandbox_advait_darpa_m3',
                                      'switch_among_controllers_node.py',
                                      '--batch', 
                                      '--'+sys.argv[5],
                                      '--ignore_mobile_base',
                                      CONTROLLER_SWITCH,
                                      REACH_SWITCH,
                                      '--acf='+sys.argv[4]])

        start = time.time()

        while time.time()-start < 2:
            time.sleep(0.1)
        
        var_imped.kill()
        simulator.kill()
        logger_process.kill()
        skin_msg.kill()
        contact_memory.kill()

        # subprocess.Popen(['rosnode', 'kill', 'software_simulation_node'])
        # subprocess.Popen(['rosnode', 'kill', 'taxel_array_to_skin_contact'])
        # subprocess.Popen(['rosnode', 'kill', 'skin_contact_to_resultant_force'])
        # subprocess.Popen(['rosnode', 'kill', 'ode_log_and_monitor_node'])
        # subprocess.Popen(['rosnode', 'kill', 'contact_memory_node'])

        if check_files('./'):
            not_done = False
            check = True
        elif count >= 3:
            not_done = False

    os.system('rm running.txt')

    if check == False:
        if os.path.isfile('../unrun_trials.pkl'):
            data = ut.load_pickle("../unrun_trials.pkl")
            data['unrun_trials'].append(dir_name)
            ut.save_pickle(data, "../unrun_trials.pkl")
        else:
            data = {'unrun_trials':[]}
            data['unrun_trials'].append(dir_name)
            ut.save_pickle(data, "../unrun_trials.pkl")
