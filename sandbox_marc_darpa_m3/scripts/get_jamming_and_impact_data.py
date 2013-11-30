import roslib
roslib.load_manifest('hrl_software_simulation_darpa_m3')
import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as sim_robot_config
import hrl_software_simulation_darpa_m3.gen_sim_arms as sim_robot
import cPickle as pkl
import numpy as np

robot = sim_robot.ODESimArm(sim_robot_config)
data = pkl.load(file('/home/mkillpack/hrl_file_server/darpa_m3/final_ijrr_results_with_10_second_timeout/75_obstacle_configurations_movable000_fixed080/mpc_qs_1_using_skin/101_r02_t01/reach_log.pkl'))
#data = pkl.load(file('/home/mkillpack/hrl_file_server/darpa_m3/final_ijrr_results_with_10_second_timeout/75_obstacle_configurations_movable000_fixed020/mpc_qs_1_using_skin/51_r02_t00'))

f_thresh = 5
perc = 0.10

counter = 0

contact_vel = []
contact_force = []
print "length of q list :", len(data['q_list'])
print "length of num_contacts :",len(data['num_contacts_at_time_instant_list'])
print data.keys()

jamming_forces = []
jamming_normals = []
total_num_jamming = 0


for i in xrange(len(data['q_list'])):
    if data['num_contacts_at_time_instant_list'][i] > 1:
        flag = False
        force_buf = []
        nrml_buf = []
        for j in xrange(int(data['num_contacts_at_time_instant_list'][i])):
            #raw_input('in first loop')
            print "\n\n\n"
            print "j is :", j
            for k in xrange(j+1, int(data['num_contacts_at_time_instant_list'][i])):
                #raw_input('in second loop')
                print "k is :", k
                # print "do tproduct is ", np.dot(data['all_forces_nrmls_list'][counter], data['all_forces_nrmls_list'][counter+k])

                if np.dot(data['all_forces_nrmls_list'][counter], data['all_forces_nrmls_list'][counter+k]) < 0:
                    flag = True
                    raw_input("\n\n\n\n Starting again:")
                    print "data['all_forces_nrmls_list'][counter] :", data['all_forces_nrmls_list'][counter]
                    print "data['all_forces_nrmls_list'][counter+k] :", data['all_forces_nrmls_list'][counter+k]
                    print "do tproduct is ", np.dot(data['all_forces_nrmls_list'][counter], data['all_forces_nrmls_list'][counter+k])
                    print "flag is:  True"

            if data['all_forces_list'][counter] == data['all_forces_list'][counter-2]:
                print "!!!!!!!!!!!!!!!!!! IT'S TRUE !!!!!!!!!!!!!!!!!!!!!!!!"
            force_buf.append(data['all_forces_list'][counter])
            nrml_buf.append(data['all_forces_nrmls_list'][counter])
            counter = counter + 1
        if flag == True:
            print "force_buf is :", force_buf
            print "normals are :", nrml_buf
            jamming_forces.append(force_buf)
            jamming_normals.append(nrml_buf)
            total_num_jamming = total_num_jamming + data['num_contacts_at_time_instant_list'][i]
        flag = False

print "total_num_jamming :", total_num_jamming
print "sum of all contacts :", np.sum(data['num_contacts_at_time_instant_list'])
print "sum of 2 contacts :", len(np.where(np.array(data['num_contacts_at_time_instant_list']) == 2)[0])*2




# ['qdot_list',
#  'torso_rotation',
#  'all_forces_list',
#  'max_force_list',
#  'jep_list',
#  'local_goal',
#  'all_forces_locs_list',
#  'taxel_hist',
#  '__version__',
#  'ee_pos_list',
#  'all_forces_jts_list',
#  'jep_send_rate',
#  'torso_position',
#  'controller',
#  'num_contacts_at_time_instant_list',
#  'q_list',
#  'all_forces_nrmls_list',
#  'ee_gaussian_50_cov_list',
#  'ee_gaussian_50_mn_list',
#  'time_stamp_list',
#  '__header__',
#  '__globals__',
#  'mean_motion_list']
