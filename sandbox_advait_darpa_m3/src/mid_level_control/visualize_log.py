

import numpy as np, math
import sys
import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy

import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu


##
# q - joint angles.
# fvec_l - 3xN np matrix
# loc_mat - 3xN np matrix
def visualize_contact_state(kinematics, q, fvec_mat, loc_mat, alpha=0.8):
    kinematics.plot_arm(q, 'b', alpha, flip_xy=True)
    if fvec_mat.shape[1] > 0:
        pp.quiver(-loc_mat[1,:].A1, loc_mat[0,:].A1,
                  -fvec_mat[1,:].A1, fvec_mat[0,:].A1,
                  units='xy', width=0.004, color='k', scale=30.)
    pp.axis('equal')

##
# log_dict - dict saved by the log_and_monitor node
def visualize_final_contact_state(kinematics, log_dict, alpha=0.8):
    q = log_dict['q_list'][-1]
    n_contact_arr = np.array(log_dict['num_contacts_at_time_instant_list'])
    idx_arr = np.cumsum(n_contact_arr) - n_contact_arr
    idx = int(idx_arr[-1])
    n = int(n_contact_arr[-1])

    f_mag_arr = np.array(log_dict['all_forces_list'][idx:idx+n])
    loc_mat = np.matrix(log_dict['all_forces_locs_list'][idx:idx+n]).T
    nrml_arr = np.array(log_dict['all_forces_nrmls_list'][idx:idx+n]).T

    if nrml_arr.shape[0] !=0 and (nrml_arr.shape[1] != f_mag_arr.shape[0]):
        rospy.logwarn('Something fishy here. nrml_arr.shape[1] != f_mag_arr.shape[0]. Ignoring')
        fvec_mat = np.matrix([])
    else:
        fvec_mat = np.matrix(nrml_arr * f_mag_arr)

    visualize_contact_state(kinematics, q, fvec_mat, loc_mat,
                            alpha=alpha)


def plot_end_effector_trajectory(log_dict, color):
    ee_pos_arr = np.array(log_dict['ee_pos_list'])
    pp.plot(-ee_pos_arr[:,1], ee_pos_arr[:,0], c=color)


def plot_initial_final_and_path(kinematics, log_dict):
    q0 = log_dict['q_list'][0]
    kinematics.plot_arm(q0, 'b', 0.7, flip_xy=True)
    visualize_final_contact_state(kinematics, log_dict, alpha=0.2)
    plot_end_effector_trajectory(log_dict, 'b')

if __name__ == '__main__':
    import hrl_common_code_darpa_m3.robot_config.three_link_planar_cuboid as d_robot
    import optparse
    p = optparse.OptionParser()
    p.add_option('--log_pkl', action='store', dest='log_pkl',
                 type='string', help='pkl with logged data',
                 default=None)
    opt, args = p.parse_args()

    if opt.log_pkl == None:
        rospy.logerror('Please provide a pkl with the logged data.')
        sys.exit()
    
    kinematics = gsa.RobotSimulatorKDL(d_robot)

    mpu.figure()
    log_dict = ut.load_pickle(opt.log_pkl)

    #visualize_final_contact_state(kinematics, log_dict)
    plot_initial_final_and_path(kinematics, log_dict)

    pp.show()






