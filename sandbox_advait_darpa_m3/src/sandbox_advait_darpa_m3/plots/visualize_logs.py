
#
# visualize data from the pkls logged by the contact_memory and
# logging_and_monitoring nodes.
#


import sys, os
import matplotlib.pyplot as pp
import numpy as np, math


import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import mid_level_control.contact_memory as cm
import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu
import hrl_common_code_darpa_m3.software_simulation_setup.viz as sssv

#============ data logged by the contact_memory node ===========

# cm_list - contact_memory list (what the contact memory node saves.)
def plot_contact_memory(cm_dict):
    cm_list = cm_dict['contact_memory_list']
    torso_trans = cm_dict['last_torso_position']
    torso_rot = cm_dict['last_torso_rotation']

    pp.axis('equal')
    for i, tr_dict in enumerate(cm_list):
        ch = tr_dict['convex_hull']
        pts_world = np.matrix(ch).T
        # rotating to last torso coodr frame. origin is still at the
        # world frame.
        pts = torso_rot[0:2,0:2].T * pts_world
        pp.plot(pts[0,:].A1, pts[1,:].A1, '.-', color = mpu.random_color(),
                linewidth=1, ms=5, label='Obstacle %d'%(i+1))
    pp.legend()

def examine_stiffness_estimation(cm_dict, contact_to_examine):
    cm_list = cm_dict['contact_memory_list']
    torso_trans = cm_dict['last_torso_position']
    torso_rot = cm_dict['last_torso_rotation']

    d = cm_list[contact_to_examine]
    all_locs = torso_rot.T * np.matrix(d['loc_list']).T

    n_pts = 10
    n_start = 0
    locs = torso_rot.T * np.matrix(d['loc_list'][n_start : n_start+n_pts]).T
    forces = torso_rot.T * np.matrix(d['force_list'][n_start : n_start+n_pts]).T
    nrml = torso_rot.T * np.matrix(d['contact_normals_list'][n_start+n_pts-1]).T

    pp.axis('equal')
    pp.plot(all_locs[0,:].A1, all_locs[1,:].A1, '.-', linewidth=1,
            ms=5, color = 'b')
    pp.plot(locs[0,:].A1, locs[1,:].A1, '.-', linewidth=1, ms=5,
            color = 'y')

    cm.estimate_stiffness(locs, forces, nrml, visualize=True)

    esl = d['estimated_stiffness_list']
    mpu.figure()
    pp.plot(esl)
    pp.title('Online Estimates of the stiffness')


#============ data logged by the log_and_monitor_node ============

def plot_distance_to_goal(kinematics, common_log_dict, label):
    q_list = common_log_dict['q_list']
    if q_list == []:
        return
    ee_pos_l = [kinematics.FK(q)[0] for q in q_list]
    ee_pos_mat = np.column_stack(ee_pos_l)
    local_goal = common_log_dict['local_goal']

    time_l = common_log_dict['time_stamp_list']
    time_l = [t-time_l[0] for t in time_l]
    
    dist_from_goal = ut.norm((ee_pos_mat - local_goal)[0:2,:]).A1
    pp.plot(time_l, dist_from_goal, '.-', color=mpu.random_color(),
            linewidth=1, ms=5, label=label)
    pp.legend()

def plot_ee_trajectory(kinematics, common_log_dict, label, color = None,
                       alpha = 1.):
    q_list = common_log_dict['q_list']
    t_pos = common_log_dict['torso_position']
    t_rot = common_log_dict['torso_rotation']

    ee_pos_l = [t_pos + t_rot*kinematics.FK(q)[0] for q in q_list]
    if ee_pos_l == []:
        print 'ee_pos_l is empty.'
        return

    pp.axis('equal')

    ee_pos_mat = np.column_stack(ee_pos_l)

    # plot end effector trajectory.
    ee_x = -ee_pos_mat[1,:].A1
    ee_y = ee_pos_mat[0,:].A1
    if color == None:
        color = mpu.random_color()

    pp.plot(ee_x, ee_y, '.-', color = color,
            #linewidth=1, ms=5, label=label)
            linewidth=2, ms=0, label=label, alpha = alpha)

    pp.legend()

def plot_cep_trajectory(kinematics, common_log_dict, label):
    jep_list = common_log_dict['jep_list']
    t_pos = common_log_dict['torso_position']
    t_rot = common_log_dict['torso_rotation']

    cep_pos_l = [t_pos + t_rot*kinematics.FK(jep)[0] for jep in jep_list]
    if cep_pos_l == []:
        print 'cep_pos_l is empty.'
        return

    pp.axis('equal')

    cep_pos_mat = np.column_stack(cep_pos_l)
    print 'cep_pos_mat.shape:', cep_pos_mat.shape

    # plot end effector trajectory.
    cep_x = -cep_pos_mat[1,:].A1
    cep_y = cep_pos_mat[0,:].A1
    pp.plot(cep_x, cep_y, '.-', color = mpu.random_color(),
            linewidth=1, ms=5, label=label)

    pp.legend()

def plot_force_histogram(f_mag_l, label):
    max_force = max(f_mag_l)
    bin_width = 0.5
    bins = np.arange(0.-bin_width/2., max_force+2*bin_width, bin_width)
    hist, bin_edges = np.histogram(np.array(f_mag_l), bins)
    mpu.plot_histogram(bin_edges[:-1]+bin_width/2., hist,
                       width=bin_width*0.8)
    pp.title('Histogram of contact force magnitudes: '+label)
    pp.xlabel('contact force (N)')
    pp.ylabel('bin count')

    print 'mean force:', np.mean(f_mag_l)

def compare_qp_baseline_force_histogram(save_force_lists):
    # combined force histograms for simple and qp trials
    all_forces_simple = []
    max_forces_simple = []
    all_forces_qp = []
    max_forces_qp = []
    for pkl in pkl_list:
        pkl_nm_split = '.'.join((pkl.split('/')[-1]).split('.')[0:-1])
        print 'pkl:', pkl

        if 'reach_from' not in pkl:
            continue

        d = ut.load_pickle(pkl)
        if 'controller' in d:
            f_list = d['all_forces_list']
            if 'simple' in pkl_nm_split:
                all_forces_simple = all_forces_simple + f_list
                max_forces_simple.append(max(f_list))
            if 'qp' in pkl_nm_split:
                all_forces_qp = all_forces_qp + f_list
                max_forces_qp.append(max(f_list))

    if save_force_lists:
        aggregated_forces_dict = {}
        aggregated_forces_dict['all_qp'] = all_forces_qp
        aggregated_forces_dict['all_simple'] = all_forces_simple
        aggregated_forces_dict['max_qp'] = max_forces_qp
        aggregated_forces_dict['max_simple'] = max_forces_simple
        ut.save_pickle(aggregated_forces_dict, 'aggregated_forces_dict.pkl')

    mpu.figure()
    plot_force_histogram(all_forces_qp, 'QP')
    mpu.figure()
    plot_force_histogram(all_forces_simple, 'simple')

#=========== example way to use the functions in this file ===========
def the_usual():
    for pkl in pkl_list:
        pkl_nm_split = '.'.join((pkl.split('/')[-1]).split('.')[0:-1])
        print 'pkl:', pkl
        #if 'to_goal' not in pkl and 'pull_out' not in pkl or '3' not in pkl:
        #    continue

        #if not ('reach_from' in pkl or 'pull_out' in pkl):
        #    continue

        if not ('reach_from' in pkl):
            continue

        d = ut.load_pickle(pkl)
        if 'controller' in d:
            #plot_ee_trajectory(kinematics, d, pkl_nm_split)
            #kinematics.plot_arm(d['q_list'][0], color='b', alpha=1.0, flip_xy=True)
            #plot_cep_trajectory(kinematics, d, pkl_nm_split)

            #f_mag_l = d['all_forces_list']
            #plot_force_histogram(f_mag_l, pkl_nm_split)

            base_pos = pkl_nm_split.split('=')[0]
            plot_distance_to_goal(kinematics, d, base_pos)

        if 'contact_memory_list' in d:
            plot_contact_memory(d)
            mpu.figure()
            examine_stiffness_estimation(d, 2)

#======== Specific cases of Advait visualizing and debugging =========
# These functions will become useless fairly rapidly but might be good
# sources of sample code.
#=====================================================================
def current_mid_level_is_inefficient():
    d = ut.load_pickle(opt.rpd)
    sssv.draw_obstacles_from_reach_problem_dict(d)

    goal = d['goal']
    pp.plot([goal[0]], [goal[1]], 'gx', ms=15, mew=4)
    
    #color_list = [mpu.random_color() for i in range(5)]
    color_list = ['#9697BB', '#180647', '#A621ED', '#9E6A5C', '#4324D3']
    #print 'color_list:', color_list

    for pkl in pkl_list:
        pkl_nm_split = '.'.join((pkl.split('/')[-1]).split('.')[0:-1])

        if not('to_goal' in pkl or 'side' in pkl):
            continue

        if 'reach_from' not in pkl:
            continue
        #if 'pos_3' not in pkl:
        #    continue

        d = ut.load_pickle(pkl)
        if 'controller' in d:
            dy = pkl_nm_split.split('_')[5]
            if dy == '0.00':
                color = color_list[0]
                alpha = 1.0
            if dy == '-0.05':
                color = color_list[1]
                alpha = 1.0
            if dy == '-0.12':
                color = color_list[2]
                alpha = 1.0
            if dy == '-0.17':
                color = color_list[3]
                alpha = 1.0
            if dy == '-0.22':
                color = color_list[4]
                alpha = 1.0

            if 'retry_right' in pkl and 'to_goal_1' in pkl:
                q_list = d['q_list']
                kinematics.plot_arm(q_list[0], color=color,
                                    alpha=alpha)

            plot_ee_trajectory(kinematics, d, '__no_legend__',
                               color=color, alpha=alpha)

def debugging_pull_out_fail():
    for pkl in pkl_list:
        pkl_nm_split = '.'.join((pkl.split('/')[-1]).split('.')[0:-1])

        if not ('reach_from' in pkl or 'pull_out' in pkl):
            continue

        d = ut.load_pickle(pkl)
        if 'controller' in d:
            plot_ee_trajectory(kinematics, d, pkl_nm_split)

        if 'pull_out_0_1=elbow_log' in pkl:
            q = d['q_list'][-1]
            kinematics.plot_arm(q, 'b', 1.0, True)



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--dir', action='store', dest='direc',
                 type='string', default = None,
                 help='directory with all the logs from one switched control trial.')
    p.add_option('--reach_problem_dict', '--rpd', action='store', dest='rpd',
                 type='string', default = None,
                 help='reach problem dict pkl file')
    p.add_option('--cody', action='store_true', dest='cody',
                 help='visualize cody logs')
    p.add_option('--sim3', action='store_true', dest='sim3',
                 help='visualize software simulation logs')
    p.add_option('--sim3_with_hand', action='store_true',
            dest='sim3_with_hand',
                 help='visualize software simulation logs')
    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)

    opt, args = p.parse_args()

    if opt.rpd:
        d = ut.load_pickle(opt.rpd)
        sssv.draw_obstacles_from_reach_problem_dict(d)


    if opt.cody:
        import hrl_cody_arms.cody_arm_kinematics as cak

        if opt.arm == None:
            print 'Please specify an arm to use. Exiting...'
            sys.exit()

        kinematics = cak.CodyArmKinematics(opt.arm)
        #kinematics.set_tooltip(np.matrix([0.,0.,-0.16]).T) # tube
        #kinematics.set_tooltip(np.matrix([0.,0.,-0.01]).T) # stub
        kinematics.set_tooltip(np.matrix([0.,0.,-0.04]).T) # stub with mini45

    else:
        import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa
        if opt.sim3:
            import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
        elif opt.sim3_with_hand:
            import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as d_robot

        kinematics = gsa.RobotSimulatorKDL(d_robot)
        sssv.viz_reach_problem_dict(d, kinematics)

    if opt.direc == None:
        print 'Please provide a directory'
        print 'Exiting ...'
        sys.exit()

    # want pkl_list in the order in which they were saved to disk.
    pkl_list = ut.get_bash_command_output('ls -t '+opt.direc+'/*.pkl')
    pkl_list.reverse()

    
    the_usual()

    #debugging_pull_out_fail()
    #current_mid_level_is_inefficient()
    #compare_qp_baseline_force_histogram(save_force_lists = True)

    pp.show()




