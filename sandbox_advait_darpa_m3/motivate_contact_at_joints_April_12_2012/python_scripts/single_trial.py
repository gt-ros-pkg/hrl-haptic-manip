
import numpy as np, math
import matplotlib.pyplot as pp
import sys

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu

import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa


# I don't have the force vector. Only plotting arm configuration and
# the contact location.
# loc_l - list of np arrays.
def plot_contact_state(q, loc_l, kinematics):
    kinematics.plot_arm(q, 'b', 0.7, flip_xy=True)
    loc_arr = np.row_stack(loc_l)
    x = loc_arr[:,0]
    y = loc_arr[:,1]
    pp.scatter(-y, x, linewidth=2, s=70, color='g', alpha=1.0,
               marker='+')
    pp.axis('equal')
    pp.xlim(-0.5, 0.5)
    pp.ylim(-0.1, 0.7)


def any_contact_at_joint(loc_arr, q, kinematics, dist_threshold):
    for p in loc_arr:
        if kinematics.is_contact_at_joint(np.matrix(p).T, q, dist_threshold):
            return True
    return False


def find_states_with_contact_at_joints(d, d_robot, kinematics):
    n = len(d['q_list'])
    q_arr = np.array(d['q_list'])
    all_forces_locs_arr = np.array(d['all_forces_locs_list'])
    num_contacts_arr = np.array(d['num_contacts_at_time_instant_list'])
    idx = 0
    dist_threshold = d_robot.dia / 2 + 0.002

    non_joint_contact_q_list = []
    non_joint_contact_loc_list = []
    joint_contact_q_list = []
    joint_contact_loc_list = []

    q_motion_thresh = math.radians(1.0)

    prev_q = None
    for i in xrange(n):
        q = q_arr[i]

        if i >= len(num_contacts_arr):
            break

        n_contacts = num_contacts_arr[i]
        if n_contacts == 0:
            continue
        loc_arr = all_forces_locs_arr[idx:idx+n_contacts]
        idx += n_contacts

        if prev_q == None:
            dist_q = q_motion_thresh
        else:
            dist_q = np.linalg.norm(q-prev_q)
        if dist_q < q_motion_thresh:
            continue
        prev_q = q

        if any_contact_at_joint(loc_arr, q, kinematics, dist_threshold):
            joint_contact_q_list.append(q)
            joint_contact_loc_list.append(loc_arr)
        else:
            non_joint_contact_q_list.append(q)
            non_joint_contact_loc_list.append(loc_arr)

    return joint_contact_q_list, joint_contact_loc_list, non_joint_contact_q_list, non_joint_contact_loc_list



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--dir', action='store', dest='direc',
                 type='string', default = None)
    p.add_option('--sim3', action='store_true', dest='sim3',
                 help='three link planar (torso, upper arm, forearm)')
    p.add_option('--sim3_with_hand', action='store_true', dest='sim3_with_hand',
                 help='three link planar (upper arm, forearm, hand)')

    opt, args = p.parse_args()

    if opt.direc == None:
        print 'Specify a root directory.'
        print 'Exiting ...'
        sys.exit()

    if opt.sim3:
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
    elif opt.sim3_with_hand:
        import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as d_robot
    else:
        print 'Please specify --sim3 or --sim3_with_hand'
        print 'Exiting ...'
        sys.exit()

    nm_l = ut.get_bash_command_output('ls '+opt.direc+'/reach_from*to_goal*log.pkl')
    kinematics = gsa.RobotSimulatorKDL(d_robot)

    for nm in nm_l:
        d = ut.load_pickle(nm)
        jt_q_l, jt_loc_l, non_jt_q_l, non_jt_loc_l = find_states_with_contact_at_joints(d, d_robot, kinematics)

        #for i, (q,locs) in enumerate(zip(jt_q_l, jt_loc_l)):
        #    mpu.figure()
        #    plot_contact_state(q, locs, kinematics)
        #    pp.savefig(str(i)+'.png')

        print 'Number of contact states with joint contacts:', len(jt_q_l)
        print 'Number of contact states WITHOUT joint contacts:', len(non_jt_q_l)





