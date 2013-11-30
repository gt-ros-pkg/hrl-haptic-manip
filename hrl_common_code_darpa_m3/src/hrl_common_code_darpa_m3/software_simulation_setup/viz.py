
#
# Functions to visualize a reach_problem_dict
#
#

import sys
import numpy as np, math
import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu


# X coordinate of obstacle is along the Y-axis of matplotlib and Y
# coordinate of the obstacles increase to the left.
def draw_obstacles(pos_list, dimen_list, color):
    if len(pos_list) == 0:
        return
    arr = np.array(pos_list)
    pos_arr = arr[:,0:2]
    rad_arr = (np.array(dimen_list)[:,0]).flatten()
    for i in range(len(rad_arr)):
        cx, cy = -pos_arr[i,1], pos_arr[i,0]
        mpu.plot_circle(cx, cy, rad_arr[i], 0., math.pi*2, color=color)
        #mpu.plot_circle(cx, cy, rad_arr[i], 0., math.pi*2,
        #                color='k', linewidth=0.5)

def draw_obstacles_from_reach_problem_dict(d):
    n = d['num_fixed_used']
    draw_obstacles(d['fixed_position'][0:n], d['fixed_dimen'][0:n], color='r')
    n = d['num_move_used']
    draw_obstacles(d['moveable_position'][0:n], d['moveable_dimen'][0:n], color='#A0A0A0')
    pp.xlabel('Negative Y-coordinate in the robot\'s frame')
    pp.ylabel('X-coordinate in the robot\'s frame')

def draw_goal_from_reach_problem_dict(d):
    g = d['goal']
    pp.scatter([-g[1]], [g[0]], s=100, c='g', marker='x', lw=3,
            edgecolor='g')

def viz_reach_problem_dict(d, kinematics):
    draw_obstacles_from_reach_problem_dict(d)

#    q = np.radians([60, 0, 0])
#    kinematics.plot_arm(q, color='b', alpha=0.7, flip_xy=True,
#                        linewidth = 0.5)

    q = np.radians([90, 0, 0])
    kinematics.plot_arm(q, color='b', alpha=0.7, flip_xy=True,
                        linewidth = 0.5)

    #    q = np.radians([120, 0, 0])
    #    kinematics.plot_arm(q, color='b', alpha=0.7, flip_xy=True,
    #                        linewidth = 0.5)

    draw_goal_from_reach_problem_dict(d)



if __name__ == '__main__':
    import optparse

    # if we are using this file, then we need to have
    # hrl_software_simulation_darpa_m3
    roslib.load_manifest('hrl_software_simulation_darpa_m3')
    import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa

    p = optparse.OptionParser()

    p.add_option('--problem_dict_pkl', '--pdp', action='store', dest='pdp',
                 type='string', default = None,
                 help='reach problem dict pkl file')
    p.add_option('--quiet', '-q', action='store_true', dest='q',
                 help='do not bring up matplotlib window')
    p.add_option('--save_figure', '--sf', action='store_true', dest='sf',
                 help='save the figure')
    p.add_option('--sim3', action='store_true', dest='sim3',
                 help='three link planar (torso, upper arm, forearm)')
    p.add_option('--sim3_with_hand', action='store_true', dest='sim3_with_hand',
                 help='three link planar (upper arm, forearm, hand)')

    opt, args = p.parse_args()

    if opt.sim3:
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
    elif opt.sim3_with_hand:
        import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as d_robot

    kinematics = gsa.RobotSimulatorKDL(d_robot)

    if not opt.pdp:
        raise RuntimeError('Please specify a reach problem dict file')

    d = ut.load_pickle(opt.pdp)

    mpu.figure()
    viz_reach_problem_dict(d, kinematics)

    if opt.sf:
        nm = '.'.join(opt.pdp.split('.pkl')[0:-1]) + '.png'
        pp.savefig(nm)

    if not opt.q:
        pp.show()





