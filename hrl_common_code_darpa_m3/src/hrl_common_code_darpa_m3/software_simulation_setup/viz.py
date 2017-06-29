
#
# Functions to visualize a reach_problem_dict
#
#

import sys
import numpy as np, math
import matplotlib.pyplot as pp
from mpl_toolkits.mplot3d import Axes3D as ax3d

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu


# X coordinate of obstacle is along the Y-axis of matplotlib and Y
# coordinate of the obstacles increase to the left.
def draw_obstacles(ctype_list, pos_list, dimen_list, color, dim):
    if len(pos_list) == 0:
        return
    pos_arr = np.array(pos_list)

    for i in range(len(ctype_list)):
        cx, cy, cz = -pos_arr[i,1], pos_arr[i,0], pos_arr[i,2]

        if dim == 2:
            if ctype_list[i] == 'wall':
                # x,y is exchanged
                length = dimen_list[i][1]
                width  = dimen_list[i][0]
                slope  = pos_arr[i,3] 

                mpu.plot_rectangle(cx, cy, slope, width, length, color=color)
            else:
                radius = dimen_list[i][0]
                mpu.plot_circle(cx, cy, radius, 0., math.pi*2, color=color)    
                #mpu.plot_circle(cx, cy, rad_arr[i], 0., math.pi*2,
                #                color='k', linewidth=0.5)
        else:
            # Spheres
            radius = dimen_list[i][0]
            ax = pp.gca()
            ax.scatter(cx, cy, cz, s=20, c=color, linewidth=0.5 )

def draw_obstacles_from_reach_problem_dict(d,dim=2):
    draw_obstacles(d['fixed_ctype'], d['fixed_position'], d['fixed_dimen'], color='r', dim=dim)
    draw_obstacles(d['moveable_ctype'], d['moveable_position'], d['moveable_dimen'], color='#A0A0A0', dim=dim)
            
    pp.xlabel('Negative Y-coordinate in the robot\'s frame', fontsize=10)
    pp.ylabel('X-coordinate in the robot\'s frame', fontsize=10)

    if dim == 3:
        pp.ylabel('Z-coordinate in the robot\'s frame', fontsize=10)
    
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





