#!/usr/bin/python

import sys
import numpy as np, math
import copy
import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu
import hrl_common_code_darpa_m3.software_simulation_setup.viz as sssv




if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--nr', action='store', dest='nr',type='int',
                 default=3, help='number of goals in radial direction')
    p.add_option('--nt', action='store', dest='nt',type='int',
                 default=3, help='number of goals in theta direction')

    p.add_option('--rmin', action='store', dest='rmin',type='float',
                 default=0.5, help='min radial distance for goal')
    p.add_option('--rmax', action='store', dest='rmax',type='float',
                 default=0.7, help='max x radial distance for goal')

    p.add_option('--tmin', action='store', dest='tmin',type='float',
                 default=-30, help='min theta for goal (DEGREES)')
    p.add_option('--tmax', action='store', dest='tmax',type='float',
                 default=30, help='max theta for goal (DEGREES)')

    p.add_option('--pkl', action='store', dest='pkl', default=None,
                 help='pkl with obstacle locations')

    p.add_option('--save_figure', '--sf', action='store_true', dest='sf',
                 help='save the figure')
    p.add_option('--sim3', action='store_true', dest='sim3',
                 help='three link planar (torso, upper arm, forearm)')
    p.add_option('--sim3_with_hand', action='store_true', dest='sim3_with_hand',
                 help='three link planar (upper arm, forearm, hand)')

    p.add_option('--grid_goal', action='store', dest='grid_resol', type='float',
                 default=0.0, help='Grid Goal resolution for equaly distributed goals')
    p.add_option('--xmin', action='store', dest='xmin',type='float',
                 default=0.2, help='min x coord for goals')
    p.add_option('--xmax', action='store', dest='xmax',type='float',
                 default=0.6, help='max x coord for goals')
    p.add_option('--ymin', action='store', dest='ymin',type='float',
                 default=-0.3, help='min y coord for goals')
    p.add_option('--ymax', action='store', dest='ymax',type='float',
                 default=0.3, help='max y coord for goals')

    opt, args = p.parse_args()

    if opt.pkl == None:
        raise RuntimeError('Please specify a reach_problem_dict pkl')

    rpd = ut.load_pickle(opt.pkl)
    nm = '.'.join(opt.pkl.split('.')[0:-1])
    g_list = []

    if opt.grid_resol:

        x = opt.xmin
        y = opt.ymin

        arGridX = []
        while x <= opt.xmax:
            arGridX.append(x)
            x += opt.grid_resol

        arGridY = []
        while y <= opt.ymax:
            arGridY.append(y)
            y += opt.grid_resol

        for i in range(len(arGridX)):
            for j in range(len(arGridY)):
                rpd['goal'] = [arGridX[i], arGridY[j], 0]
                ut.save_pickle(rpd, nm + '_x%02d'%i + '_y%02d'%j + '.pkl')
                g_list.append(copy.copy(rpd['goal']))

    else:
        # Prevent divided by zero
        if opt.nr != 1 :
            r_step = (opt.rmax - opt.rmin) / (opt.nr - 1)
        else:
            r_step = 0

        if opt.nt != 1 :
            t_step = math.radians((opt.tmax - opt.tmin) / (opt.nt - 1))
        else:
            t_step = 0

        t_start = math.radians(opt.tmin)
        nt = opt.nt

        for r in range(opt.nr):
            for t in range(nt):
                rad = opt.rmin + r_step * r
                theta = t_start + t_step * t
                rpd['goal'] = [rad * math.cos(theta), rad * math.sin(theta), 0]
                ut.save_pickle(rpd, nm + '_r%02d'%r + '_t%02d'%t + '.pkl')
                g_list.append(copy.copy(rpd['goal']))

            if r%2 == 0:
                t_start = t_start + t_step/2
                nt = nt - 1
            else:
                t_start = t_start - t_step/2
                nt = nt + 1

    if opt.sf:
        # if we are using this file, then we need to have
        # hrl_software_simulation_darpa_m3
        roslib.load_manifest('hrl_software_simulation_darpa_m3')
        import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa

        if opt.sim3:
            import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
        elif opt.sim3_with_hand:
            import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as d_robot

        mpu.set_figure_size(6,4)
        pp.figure()
        kinematics = gsa.RobotSimulatorKDL(d_robot)
        sssv.draw_obstacles_from_reach_problem_dict(rpd)
        g_arr = np.array(g_list)
        pp.scatter(-g_arr[:,1], g_arr[:,0], s=50, c='g', marker='x', lw=1, edgecolor='g')

        q = [0.,0,0]
        ee,_ = kinematics.FK(q)
        rad = np.linalg.norm(ee)
        sa = -math.radians(45)
        ea = math.radians(45)
        mpu.plot_circle(0., 0., rad, sa, ea, color='b', linewidth=0.5)
        mpu.plot_radii(0., 0., rad, sa, ea, 2*math.pi, color='b', linewidth=0.5)

        pp.xlim(-0.7, 0.7)
        mpu.reduce_figure_margins(left=0.02, bottom=0.02, right=0.98, top=0.98)
        pp.savefig(nm+'.pdf')



