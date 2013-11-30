
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as pp

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')

import hrl_lib.util as ut
import hrl_motion_planners_darpa_m3.planar_openrave as po
import hrl_common_code_darpa_m3.robot_config.three_link_planar as robot_defn
import hrl_lib.matplotlib_util as mpu


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--reach_problem_dict', '--rpd', action='store',
                 dest='rpd', type='string', default=None,
                 help='reach problem dict pkl')
    p.add_option('--ignore_movable', '--im', action='store_true', dest='im',
                 help='do not use movable obstacles.')
    p.add_option('--save_plot', '--sp', action='store_true', dest='sp',
                 help='save plot with initial and final arm configuration.')
    p.add_option('--safety_margin', action='store',
                 dest='safety_margin', type='float', default=0.,
                 help='Safety margin added to radius of all obstacles')

    opt, args = p.parse_args()

    rpd = ut.load_pickle(opt.rpd)

    for dim in rpd['moveable_dimen']:
        dim[0] += opt.safety_margin
        dim[1] += opt.safety_margin

    for dim in rpd['fixed_dimen']:
        dim[0] += opt.safety_margin
        dim[1] += opt.safety_margin

    env = po.setup_openrave(rpd, robot_defn, opt.im, True)

    # need to manually match this with the parameter used with the
    # feedback controllers.
    stopping_dist = 0.025
    res_dict = po.plan_with_stopping_dist_from_goal(stopping_dist, rpd['goal'],
                                                    env, robot_defn.b_jt_start,
                                                    True, 'BiRRT')
    #res_dict = po.plan_path_to_goal(rpd['goal'], env, robot_defn.b_jt_start,
    #                                True, 'BiRRT')
    env.Destroy()

    res_pkl = 'openrave_'+'_'.join(opt.rpd.split('/')[-1].split('_')[3:])
    ut.save_pickle(res_dict, res_pkl)

    if opt.sp:
        import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa
        import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as d_robot
        import hrl_common_code_darpa_m3.software_simulation_setup.viz as sssv

        mpu.figure()
        kinematics = gsa.RobotSimulatorKDL(d_robot)

        sssv.viz_reach_problem_dict(rpd, kinematics)

        q = robot_defn.b_jt_start
        kinematics.plot_arm(q, color='b', alpha=1., flip_xy=True)

        q = res_dict['final_q']
        kinematics.plot_arm(q, color='b', alpha=1., flip_xy=True)

        nm = '.'.join(res_pkl.split('.')[0:-1]) + '.png'
        pp.savefig(nm)


