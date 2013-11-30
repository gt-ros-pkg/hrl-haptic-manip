
import numpy as np, math
import matplotlib.pyplot as pp

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')

import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa
import hrl_lib.geometry as hg



if __name__ == '__main__':

    import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as d_robot


    kine = gsa.RobotSimulatorKDL(d_robot)

    q = np.radians([0, 0, 0])
    kine.plot_arm(q, 'b', 0.1, flip_xy=True)
    q = np.radians([90, 0, 0])
    kine.plot_arm(q, 'b', 0.1, flip_xy=True)
    q = np.radians([-10, 0, 0])
    kine.plot_arm(q, 'b', 0.1, flip_xy=True)
    q = np.radians([120, 0, 0])
    kine.plot_arm(q, 'b', 0.1, flip_xy=True)

    q = np.radians([15, 38, -21])
    q = np.radians([0, 90, -90])
    kine.plot_arm(q, 'b', 0.8, flip_xy=True)

    pts_list = kine.arm_config_to_points_list(q)

    for d in np.arange(0.1, 0.75, 0.005):
        pt = hg.get_point_along_curve(pts_list, d) + np.matrix([0.02, 0.02]).T
        if kine.is_contact_at_joint(pt, q, d_robot.dia / 2. + 0.01):
            c = 'r'
        else:
            c = 'y'
        pp.scatter([-pt[1,0]], [pt[0,0]], s=10, color=c, linewidth=0)

    pp.axis('equal')
    pp.show()







