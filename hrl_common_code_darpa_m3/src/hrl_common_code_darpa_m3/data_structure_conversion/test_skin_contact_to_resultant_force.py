
import matplotlib.pyplot as pp
import numpy as np, math

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')

import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa
import hrl_common_code_darpa_m3.data_structure_conversion.skin_contact_to_resultant_force as scrf
import hrl_lib.matplotlib_util as mpu


if __name__ == '__main__':
    print 'Hello World'

    import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot

    kinematics = gsa.RobotSimulatorKDL(d_robot)

    q = np.radians([30., -20., 50.])

    o, _ = kinematics.FK(q, 2)
    o_next, _ = kinematics.FK(q, 3)

    v = o_next - o

    #fc_mat = np.matrix([[2.,0.,0.]]).T
    #xc_mat = np.column_stack([o + 0.2 * v])

    fc_mat = np.matrix([[2.,0.,0.], [1., 1., 0.]]).T
    xc_mat = o + np.column_stack([0.2 * v, 0.4*v])

    mpu.figure()
    kinematics.plot_arm(q, 'b', 0.7, False)
    pp.axis('equal')

    pp.quiver(xc_mat[0,:].A1, xc_mat[1,:].A1, fc_mat[0,:].A1,
              fc_mat[1,:].A1, units='xy', width=0.005)

    f_res, loc = scrf.simulate_ft_sensor_on_link_base(fc_mat, xc_mat,
                                                      o, o_next)
    print 'f_res:', f_res.A1
    print 'xc_mat:', xc_mat.T.A
    print 'loc:', loc.A1

    pp.quiver(loc[0,:].A1, loc[1,:].A1, f_res[0,:].A1,
              f_res[1,:].A1, units='xy', width=0.003, color='r')

    pp.show()





