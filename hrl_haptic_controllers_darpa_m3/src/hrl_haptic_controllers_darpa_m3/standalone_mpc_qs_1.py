
import numpy as np, math
import time
import sys

import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('hrl_haptic_controllers_darpa_m3')

import rospy
import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu
import hrl_haptic_controllers_darpa_m3.epc_skin_math as esm


def single_step_qp(d, epcon):
    J_h = d['J_h']
    Jc_l = d['Jc_l']
    Kc_l = [Kc for Kc in d['Kc_l']]

    Kc_l[0] = Kc_l[0] * 0.001

    Rc_l = d['Rc_l']
    delta_f_min = d['delta_f_min']
    delta_f_max = d['delta_f_max'] * 10.

#    delta_f_max[6,0] = -0.1
#    delta_f_max[1,0] = -0.01

    print 'delta_f_min:', delta_f_min.A1
    print 'delta_f_max:', delta_f_max.A1

    phi_curr = d['phi_curr']
    delta_x_g = d['delta_x_g'] * 1.

    delta_phi_max_deg = d['delta_phi_max_deg']
    K_j = d['K_j']

    max_delta_x_h = d['max_delta_x_h']

    n_faces = d['n_faces']

    loc_l = d['loc_l']
    n_l = d['n_l']

    jerk_opt_weight = 0.0000001

    P0_l, P1, P2, P3, P4_l = esm.P_matrices(J_h, K_j, Kc_l, Jc_l)
    D2, D3, D4, D5, D6 = esm.D_matrices(delta_x_g, delta_f_min, 
                                        K_j, Rc_l, P3, P4_l)

    delta_theta_min, delta_theta_max = self.joint_limit_bounds(theta_curr)
    D7 = P2 * K_j

    if num_joints == 3:
        # taking squared magnitude of JEP change in software
        # simulation to tackle cases where I vary joint
        # stiffness by a lot.
        K_j_t = np.matrix(np.eye(3))
    else:
        K_j_t = K_j

    min_jerk_mat = esm.min_jerk_quadratic_matrix(jerk_opt_weight, K_j)

    cost_quadratic_matrices = [D2, min_jerk_mat]
    cost_linear_matrices = [D3]

    constraint_matrices = [D4, D5, A3]
    constraint_vectors = [delta_f_max, D6, b3]

    #constraint_matrices = [D4, A3]
    #constraint_vectors = [delta_f_max, b3]

    lb = delta_phi_min
    ub = delta_phi_max
    
    t0 = time.time()
    delta_phi_opt, opt_error, feasible = epcon.solve_qp(cost_quadratic_matrices, 
                                                      cost_linear_matrices, 
                                                      constraint_matrices, 
                                                      constraint_vectors, 
                                                      lb, ub, 
                                                      False)
    t1 = time.time()
    print 'time to optimize:', t1-t0

    print '------------- Inputs -----------------'
    print

    print 'delta_x_g:', delta_x_g.A1
    print 'delta_f_max:', delta_f_max.A1
    print 'delta_f_min:', delta_f_min.A1
    print 'max_delta_x_h:', max_delta_x_h
    print 'delta_phi_min:', np.degrees(delta_phi_min.A1)
    print 'delta_phi_max:', np.degrees(delta_phi_max.A1)
    print 'Kc_l[0]:', Kc_l[0]

    print

    print '---------- Optimization Result ---------------'
    print
    print 'delta_phi_opt:', np.degrees(delta_phi_opt.A1)
    print 'J_h * delta_phi_opt:', (J_h * delta_phi_opt).A1
    print 'feasible?', feasible
    print
    delta_theta = P2 * K_j * delta_phi_opt
    delta_x_h = J_h * delta_theta
    print 'delta_x_h:', delta_x_h.A1

    delta_x_c = Jc_l[0] * delta_theta
    print 'Jc * delta_theta:', delta_x_c.A1
    print 'delta_fc:', (Kc_l[0] * delta_x_c).A1
    print 'n_l:', n_l[0].A1
    print
    print

    
    print '---------------------------------------------'
    print 'Jacobian pseudo inverse solution:'
    print '---------------------------------------------'
    J_h_temp = J_h[:,0:2]
    d_q_temp = np.linalg.pinv(J_h_temp) * delta_x_g
    d_q = np.matrix([d_q_temp[0,0], d_q_temp[1,0], 0.]).T
    print 'd_q:', np.degrees(d_q.A1)
    print 'J_h * d_q:', (J_h * d_q).A1

    delta_theta = P2 * K_j * d_q
    delta_x_h = J_h * delta_theta
    print 'delta_x_h:', delta_x_h.A1

    delta_x_c = Jc_l[0] * delta_theta
    print 'Jc * delta_theta:', delta_x_c.A1
    print 'delta_fc:', (Kc_l[0] * delta_x_c).A1
    print 'n_l:', n_l[0].A1
    print
    print





if __name__ == '__main__':
    import optparse

    p = optparse.OptionParser()

    p.add_option('--cody', action='store_true', dest='cody',
                 help='task monitoring for cody')
    p.add_option('--sim', action='store_true', dest='sim',
                 help='software simulation')
    p.add_option('--pkl', action='store', dest='pkl',
                 type='string', default = None,
                 help='pkl saved by the controller.')
    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)

    opt, args = p.parse_args()

    skin_topic = '/skin/contacts'
    
    if opt.pkl == None:
        print 'Please specify a pkl'
        print 'Exiting ...'
        sys.exit()

    if opt.sim:
        roslib.load_manifest('hrl_software_simulation_darpa_m3')
        import hrl_software_simulation_darpa_m3.ode_sim_arms as osa
        import hrl_software_simulation_darpa_m3.ode_sim_guarded_move as sgm

        robot = osa.ODESimArm()
        scl = sgm.ode_SkinClient(skin_topic)

    if opt.cody:
        import hrl_cody_arms.cody_arm_client as cac
        import sandbox_advait.cody.cody_guarded_move as cgm

        if opt.arm == None:
            rospy.logerr('Need to specify --arm_to_use.\nExiting...')
            sys.exit()

        robot = cac.CodyArmClient(opt.arm)
        robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.16]).T) # tube
        scl = cgm.Cody_SkinClient(skin_topic)

    epcon = es.Skin_EPC(robot, scl)

    d = ut.load_pickle(opt.pkl)
    q = d['q']
    loc_l = d['loc_l']
    loc_mat = np.column_stack(loc_l)
    x_arr = loc_mat[0,:].A1
    y_arr = loc_mat[1,:].A1
    force_mat = np.column_stack(d['f_l'])

    force_mags = ut.norm(force_mat).A1.tolist()
    print 'force_mags:', force_mags

    ee = robot.kinematics.FK(q)[0]
    goal = ee + d['delta_x_g']

    single_step_qp(d, epcon)

#    pp.figure()
#    pp.axis('equal')
#
#    mpu.plot_quiver_yxv(y_arr, x_arr, force_mat)
#    robot.kinematics.plot_arm(q)
#    pp.plot(ee[0,0], ee[1,0], 'xb', ms=10, mew=2, label='EE')
#    pp.plot(goal[0,0], goal[1,0], 'xg', ms=10, mew=2, label='Goal')
#
#    pp.legend()
#    pp.show()














