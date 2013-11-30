import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy
import hrl_cody_arms.cody_arm_client as cac
import epc_skin_math as esm
import cPickle as pkl
import numpy as np

path = '/home/mkillpack/git/hrl_haptic_manipulation_in_clutter/sandbox_advait_darpa_m3/src/sandbox_advait_darpa_m3/teleop_rviz/'
d = pkl.load(file(path+'delta_qp_opt_dict.pkl'))
Jc_l = d['Jc_l']
loc_l = d['loc_l']
Kc_l = d['Kc_l']             
delta_f_min = d['delta_f_min']
f_l = d['f_l']
delta_f_max = d['delta_f_max']
f_n = d['f_n']
delta_x_g = d['delta_x_g']
Rc_l = d['Rc_l']
n_l = d['n_l']
J_h = d['J_h']
K_j = d['K_j']
q = d['q']
phi_curr = d['phi_curr']

robot = cac.CodyArmClient_7DOF('l')

if True:
    jerk_opt_weight = 0.00001
    max_force_mag = 3.0

    cost_quadratic_matrices, cost_linear_matrices, \
        constraint_matrices, \
        constraint_vectors, lb, ub = esm.convert_to_qp(J_h, Jc_l,
                                                       K_j, Kc_l, Rc_l,
                                                       delta_f_min,
                                                       delta_f_max,
                                                       phi_curr,
                                                       delta_x_g, f_n, q,
                                                       robot.kinematics,
                                                       jerk_opt_weight,
                                                       max_force_mag)

    delta_phi_opt, opt_error, feasible = esm.solve_qp(cost_quadratic_matrices, 
                                                      cost_linear_matrices, 
                                                      constraint_matrices, 
                                                      constraint_vectors, 
                                                      lb, ub, 
                                                      debug_qp=False)
    contact_mat = np.matrix(np.zeros((len(delta_phi_opt), len(delta_phi_opt))))

    for i in xrange(len(Kc_l)):
        contact_mat = contact_mat + Jc_l[i].T*Kc_l[i]*Jc_l[i]

    B = np.linalg.inv(contact_mat + K_j)*K_j



    #print "predicted_delta_change :\n", J_h*(B*delta_phi_opt)
    print "predicted_delta_change :\n", J_h*(delta_phi_opt)*100
    print "delta_phi_opt :\n", delta_phi_opt
    print "phi_curr :\n ", phi_curr
    

if False:
    robot.set_ep(phi_curr)
    # jerk_opt_weight = 0.00001
    # max_force_mag = 3.0

    # cost_quadratic_matrices, cost_linear_matrices, \
    #     constraint_matrices, \
    #     constraint_vectors, lb, ub = esm.convert_to_qp(J_h, Jc_l,
    #                                                    K_j, Kc_l, Rc_l,
    #                                                    delta_f_min,
    #                                                    delta_f_max,
    #                                                    phi_curr,
    #                                                    delta_x_g, f_n, q,
    #                                                    robot.kinematics,
    #                                                    jerk_opt_weight,
    #                                                    max_force_mag)

    # delta_phi_opt, opt_error, feasible = esm.solve_qp(cost_quadratic_matrices, 
    #                                                   cost_linear_matrices, 
    #                                                   constraint_matrices, 
    #                                                   constraint_vectors, 
    #                                                   lb, ub, 
    #                                                   debug_qp=False)
    # contact_mat = np.matrix(np.zeros((len(delta_phi_opt), len(delta_phi_opt))))

    # for i in xrange(len(Kc_l)):
    #     contact_mat = contact_mat + Jc_l[i].T*Kc_l[i]*Jc_l[i]

    # B = np.linalg.inv(contact_mat + K_j)*K_j

    # print "predicted_delta_change :\n", J_h*(B*delta_phi_opt)
    # print "delta_phi_opt :\n", delta_phi_opt
    
    rospy.sleep(5)
    
    x = 0
    while x < 500: 
        q_cur = robot.get_joint_angles()
        pos_cur,_ = robot.kinematics.FK(q)
        x = x+1
        rospy.loginfo("MOVING NOW ...")
        phi = phi_curr + delta_phi_opt 
        robot.set_ep(phi)
        rospy.sleep(0.01)
        q_next = robot.get_joint_angles()
        pos_next,_ = robot.kinematics.FK(q)
        print "predicted change :\n", robot.kinematics.Jacobian(q_cur)*delta_phi_opt
        print "actual change :\n", pos_next-pos_cur

    print "phi_curr is :\n", phi_curr
    print "phi is :\n", phi
