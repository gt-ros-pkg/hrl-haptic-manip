import roslib
roslib.load_manifest('sandbox_marc_darpa_m3')
import hrl_haptic_controllers_darpa_m3.epc_skin as es
import haptic_controllers.epc_generator_marc as egm
import equilibrium_point_control.epc as epc
import numpy as np
import math
import rospy
import hrl_common_code_darpa_m3.visualization.draw_scene as ds
import matplotlib.pyplot as pl
from hrl_haptic_manipulation_in_clutter_msgs.msg import MechanicalImpedanceParams
from sandbox_marc_darpa_m3.srv import * #ServiceBasedMPC
import itertools as it
import time

import threading

class EqPtGen(Exception):
    def __init__(self, stop, control_args, delta_qp_opt_dict=None):
        self.stop = stop
        self.control_args = control_args
        self.delta_qp_opt_dict = delta_qp_opt_dict

class Skin_EPC_marc(es.Skin_EPC):
    def __init__(self, robot, skin_client):
        es.Skin_EPC.__init__(self, robot, skin_client)
        self.draw = ds.SceneDraw('/haptic_controllers/vmc/virtual_force')
        #self.tau = []
        self.var_imped_sub = rospy.Subscriber('/sim_arm/joint_impedance', MechanicalImpedanceParams, self.get_impedance)
        self.lock = threading.RLock()
        #rospy.wait_for_service('get_mpc_fast')
        self.get_delta_phi_opt = rospy.ServiceProxy('get_mpc_fast', ServiceBasedMPC)
        self.advait_prep_time = []
        self.fill_matrices_time = []
        self.service_call_time = []
        self.K = None
        self.K_d = None

    def get_impedance(self, msg):
        self.lock.acquire()
        self.K = np.matrix(np.diag(list(msg.k_p.data)))
        self.K_d = np.matrix(np.diag(list(msg.k_d.data)))
        self.lock.release()

    def fast_delta_mpc_qs(self, ep_gen):
        #time.sleep(2)
        # start_cont = time.time()
        self.controller_time.update()
        
        t_tmp = self.controller_time.average_period(20)

        # t is the estimated time step for each controller cycle
        steps_to_average = 5
        t = self.controller_time.average_period(steps_to_average)
        if t is None:
            # not enough time steps to have an estimate yet, so be
            # very conservative.  0.000001 seconds = 1 x 10^-6 seconds
            # = microsecond -> million times per second. This means
            # that the controller will be operating at a much lower
            # rate, and will not do very much for the first few time
            # steps.
            #t = 0.000001 

            # Advait: setting it to the max rate at which this
            # controller can run.
            t = ep_gen.time_step

        # The system can become unstable and perform poorly if the
        # time period gets large, since this means that it has
        # insufficient time resolution in its control. This limits how
        # much the controller will try to achieve in a single time
        # step.
        #max_t = 0.05 #seconds -> 20 Hz
        max_t = ep_gen.time_step * 2
        if t > max_t:
            t = max_t #seconds


        time_step = ep_gen.time_step
        stopping_dist_to_goal = ep_gen.stopping_dist_to_goal

        x_g = ep_gen.goal_pos
        
        # joint number of the point that we are controlling to go to a
        # goal location.
        # end effector = 7, elbow = 3.
        # x_h and J_h will be computed using this.
        control_point_joint_num = ep_gen.control_point_joint_num

        ############### Controller Specific Parameters #################

        robots_dict = {
          "Cody" : {
                "max_delta_force_mag" : 10., # Newtons/second
                "jerk_opt_weight" : 0.00001, # 0.0 means ignore

                "only_use_normal_force_sensing" : True,
                "use_var_impedance" : False,

                'save_state': False,

                'enable_fragile_regions': False,
                },
          "3 link ODE simulation" : {
                "max_delta_force_mag" : 10.0, # 10.0 Newton/second
                "jerk_opt_weight" : 0.01, # 0.0 means ignore

                "use_var_impedance" : True,
                "only_use_normal_force_sensing" : True,

                'save_state': False,

                'enable_fragile_regions': False,
                },
          "6 link ODE simulation" : {
                "max_delta_force_mag" : 10.0, # 10.0 Newton/second
                "jerk_opt_weight" : 0.00001, # 0.0 means ignore

                "use_var_impedance" : True,
                "only_use_normal_force_sensing" : True,

                'save_state': False,

                'enable_fragile_regions': False,
                }
            }

        # Specify which joint stiffness parameters to use.
        num_joints = len(self.robot.get_joint_angles())
        robot_name = None
        if num_joints == 3:
            # assume that 3 joints means this is simulation
            robot_name = "3 link ODE simulation"
            use_cody_stiff = False
        elif num_joints == 6:
            # assume that 3 joints means this is simulation
            robot_name = "6 link ODE simulation"
            use_cody_stiff = False            
        elif num_joints == 7:
            # assume that 7 joints means this is the real robot Cody
            robot_name = "Cody"            
            use_cody_stiff = True
        else:
            err_msg = "The number of joints for the robot (num_joints = %d) does not correspond with a known robot, (robot_name = %s). Currently, 3 joints is assumed to be the simulated robot, and 7 joints is assumed to be the real robot Cody. Based on this number this controller (delta_qp_jep_gen) decides which parameters to use." % (num_joints, robot_name)
            raise ValueError(err_msg)
        param_dict = robots_dict[robot_name]

        # Specify whether or not to use sensing of normal forces
        # **** WARNING: THIS FEATURE MAY NOT BE IMPLEMENTED ****
        only_use_normal_force_sensing = param_dict["only_use_normal_force_sensing"]

        # Prior to optimization, the goal position for the hand, x_g,
        # is set to be at most dist_g * time_period away from the
        # current location of the hand.
        vel_g = ep_gen.goal_velocity_for_hand
        dist_g = t * vel_g

        # For now, each contact uses the same stiffness matrix, which
        # is defined such that there is a stiffness of k in the
        # direction of the surface normal, and a stiffness of 0 in
        # directions orthogonal to the normal. Consequently, shear
        # forces are currently ignored.
        #k = param_dict["k"]
        k = ep_gen.k
        estimate_contact_stiffness = ep_gen.estimate_contact_stiffness

        # The controller attempts to constrain the magnitude of the
        # force at the contact locations to be less than
        # allowable_contact_force
        max_force_mag = ep_gen.allowable_contact_force

        # The controller attempts to constrain the magnitude of the
        # change in the force at each contact location to be less
        # than max_delta_force_mag Newtons/second.
        max_delta_force_mag = t * param_dict["max_delta_force_mag"]

        # This weights the importance of minimizing the jerk versus
        # achieving the goal position of the hand x_g. 0.0 would mean
        # that this part of the optimization is ignored. 1.0 would
        # mean jerk optimization is as important as x_h optimization
        jerk_opt_weight = param_dict["jerk_opt_weight"]
        
        # The controller restricts the expected change in the position
        # of the hand to be within a polytope of approximately n_faces
        # currently n_faces can be 6, 12, or 20
        #max_delta_x_h = t * param_dict["max_delta_x_h"]
        #n_faces =  param_dict["n_faces"]

        # save the state required to run the optimization at every
        # time step. can use this state to re-run the optimization
        # offline, with different parameters.
        save_state = param_dict['save_state']

        ignore_wrist_joints = ep_gen.ignore_wrist_joints

        planar = ep_gen.planar

        # Specify the precision with which the numpy arrays should be
        # printed.
        np.set_printoptions(precision=4)

        # Suppress the use of scientific notation for small numbers
        # when printing numpy arrays
        np.set_printoptions(suppress=True)


        ###########################################
        ###
        ### BEWARE: directly copied from C++ files
        ###         /darpa_m3/src/software_simulation/demo_kinematic.cpp

        if use_cody_stiff:
            k0 = 20
            k1 = 30
            k2 = 15
            k3 = 25
            k4 = 30
            k5 = 30
            k6 = 30

            if ignore_wrist_joints:
                K_j = np.matrix(np.diag([k0,k1,k2,k3]))
            else:
                K_j = np.matrix(np.diag([k0,k1,k2,k3,k4,k5,k6]))
        else:
            # k1 = 30
            # k2 = 20
            # k3 = 15
            K_j = self.K_j  # np.matrix(np.diag([k1,k2,k3]))

        m = K_j.shape[0]

        ###########################################

        try:
            dt = self.controller_time.prev_period()
            q = self.robot.get_joint_angles()
            # theta is a vector with shape n x 1
            theta = np.matrix(q).T

            # x_h is the current position of the hand
            x_h = self.robot.kinematics.FK(q, control_point_joint_num)[0]
            if (self.prev_x_h is None) or (dt is None):
                x_h_vel = None
            else:
                x_h_vel = (x_h - self.prev_x_h)/dt
            self.prev_x_h = x_h

            stop = ''
            # if close enough to goal declare success!
            dist_goal_2D = np.linalg.norm((x_g - x_h)[0:2])
            dist_goal_3D = np.linalg.norm(x_g - x_h)
            if planar:
                dist_goal = dist_goal_2D
            else:
                dist_goal = dist_goal_3D

            if dist_goal < stopping_dist_to_goal:
                stop = 'Reached'
                raise EqPtGen(stop, ())

            # This acquires lists with the forces, taxel normals,
            # locations of forces, and distal joints beyond contact.
            #
            # All vector quantities are specified with respect to the
            # world frame of reference, which is typically the base
            # frame of the robot.
            #
            # f_l is a list of the contact forces. Each element of f_l
            # is a 3 dimensional force vector that describes the force
            # that the robot is applying to the world at the contact
            # location. So, if the robot is pushing on something, the
            # contact force vector, f_ci, would point away from the
            # robot's arm. Adhesion would be indicated by the a force
            # vector that points towards the robot's arm.
            #
            # n_l is a list of the surface normals at the contact
            # locations. Each element of n_l is a 3 dimensional unit
            # vector that is normal to the surface of the robot at the
            # contact location and points away from the surface of the
            # robot.
            #
            # loc_l is a list of the contact locations. Each element
            # of loc_l is a 3 dimensional vector that specifies the
            # location of the corresponding contact.
            #
            # jt_l is a list that specifies the link upon which each
            # contact is located. Each element of jt_l indicates the
            # number for the contact's corresponding link. This can be
            # used to determine which of the distal joints are
            # uninfluenced by the contact.

            f_l, n_l, loc_l, jt_l, time_stamp = self.scl.force_normal_loc_joint_list(
                normal_component_only = only_use_normal_force_sensing, 
                return_time = True)            

            try:
                if ep_gen.ignore_skin:
                    f_l, n_l, loc_l, jt_l = [], [], [], []
            except AttributeError:
                # Advait added ignore_skin as a way to get a baseline
                # controller that behaves just like the QP controller
                # in free space in terms of joint limits, min jerk
                # term etc. etc.
                pass

            # n = the number of contacts
            n = len(n_l)

            # f_mag_list is a list of the magnitudes of the contact
            # forces
            f_mag_list  = [np.linalg.norm(f_vec) for f_vec in f_l]

            # stop if a contact force is too high
            # the 100N threshold is a remnant from old simulation trials, not sure if safe to remove
            #####the elif statement is a safety catch that allows user to specify######## 
            #####stopping force while not using the monitoring and logging nodes#########

            try:
                if f_mag_list != [] and max(f_mag_list) > 100:
                    stop = 'high force %f'%(max(f_mag_list))
                    raise EqPtGen(stop, ())
                elif f_mag_list != [] and ep_gen.kill_controller_force > 0 and max(f_mag_list) > ep_gen.kill_controller_force:
                    stop = 'high force %f'%(max(f_mag_list))
                    raise EqPtGen(stop, ())
            except AttributeError:
                # probably because of ep_gen.kill_controller_force not
                # existing. This is a temporary thing.
                pass

            J_all = self.robot.kinematics.Jacobian(q, x_h)
            # J_h = Jacobian for the hand (end effector)
            J_h = J_all[0:3]
            J_h[:, control_point_joint_num:] = 0.

            J_h = J_h[:,0:m] # comes into play when ignore_wrist_joints=True

            jep = np.array(self.robot.get_ep())
            # phi_curr = phi[t] = current equilibrium angles for 
            # the virtual springs on the robot's arm
            phi_curr = (np.matrix(jep).T)[0:m]

            # compute contact Jacobians
            Jc_l = []
            for jt_li, loc_li in it.izip(jt_l, loc_l):
                Jc = self.robot.kinematics.Jacobian(q, loc_li)
                Jc[:, jt_li+1:] = 0.0
                Jc = Jc[0:3, 0:m]
                Jc_l.append(Jc)

			#this is currently not doing anything - marc
            # if use_var_impedance == True:
            #     pass

            # HACK by Advait on Oct 12, 2011.
            # dunno if I want to start with low estimate of stiffness
            # and then have a cap on the max allowed stiffness, or
            # start with a high estimate of the stiffness and keep
            # that as the max allowed estimate of the stiffness.
            if use_cody_stiff:
                #k_est_max = 500.
                k_est_max = 1000.
            else:
                k_est_max = max(k, 1000.)

            Kc_l = self.contact_stiffness_matrix(n_l, loc_l, time_stamp, 
                                                 k,
                                                 estimate_contact_stiffness,
                                                 k_est_max = k_est_max)
            
            Rc_l = self.contact_force_transformation_matrix(n_l)
                
            # calculate the normal components of the current contact
            # forces
            tmp_l = [R_ci * f_ci for R_ci, f_ci in it.izip(Rc_l, f_l)]
            f_n = np.matrix([tmp_i[0,0] for tmp_i in tmp_l]).T

           
            delta_f_min, delta_f_max = self.delta_f_bounds(f_n, 
                                                           max_pushing_force = max_force_mag,
                                                           max_pulling_force = max_force_mag,
                                                           max_pushing_force_increase = max_delta_force_mag, 
                                                           max_pushing_force_decrease = max_delta_force_mag, 
                # large min_decrease_when_over_max_force was causing
                # the constraints to become infeasible in lots of
                # cases. (Advait, Sept 15, 2011)
                                                           #min_decrease_when_over_max_force = 0.5,
                                                           min_decrease_when_over_max_force = 0.01,
                                                           max_decrease_when_over_max_force = 10.0
                                                           )

            delta_x_g  = self.goal_motion_for_hand_advait(x_h, x_g, dist_g)

            # if save_state:
            #     delta_qp_opt_dict = {}
            #     delta_qp_opt_dict['J_h'] = J_h
            #     delta_qp_opt_dict['Jc_l'] = Jc_l
            #     delta_qp_opt_dict['Kc_l'] = Kc_l
            #     delta_qp_opt_dict['Rc_l'] = Rc_l
            #     delta_qp_opt_dict['delta_f_min'] = delta_f_min
            #     delta_qp_opt_dict['delta_f_max'] = delta_f_max
            #     delta_qp_opt_dict['phi_curr'] = phi_curr
            #     delta_qp_opt_dict['delta_x_g'] = delta_x_g
            #     delta_qp_opt_dict['K_j'] = K_j
            #     delta_qp_opt_dict['loc_l'] = loc_l
            #     delta_qp_opt_dict['n_l'] = n_l
            #     delta_qp_opt_dict['f_l'] = f_l
            #     delta_qp_opt_dict['f_n'] = f_n # normal component of the force.
            #     delta_qp_opt_dict['q'] = q
                #ut.save_pickle(delta_qp_opt_dict, 'delta_qp_opt_dict.pkl')

            
           
            # delta_phi_opt, opt_error, feasible = esm.solve_qp(cost_quadratic_matrices, 
            #                                                   cost_linear_matrices, 
            #                                                   constraint_matrices, 
            #                                                   constraint_vectors, 
            #                                                   lb, ub, 
            #                                                   debug_qp=False)

            # if save_state:
            #     delta_qp_opt_dict['delta_phi_opt'] = delta_phi_opt

            # self.advait_prep_time.append(time.time()-start_cont)
            # fill_mat_time = time.time()

            #assert(False)


            req = ServiceBasedMPCRequest()
            # these gains for the multi-objective cost function were hand tuned
            # but seem to give reasonable results
            req.alpha = 100.0
            req.beta = 0.001
            req.gamma = 0.1
            
            # change in goal position for end effector
            req.delta_x_d = (delta_x_g[0:2]).A1.tolist()
            
            #jacobian of end effector
            req.J = J_h[0:2,:].flatten('F').A1.tolist()
            
            #initial config of joint angles
            req.x_0 = list(q)

            # weighting matrix to limit the change in torque at each time step
            req.KP_t_KP = (K_j.T*K_j).flatten('F').A1.tolist()

            #limits on predicted joint angles (state)
            req.q_min = np.radians(np.array([-150., -63, 0.])).tolist()
            req.q_max = np.radians(np.array([150., 162, 159.])).tolist()

            # limits on change in equilibrium angles (these numbers are probably too big)
            req.u_min = [-0.54]*3
            req.u_max = [0.54]*3

            # calculation of the input weighting matrix for B*q_eq
            sum_matrix = np.matrix(np.zeros((3,3)))
            for ii in xrange(len(Jc_l)):
                sum_matrix = sum_matrix+Jc_l[ii].T*Kc_l[ii]*Jc_l[ii]
            req.B = (np.linalg.inv(K_j+sum_matrix)*K_j).flatten('F').A1.tolist() #A1.tolist()

            num_poss_contacts = 150
            
            # filling vectors for allowable change in force at each contact
            if np.where(f_n.A1 < max_force_mag)[0].tolist() == []:
                delta_f_min = np.matrix(np.zeros((num_poss_contacts, 1)))
                delta_f_max = np.matrix(np.zeros((num_poss_contacts, 1)))
            else:
                delta_f_min = delta_f_min[np.where(f_n.A1 < max_force_mag)[0].tolist()]
                delta_f_min = np.vstack((delta_f_min, np.zeros((num_poss_contacts-delta_f_min.shape[0], 1))))
                delta_f_max = delta_f_max[np.where(f_n.A1 < max_force_mag)[0].tolist()]
                delta_f_max = np.vstack((delta_f_max, np.zeros((num_poss_contacts-delta_f_max.shape[0], 1))))
            req.f_min = delta_f_min.A1.tolist()
            req.f_max = delta_f_max.A1.tolist()

            # filling the matrices for estimating change in force from predicted change in joint angles
            # as well as the desired decrease in contact force when forces are above threshold
            n_K_ci_J_ci_buff = []
            n_K_ci_J_ci_max_buff = []
            des_force_decrease = []

            for ii in xrange(len(Jc_l)):
                if f_n[ii, 0] <= max_force_mag:
                    buff_list = (n_l[ii].T*Kc_l[ii]*Jc_l[ii]).A1.tolist()
                    n_K_ci_J_ci_buff.append(buff_list)
                elif f_n[ii,0] > max_force_mag:
                    des_force_decrease.append(max_force_mag - f_n[ii,0])
                    buff_list = (n_l[ii].T*Kc_l[ii]*Jc_l[ii]).A1.tolist()
                    n_K_ci_J_ci_max_buff.append(buff_list)

            # this is a check for any contact at all before populating matrices
            if n_K_ci_J_ci_buff == []:
                n_K_ci_J_ci = np.zeros((num_poss_contacts, 3))
            else:
                n_K_ci_J_ci = np.array(n_K_ci_J_ci_buff)
                n_K_ci_J_ci = np.vstack((n_K_ci_J_ci, np.zeros((num_poss_contacts-n_K_ci_J_ci.shape[0], 3))))
            if n_K_ci_J_ci_max_buff == []:
                n_K_ci_J_ci_max = np.zeros((num_poss_contacts, 3))                
            else:
                n_K_ci_J_ci_max = np.array(n_K_ci_J_ci_max_buff)
                n_K_ci_J_ci_max = np.vstack((n_K_ci_J_ci_max, np.zeros((num_poss_contacts-n_K_ci_J_ci_max.shape[0], 3))))

            req.n_K_ci_J_ci = n_K_ci_J_ci.flatten('F').tolist()
            req.n_K_ci_J_ci_max = n_K_ci_J_ci_max.flatten('F').tolist()

            if des_force_decrease != []:
                np_des_force_decrease = np.array(des_force_decrease).reshape(len(des_force_decrease),1)
                req.desired_force_decrease = (np.vstack((np_des_force_decrease, np.zeros((num_poss_contacts - np_des_force_decrease.shape[0], 1))))).flatten().tolist()
            else:
                req.desired_force_decrease = (np.zeros(num_poss_contacts)).tolist()                

            try:
                #this is the service call to the mpc optimization code (c++)
                res = self.get_delta_phi_opt(req)
                jep[0:m] = (phi_curr + np.matrix(res.delta_phi_opt).reshape(3,1)).A1

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                #if the service call failed, don't change the q_eq
                jep[0:m] = (phi_curr).A1

            # warn if JEP goes beyond joint limits
            if self.robot.kinematics.within_joint_limits(jep) == False:
                rospy.logwarn('Outside joint limits. They will be clamped later...')

            print "jep is :\n", jep.tolist()
            raise EqPtGen(stop, (jep.tolist(), time_step*1.5))#, delta_qp_opt_dict)

        except EqPtGen as e:
            return e.stop, e.control_args


    def teleop_greedy_to_goal(self, goal_pos, control_param_dict, 
                              eq_gen_type, logging_name, monitor_param_dict):

        controller_name = 'teleop_'+eq_gen_type

        control_param_dict['goal_pos'] = goal_pos
        time_step = control_param_dict['time_step']

        if eq_gen_type == 'optimize_qp_jep':
            eq_gen_func = self.delta_qp_jep_gen
        elif eq_gen_type == 'fast_delta_mpc_qs':
            eq_gen_func = self.fast_delta_mpc_qs
        elif eq_gen_type == 'torque_vmc':
            eq_gen_func = self.torque_vmc_set_pt_gen
        elif eq_gen_type == 'opt_vmc':
            eq_gen_func = self.torque_vmc_set_pt_gen
        elif eq_gen_type == 'simple':
            eq_gen_func = self.greedy_eq_gen

        # ep_gen = egm.EP_Generator_Marc(eq_gen_func, self.robot.set_ep,
        #                           self.robot.kinematics.clamp_to_joint_limits)

        ep_gen = egm.EP_Generator_Marc(eq_gen_func, self.robot.set_ep)



        ep_gen.__dict__.update(control_param_dict)

        ep_gen.reaching_in = True
        ep_gen.control_point_joint_num = self.robot.kinematics.n_jts

        res = self.epc_motion_continuous(ep_gen, time_step)

        return res

        

    def greedy_to_goal_marc(self, goal_pos,
                       threshold_dict, timeout,
                       eq_gen_type,
                       fragile_center = None, fragile_radius = None):
        threshold_dict['goal_pos'] = goal_pos
        threshold_dict['force_exceed_counter'] = 0
        time_step = threshold_dict['time_step']

        ##############################################################
        # this is what Advait uses to log and stop when high forces
        # or not making progress, should have something equivalent or
        # use his (see here and announce_controller_stop too)
        ##############################################################
        # self.announce_controller_start(controller_name, goal_pos,
        #                                logging_name,
        #                                monitor_param_dict)
        ##############################################################

        if eq_gen_type == 'optimize_qp_jep':
            eq_gen_func = self.delta_qp_jep_gen
        elif eq_gen_type == 'fast_delta_mpc_qs':
            eq_gen_func = self.fast_delta_mpc_qs
        elif eq_gen_type == 'torque_vmc':
            eq_gen_func = self.torque_vmc_set_pt_gen
        elif eq_gen_type == 'simple':
            eq_gen_func = self.greedy_eq_gen

        ep_gen = egm.EP_Generator_Marc(eq_gen_func, self.robot.set_ep)


        # ep_gen = egm.EP_Generator_Marc(eq_gen_func, self.robot.set_ep,
        #                           self.robot.kinematics.clamp_to_joint_limits)

        ep_gen.__dict__.update(threshold_dict)

        ep_gen.reaching_in = True
        ep_gen.control_point_joint_num = self.robot.kinematics.n_jts

        res = self.epc_motion(ep_gen, time_step, timeout = timeout)

        ##############################################################
        # self.announce_controller_stop(controller_name, logging_name,
        #                               control_param_dict)
        ##############################################################

        return res


    def greedy_pull_in_and_out(self, goal_pos, eq_gen_type, q_home, gtg_dict):
        testbed = 'software_simulation'
        condition = True

        offset = 0
        goals = [[0.1, goal_pos[1,0], 0.0]]
        while offset+goal_pos[1,0] >= -0.55:
            offset = offset-0.15
            goals.append([0.1, goal_pos[1,0]+offset, 0.0])
        print "here are the intermed goals :", goals

        distances = []
        for i in xrange(len(goals)):
            distances.append(np.linalg.norm(goal_pos-np.matrix(goals[i]).T))
        distances = np.array(distances)
        sorted_distances = np.copy(distances)
        sorted_distances.sort()
        res = self.greedy_to_goal_marc(goal_pos, gtg_dict,
                                      40., eq_gen_type)

        print "Goal Result is :", res
        if res[0] == 'Reached':
            return res

        for i in xrange(len(goals)):
            res_interm = self.greedy_to_goal_marc(np.matrix(goals[i]).T, gtg_dict,
                                           20., eq_gen_type)
            print "Way Point result is :", res_interm

            res = self.greedy_to_goal_marc(goal_pos, gtg_dict,
                                           30., eq_gen_type)
            print "Goal Result is :", res
            if res[0] == 'Reached':
                return res

            res_interm = self.greedy_to_goal_marc(np.matrix(goals[i]).T, gtg_dict,
                                           20., eq_gen_type)
            print "Way Point result is :", res_interm
            if res_interm[0] == 'Reached':
                print "Going to initial configuration..."
                self.go_jep(q_home.A1.tolist(), speed=math.radians(60))

        return res


    ##
    # @param ep_gen - object of EP_Generator. can include any state that you want.
    # @param time_step: time between successive calls to equi_pt_generator
    # @return stop (the string which has the reason why the epc
    # motion stopped.), ea (last commanded equilibrium angles)
    def epc_motion_continuous(self, ep_gen, time_step):
        ep_gen_func = ep_gen.ep_gen_func
        control_function = ep_gen.control_function
        ep_clamp_func = ep_gen.ep_clamp_func

        rt = rospy.Rate(1/time_step)
        stop = ''
        ea = None
        while stop == '':
            if rospy.is_shutdown():
                stop = 'rospy shutdown'
                continue

            if self.stop_epc:
                stop = 'stop_command_over_ROS'
                continue
            
            if self.pause_epc:
                rospy.sleep(0.1)
                continue

            if stop == '':
                stop, ea = ep_gen_func(ep_gen)
            if stop == 'reset timing':
                stop = ''
                t_end = rospy.get_time()

            if stop == '':
                if ep_clamp_func != None:
                    ep = ea[0]
                    ea = list(ea)
                    ea[0] = ep_clamp_func(ep)
                    ea = tuple(ea)

                control_function(*ea)

            if stop == 'Reached':
                stop = ''

            rt.sleep()
            
        return stop, ea

    

    def gain_func(self, force, f_thresh=5, max_lim=20):
        if force > f_thresh:
            return (1/(1+math.exp(-force)) - 0.5)*(max_lim)
        else:
            return 0.0
    def torque_vmc_set_pt_gen(self, ep_gen):
        """
        Controller uses virtual model control-like forces applied
        at the end-effector.  These forces are determined in direction
        and magnitude by the a PID loop on the error between the end-effector
        and the current goal.

        Controller tries to regulate forces by subtracting off a portion or 
        a function of the applied contact forces at each taxel location.

        This controller comes from ideas I shared with Charlie, but is slightly 
        different than his initial interpretation of those ideas.
        
        Initial work: 10 May 2011
        """

        # threshold_dict = eq_gen.threshold_dict
        time_step = ep_gen.time_step
        max_jep_mag = ep_gen.max_jep_mag
        stopping_dist_to_goal = ep_gen.stopping_dist_to_goal
        f_thresh = ep_gen.allowable_contact_force
        kill_force = ep_gen.kill_controller_force
             #5.0  # this should be in dictionary
       
        # active_joints = threshold_dict['active_joints']
        goal_pos = ep_gen.goal_pos
        
        #if threshold_dict.has_key('err_prev'):
        #     err_prev = threshold_dict['err_prev']
        # else:
        #     err_prev = None

        # if threshold_dict.has_key('err_sum'):
        #     err_sum = threshold_dict['err_sum']
        # else:
        #     err_sum = None

        try: 
            err_prev = ep_gen.err_prev
        except:
            err_prev = None
        try:
            err_sum = ep_gen.err_sum
        except:
            err_sum = None

        ######## TO DO ########
        ## + Generalize to 3D goal location (e.g., remove [0:2])
        ## + Add windup protection for integral term
        ## + Try changing the stiffness for "pulling string" method
        ## + Filter and include damping term on joint velocities
        ## + add other damping if necessary
        ## + avoid jt limits or locked joints using other virtual forces

        #################################################
        ######## Controller Specific Parameters #########

        use_cody_stiff = False #if true use stiffness settings for Cody
        only_use_normal_force_sensing = True

        # compute magnitude of virtual force using PID
        k_p = 15.0 
        k_d = 0.8 #1.0 
        k_i = 0.08 #1.0 #0.05 


        while self.K == None:
            rospy.sleep(0.001)
        # if use_cody_stiff:
        #     k0 = 20
        #     k1 = 30
        #     k2 = 15
        #     k3 = 25
        #     k4 = 10
        #     k5 = 10
        #     k6 = 10
        #     K = np.matrix(np.diag([k0,k1,k2,k3,k4,k5,k6]))
        # else:
        #     k1 = 30
        #     k2 = 20
        #     k3 = 15
        #     K = np.matrix(np.diag([k1,k2,k3]))
        #     kd1 = 15
        #     kd2 = 10
        #     kd3 = 8
        #     K_d = np.matrix(np.diag([kd1, kd2, kd3]))

        K_inv = np.linalg.pinv(self.K)
        ###########################################


        try:
            q = self.robot.get_joint_angles()

            # probably can't use these until add a filter in self.robot###
            # q_dot = self.robot.get_joint_angle_rates(arm)
            #############################################################

            p, r = self.robot.kinematics.FK(q)
            J_all = self.robot.kinematics.Jacobian(q, p)

            # J = Jacobian for the end effector
            #J = J_all[0:3, active_jt_idxs]
            J = J_all[0:3]  #active jt idxs no longer exists how do I do this now?? 
            JT_pinv = np.linalg.pinv(J.T) 

            # returns forces, taxel normals, locations of forces, 
            # and distal joints beyond contact 
            f_l, n_l, loc_l, jt_l = self.scl.force_normal_loc_joint_list(normal_component_only = 
                                                                         only_use_normal_force_sensing) 
            f_mag_list  = [np.linalg.norm(f_vec) for f_vec in f_l] 


            ##########################################
            # compute virtual force based on the error
            err = (goal_pos - p)
            err_mag = np.linalg.norm(err[0:2,:])
            err_unit = err/err_mag 

########################################################################################
########################################################################################
##############NEED TO FIX THE DERIVATIVE TERM, this is noisy right now
########################################################################################
            if err_prev is None:
                # assume initial condition is zero error to smooth the start
                err_diff = err
            else:
                err_diff = err - err_prev
            err_prev = err.copy()
            err_diff_mag = np.linalg.norm(err_diff[0:2, :])
            ep_gen.err_prev = err_prev
########################################################################################
########################################################################################
########################################################################################


            if err_sum is None:
                err_sum = err.copy()
            elif err.T*err_sum < 0:
                err_sum = np.zeros((3,1))
            elif f_mag_list != [] and max(f_mag_list) > 5.:
                err_sum = ep_gen.err_sum
            else:
                err_sum = err_sum + err 
            err_sum_mag = np.linalg.norm(err_sum[0:2, :]) 
            ep_gen.err_sum = err_sum 

            x_dot = J*np.matrix(self.robot.qdot).reshape(self.robot.kinematics.n_jts,1) 

            x_dot_err = (0.05*err_unit - x_dot) 
            x_dot_err_mag = np.linalg.norm(x_dot_err[0:2,:]) 
            x_dot_err_unit = x_dot_err/x_dot_err_mag 


            #(k_d * err_diff_mag) + 
            F_virtual = err_unit * ((k_p * err_mag) + 
                                    (k_d * np.linalg.norm(x_dot)) + 
                                    (k_i * err_sum_mag)) 

            #######I should add a spring and damper near joint limits!!!

        # self.min_jtlim_arr = np.radians(np.array([-150., -63, 0.]))
        # self.max_jtlim_arr = np.radians(np.array([150., 162, 159.]))
            tau_limits = np.matrix(np.zeros((self.robot.kinematics.n_jts,1)))
            # for i in xrange(len(q)):
            #     if q[i] < (self.robot.kinematics.min_jtlim_arr[i] + np.radians(2)):
            #         tau_limits[i,0] = 1*1/(q[i]-self.robot.kinematics.min_jtlim_arr[i]) - self.robot.qdot[i]*8.0
            #     elif  q[i] > (self.robot.kinematics.max_jtlim_arr[i] - np.radians(2)):
            #         tau_limits[i,0] = 1*1/(q[i]-self.robot.kinematics.max_jtlim_arr[i]) - self.robot.qdot[i]*8.0
                
            #F_damping = -5*x_dot

            ##############################################would be better if this slewed from current Force instead of flipping so fast (i.e. min jerk)#####
            if np.linalg.norm(x_dot) > 0.04:
                F_virtual = x_dot/np.linalg.norm(x_dot) * -(1/(1+math.exp(-np.linalg.norm(x_dot)*10.0)) - 0.5)*(np.linalg.norm(F_virtual)*4.0)
                #F_virtual = F_virtual + 100*(0.04 - x_dot)

            ################################################################################################################################################

            # if np.linalg.norm(x_dot) > 0.05:
            #     F_virtual_vel_damping = x_dot_err_unit*(1000.0*x_dot_err_mag)
            # else:
            #     F_virtual_vel_damping = np.matrix(np.zeros((3,1)))

            #print "velocity force term :", F_virtual_vel_damping

            #F_virtual = F_virtual + F_virtual_vel_damping

            #visualize the virtual force now
            F_virtual_unit = F_virtual/np.linalg.norm(F_virtual)

            #quat = self.draw.quat_from_axis_angle(F_virtual_unit)
            #self.draw.pub_body(p,quat,[np.linalg.norm(F_virtual)*10, 0.1, 0.1], [1, 0, 1, 1], 1, self.draw.Marker.ARROW)

            #self.draw.pub_arrow(p, p+F_virtual*0.01, [1, 0, 1, 1], str(np.linalg.norm(F_virtual)))
            #self.draw.pub_arrow(p, p+F_damping*0.01, [1, 0, 0, 1], str(np.linalg.norm(F_damping)))



            # find new joint torques corresponding to virtual force being applied to the hand
            tau_virtual_f = J.T * (F_virtual) + tau_limits # + F_damping)
            # find current joint torques using virtual spring model
            jep = np.array(self.robot.get_ep())
            q = np.array(q)
            diff = np.matrix(jep - q).T
            tau_curr = self.K * diff  #Where is the DERIVATIVE term, this makes a big difference!!!
            # use sensed arm contact when deciding how to move
            # prepare sensed contact data from arm
            Jc_l = []
            nc_l = []

            contacts_per_link = np.zeros(len(tau_curr))

            for i in range(len(f_mag_list)): 
                f_mag = f_mag_list[i] 
                #if f_mag > moveable_force: 
                nc_l.append(f_l[i]/f_mag) 

                # calculate number of contacts that each joint will influence 
                contacts_per_link[:(jt_l[i]+1)] = contacts_per_link[:(jt_l[i]+1)] + 1 

                Jc = self.robot.kinematics.Jacobian(q, loc_l[i]) 

                Jc[:, jt_l[i]+1:] = 0. 
                Jc = Jc[0:3]# , active_jt_idxs] 
                Jc_l.append(Jc) 

            tau_c = np.matrix(np.zeros(tau_virtual_f.shape)) 

            # calculate joint torques associated with each contact
            # force assuming static equilibrium
            for i in range(len(nc_l)): 
                t = Jc_l[i].T * (f_l[i]*self.gain_func(np.linalg.norm(f_l[i]),f_thresh))
                tau_c = tau_c + t 
                    
                    
            #tau_cmd = np.matrix(np.zeros(tau_virtual_f.shape)) 

            tau_cmd = (tau_virtual_f - tau_c)
            
            #self.tau.append(tau_cmd)

            # q_eq = K_inv*(tau_cmd + K_d*q_dot + K*q)
            q_eq = K_inv*(tau_cmd + self.K*np.matrix(q).reshape(self.robot.kinematics.n_jts,1) 
                          + self.K_d*np.matrix(self.robot.qdot).reshape(self.robot.kinematics.n_jts,1))

            # normalize the length of the d_jep vector to avoid bad stuff?
            # d_tau_mag_desired = 0.75 #1.0 #2.0 #set magnitude of d_tau during search
            # d_tau_mag = np.linalg.norm(d_tau)
            # d_tau = (d_tau / d_tau_mag) * d_tau_mag_desired

            # find change to JEP that will result in desired torque at the joints
            # dJEP = inv(K)(tau - tau_current)
            d_jep = (q_eq-np.matrix(q).reshape(self.robot.kinematics.n_jts,1))*0.01 #SHOULD DO A FIXED STEP IN THIS DIRECTION???
            jep += d_jep.A1



            ######## STOPPING CONDITIONS FOLLOW THIS LINE ########


            if f_mag_list != [] and max(f_mag_list) > kill_force:
                stop = 'high force %f'%(max(f_mag_list))
                raise es.EqPtGen(stop, ())

            stop = ''
            # if close enough to goal declare success!
            if err_mag < stopping_dist_to_goal:
                stop = 'Reached'

            # warn if JEP goes beyond joint limits
            # if self.robot.kinematics.within_joint_limits(jep) == False:
            #     rospy.logwarn('Outside joint limits. Clamping...')

            raise es.EqPtGen(stop, (jep.tolist(), time_step*1.5))

        except es.EqPtGen as e:
            # return string with stopping condition and tuple with new JEP and timestep
            return e.stop, e.control_args

