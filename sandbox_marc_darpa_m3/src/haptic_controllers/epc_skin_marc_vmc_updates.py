import roslib
roslib.load_manifest('sandbox_marc_darpa_m3')
import hrl_haptic_controllers_darpa_m3.epc_skin as es
import sandbox_marc_darpa_m3.haptic_controllers.epc_generator_marc as egm
import equilibrium_point_control.epc as epc
import numpy as np
import math
import rospy
import hrl_common_code_darpa_m3.visualization.draw_scene as ds


class Skin_EPC_marc(es.Skin_EPC):
    def __init__(self, robot, skin_client):
        es.Skin_EPC.__init__(self, robot, skin_client)
        self.draw = ds.SceneDraw()

    def teleop_greedy_to_goal(self, goal_pos, control_param_dict, 
                              eq_gen_type, logging_name, monitor_param_dict):

        controller_name = 'teleop_'+eq_gen_type
        self.announce_controller_start(controller_name, goal_pos,
                                       logging_name,
                                       monitor_param_dict)

        control_param_dict['goal_pos'] = goal_pos
        time_step = control_param_dict['time_step']

        if eq_gen_type == 'optimize_qp_jep':
            eq_gen_func = self.delta_qp_jep_gen
        elif eq_gen_type == 'torque_vmc':
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

        self.announce_controller_stop(controller_name, logging_name,
                                      control_param_dict)

        return res

        

    def greedy_to_goal_marc(self, goal_pos,
                       threshold_dict, timeout,
                       eq_gen_type,
                       fragile_center = None, fragile_radius = None):
        threshold_dict['goal_pos'] = goal_pos
        threshold_dict['force_exceed_counter'] = 0
        time_step = threshold_dict['time_step']

        if eq_gen_type == 'optimize_qp_jep':
            eq_gen_func = self.delta_qp_jep_gen
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

    

    def gain_func(self, force, max_lim=20):
        if force > 5.0:
            return (1/(1+math.exp(-force)) - 0.5)*(max_lim)
        else:
            return 0.0

    def gain_func_linear(self, force, max_lim=20., f_thresh = 4.0):
        if force > f_thresh:
            return (force-f_thresh)*0.1
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
        Major Revisions : 3-7 November 2011
        """

        # threshold_dict = eq_gen.threshold_dict
        time_step = ep_gen.time_step
        max_jep_mag = ep_gen.max_jep_mag
        stopping_dist_to_goal = ep_gen.stopping_dist_to_goal
        f_thresh = ep_gen.allowable_contact_force  # this should be in dictionary
       
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
        k_p = 30.0 
        k_d = 3.0 #1.0 
        k_i = 0.3 #0.08         #1.0 #0.05 

        #k_p = 15.0 #15
        #k_d = 10.0 #1.0 
        #k_i = 1.0 #0.08 #1.0 #0.05 

        if use_cody_stiff:
            k0 = 20
            k1 = 30
            k2 = 15
            k3 = 25
            k4 = 10
            k5 = 10
            k6 = 10
            K = np.matrix(np.diag([k0,k1,k2,k3,k4,k5,k6]))
        else:
            k1 = 30
            k2 = 20
            k3 = 15
            K = np.matrix(np.diag([k1,k2,k3]))
            kd1 = 15
            kd2 = 10
            kd3 = 8
            K_d = np.matrix(np.diag([kd1, kd2, kd3]))

        K_inv = np.linalg.pinv(K)
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

            x_dot = J*np.matrix(self.robot.qdot).reshape(3,1)

            if err_sum is None:
                err_sum = err.copy()
            elif err.T*err_sum < 0:# or np.linalg.norm(x_dot) > ep_gen.goal_velocity_for_hand:
                err_sum = np.zeros((3,1))
            elif f_mag_list != [] and max(f_mag_list) > 5.:
                err_sum = ep_gen.err_sum
            else:
                err_sum = err_sum + err 
            err_sum_mag = np.linalg.norm(err_sum[0:2, :]) 
            ep_gen.err_sum = err_sum 


            print "speed is :", np.linalg.norm(x_dot)
            # x_dot_err = (0.05*err_unit - x_dot) 
            # x_dot_err_mag = np.linalg.norm(x_dot_err[0:2,:]) 
            # x_dot_err_unit = x_dot_err/x_dot_err_mag 

            if f_mag_list != []:
                F_virtual = err_unit * ((k_p * err_mag) + 
                                        (k_i * err_sum_mag)) 
                                        #(k_d * err_diff_mag) + 
                F_damping = - x_dot*k_d
                F_virtual = F_virtual + F_damping
            else:
                ep_gen.err_sum = np.zeros((3,1))
                F_virtual = 5000*(0.04*err_unit - x_dot) #+ err_unit* k_p*err_mag
                #F_virtual = -5000*(0.04 - np.linalg.norm(x_dot))*x_dot/np.linalg.norm(x_dot)# + err_unit* k_p*err_mag





            #######I should add a spring and damper near joint limits!!!
            tau_limits = np.matrix(np.zeros((self.robot.kinematics.n_jts,1)))
            d_safety = np.radians(2)
            for i in xrange(len(q)):
                if q[i] < (self.robot.kinematics.min_jtlim_arr[i] + d_safety):
                    tau_limits[i,0] = 10 * (q[i]-(self.robot.kinematics.min_jtlim_arr[i] - d_safety)) #damping too ? -  self.robot.qdot[i]*2.0
                elif  q[i] > (self.robot.kinematics.max_jtlim_arr[i] - d_safety):
                    tau_limits[i,0] = 10 * (q[i]-(self.robot.kinematics.max_jtlim_arr[i] + d_safety)) # damping too? - self.robot.qdot[i]*2.0

                # if q[i] < (self.robot.kinematics.min_jtlim_arr[i] + np.radians(3)):
                #     tau_limits[i,0] = 1*1/(q[i]-self.robot.kinematics.min_jtlim_arr[i]) - self.robot.qdot[i]*2.0
                # elif  q[i] > (self.robot.kinematics.max_jtlim_arr[i] - np.radians(2)):
                #     tau_limits[i,0] = 1*1/(q[i]-self.robot.kinematics.max_jtlim_arr[i]) - self.robot.qdot[i]*2.0


                
            ##############################################would be better if this slewed from current Force instead of flipping so fast (i.e. min jerk)#####
            if np.linalg.norm(x_dot) > ep_gen.goal_velocity_for_hand:
                #F_virtual = x_dot/np.linalg.norm(x_dot) * (ep_gen.goal_velocity_for_hand - np.linalg.norm(x_dot))*40.0
                #F_virtual = F_virtual * 0.0 
                F_virtual = x_dot/np.linalg.norm(x_dot) * -(1/(1+math.exp(-np.linalg.norm(x_dot)*10.0)) - 0.5)*(np.linalg.norm(F_virtual))
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

#            quat = self.draw.quat_from_axis_angle(F_virtual_unit)
#            self.draw.pub_body(p,quat,[np.linalg.norm(F_virtual), 0.1, 0.1], [1, 0, 1, 1], 9999, self.draw.Marker.ARROW)

            self.draw.pub_arrow(p, p+F_virtual*0.01, [1, 0, 1, 1], str(np.linalg.norm(F_virtual)))
            #self.draw.pub_arrow(p, p+F_damping*0.01, [1, 0, 0, 1], str(np.linalg.norm(F_damping)))



            # find new joint torques corresponding to virtual force being applied to the hand
            tau_virtual_f = J.T * (F_virtual) + tau_limits # + F_damping)
            # find current joint torques using virtual spring model
            jep = np.array(self.robot.get_ep())
            q = np.array(q)
            diff = np.matrix(jep - q).T
            tau_curr = K * diff  #Where is the DERIVATIVE term, this makes a big difference!!!
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
                t = Jc_l[i].T * (f_l[i]*self.gain_func(np.linalg.norm(f_l[i]))) 
                tau_c = tau_c + t 


            ###############################virtual force to reconfigure joints##############
            jt_pos, jt_orien = self.robot.kinematics.FK_vanilla(q)
            Jac_jt = self.robot.kinematics.Jacobian(q,jt_pos)#[0:3]                    

            print jt_orien
            
            F_jt = np.array([0, 0, 0, 0, 0, 0]).reshape(6,1)

            print "err_unit :", err_unit

            cur_angle = math.atan2(jt_orien[1,0], jt_orien[1,1])- math.pi/2.0
            des_angle = math.atan2(err_unit[1], err_unit[0])

            F_jt[5,0] = 5*(des_angle-cur_angle)
            print "cur angle :", cur_angle
            print "des angle :", des_angle



            t_jt = Jac_jt.T*F_jt                    

            jt_pos, _ = self.robot.kinematics.FK(q,1)
            Jac_jt = self.robot.kinematics.Jacobian(q,jt_pos)#[0:3]                    
            Jac_jt[:, 1:] = 0. 
            F_jt = np.array([-10, 0, 0, 0, 0, 0]).reshape(6,1)

            t_jt2 = Jac_jt.T*F_jt                    

            t_jt = t_jt+t_jt2
            # t_jt = np.matrix(np.zeros(tau_virtual_f.shape))                 
            # t_jt[0,0] = -5.0
            #t_jt = np.matrix(np.zeros(tau_virtual_f.shape))                 
            print "jt_torque :", t_jt

            #################################################################################


            #tau_cmd = np.matrix(np.zeros(tau_virtual_f.shape)) 

            tau_cmd = (tau_virtual_f - tau_c) + t_jt
            

            # q_eq = K_inv*(tau_cmd + K_d*q_dot + K*q)
            q_eq = K_inv*(tau_cmd + K*np.matrix(q).reshape(3,1) + K_d*np.matrix(self.robot.qdot).reshape(3,1))

            # normalize the length of the d_jep vector to avoid bad stuff?
            # d_tau_mag_desired = 0.75 #1.0 #2.0 #set magnitude of d_tau during search
            # d_tau_mag = np.linalg.norm(d_tau)
            # d_tau = (d_tau / d_tau_mag) * d_tau_mag_desired

            # find change to JEP that will result in desired torque at the joints
            # dJEP = inv(K)(tau - tau_current)
            # d_jep = (q_eq-np.matrix(q).reshape(3,1))#*0.01 #SHOULD DO A FIXED STEP IN THIS DIRECTION???

            # jep += d_jep.A1
            #jep = q_eq.A1

            d_jep = (q_eq.A1-jep)*0.01
            jep += d_jep


            ######## STOPPING CONDITIONS FOLLOW THIS LINE ########


            if f_mag_list != [] and max(f_mag_list) > 100.:
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

