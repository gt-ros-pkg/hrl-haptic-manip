#!/usr/bin/env python
#
#
# Copyright (c) 2013, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \authors: Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
# \adviser: Charles Kemp (Healthcare Robotics Lab, Georgia Tech.)

#system level imports
import sys, os
import numpy as np, math
import scipy.io
import copy
from threading import RLock, Timer
import time
from math import pi
import itertools as it
from scipy.linalg import expm

#ROS or folder level imports
import roslib;
roslib.load_manifest('hrl_dynamic_mpc')
import rospy
import skin_client as sc
import hrl_lib.viz as hv
import hrl_lib.transforms as tr
import hrl_lib.util as ut
import hrl_lib.circular_buffer as cb
import tf
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Quaternion, PointStamped
from hrl_haptic_manipulation_in_clutter_msgs.msg import MpcDynFormattedData, RobotHapticState, HapticMpcState
from hrl_haptic_manipulation_in_clutter_srvs.srv import HapticMPCLogging
from hrl_msgs.msg import FloatArrayBare

########################this should be a param in the yaml files, or as passed in at least###############
# This is the number of total contacts that our current controller
# formulation can handle in the cost function.  However, all contacts
# are included in the dynamic prediction. This should be updated if
# the CVXGEN controller is modified.
num_poss_contacts = 10
########################this should be a param in the yaml files, or as passed in at least###############

class Python2Cvxgen():
    def __init__(self, M, c, c_mat, g, params, robot_client,
                 skin_topic_joints={},
                 base_frame="torso_lift_link",
                 planar=False, options=None, monitor = False, check_ee_motion=False):

        self.check_ee_motion = check_ee_motion
        self.jts_dict = skin_topic_joints
        self.topic_list = self.jts_dict.keys()
        self.controller_time = []
        self.loop_time = []

        #skin specific stuff
        self.positions = {}
        self.rotations = {}
        self.skin = sc.SkinClient(self.topic_list)
        rospy.sleep(1.0)
        self.base_frame = base_frame

        try:
            self.tf_lstnr = tf.TransformListener()
            for topic in self.topic_list:
                self.tf_lstnr.waitForTransform(self.base_frame, self.skin.frames[topic], rospy.Time(0), rospy.Duration(20.0))
                t1, q1 = self.tf_lstnr.lookupTransform(self.base_frame, self.skin.frames[topic], rospy.Time(0))
                t1 = np.matrix(t1).reshape(3,1)
                r1 = tr.quaternion_to_matrix(q1)
                self.positions[topic] = t1
                self.rotations[topic] = r1
        except:
            rospy.loginfo("Not getting the frames I expected for transforming skin messages, exiting ...")
            sys.exit()

        self.prev_q = None
        self.robot_state = robot_client #dc.DarciClient() #RobotHapticStateServer(opt)
        rospy.sleep(1.)
        self.robot_state.updateHapticState()
        #print self.robot_state.joint_angles
        self.monitor = monitor
        self.dist_sum = None
        self.cur_marker_pub = rospy.Publisher('/epc_skin/viz/cur', Marker)
        self.goal_marker_pub = rospy.Publisher('/epc_skin/viz/goal', Marker)
        #self.u_cur = [0.0]*4
        self.nom_stiffness = 10000.
        self.max_force = 0.
        self.q_s = []
        self.qd_s = []
        self.jt_pos = []
        self.jt_rot = []
        self.jt_axis = [1, 0, 2, 1, 2, 1, 0, 0]
        self.A_tl_s = []
        self.A_tr_s = []
        self.A_bl_s = []
        self.A_br_s = []
        self.B_t1_s = []
        self.B_t2_s = []
        self.B_t3_s = []
        self.B_b1_s = []
        self.B_b2_s = []
        self.B_b3_s = []
        self.torque_max = None
        self.torque_min = None
        self.q_des_cur_s = []
        self.u_s = []
        self.max_forces = []
        self.times =[]
        self.start_time = time.time()
        self.cur_forces = []
        self.result = None
        self.updated_state = False
        self.time_haptic_state = 0.
        self.time_populate_msg = 0.
        self.lock = RLock()
        self.max_time_to_fill = 0
        self.planar = planar

        self.M = M
        self.c = c
        self.c_mat = c_mat
        self.g = g
        self.params = params

        if options.start_config != None:
            q_config_string = options.start_config.split(',')
            q_config = [float(coord) for coord in q_config_string]
            self.q_config = q_config
            self.start_sequence_for_reaching(q_config)
            time.sleep(2.0)

        #get initial robot state before starting controller and parameters
        self.robot_state.updateHapticState()
        # this is end effector Cartesian position
        self.x_h = self.robot_state.end_effector_position.reshape(3,1)
        # this is end effector quaternion orientation
        self.q_h_orient = self.robot_state.end_effector_orient_quat

        # current joint angles, velocities and commanded joint angles
        self.q = self.robot_state.joint_angles
        self.q_dot = self.robot_state.joint_velocities
        self.jep = self.robot_state.desired_joint_angles
        self.J_h = self.robot_state.kinematics.Jacobian(self.q)  

        # joint impedance parameters
        self.kp = self.robot_state.joint_stiffness
        self.Kp = np.diag(self.kp)
        self.kd = self.robot_state.joint_damping
        self.Kd = np.diag(self.kd)

        # initializing contact state
        self.values_l = [] 
        self.n_l = [] 
        self.Jc_l = [] 

        self.u_0 = [0.]*len(self.q)
        self.n_jts = len(self.q) 

        # these variables are only used for checking if the end effector is moving
        self.hist_size = 20000
        self.ee_pos_buf = cb.CircularBuffer(self.hist_size, (3,))
        self.ee_gaussian_mn_buf_50 = cb.CircularBuffer(self.hist_size, (3,))
        self.ee_gaussian_cov_buf_50 = cb.CircularBuffer(self.hist_size, (3,3))
        self.mean_motion_buf = cb.CircularBuffer(self.hist_size, ())


        self.publish_cur_pose = rospy.Publisher('/haptic_mpc/gripper_pose', PoseStamped, latch=True)
        self.mpc_answer = rospy.Publisher("/delta_jep_mpc_cvxgen", FloatArrayBare, tcp_nodelay=True)
        self.goal_subscriber = rospy.Subscriber('haptic_mpc/goal_pose', PoseStamped, self.get_goal_cb)

        self.delta_t = 0.04
        self.ee_timeout = 10.0
        self.rate = rospy.Rate(1./self.delta_t)

        ############all of this should come from the param server, but not yet#########
        if options.f_thresh is None:
            self.f_thresh = 5.
        else:
            self.f_thresh = options.f_thresh
        self.dist_thresh = 0.005

        # new controller form
        # old params that definitely worked on Darci
        # self.alpha = 226. # matlab is 275  #0.5 #10.0 #0.01 # 1.0 #0.1
        # self.beta = 229. #10. #10000000.  #100000. value gives error = 4.0 for 3N over #*self.nom_stiffness
        # #FIX ME
        # self.zeta = 10. #0.01  #0.2 #1000. #1000. #1000. #1000. #100000. #100000. #0.3 #0.01 #0.1 #10. #10 #0.001 #1.
        # self.mu = 34. #10.0 #matlab is 10   #40. #5.0 #3. #0.001 #0.01 #00003 #00002 #0.00003 #1000000.
        # self.kappa = 10 #0.1 #1. #0.01 #1.
        # self.u_max_nom = 0.08  #10.
        # self.delta_t_impulse = 5.0 #10.0 #0.30 #16
        # self.u_slew_max = self.u_max_nom/50.0 #self.u_max_nom/100.0 #this worked pretty well
        # self.waypoint_mag = 1.0 #0.08


        ## Phi -- Where do these magic numbers come from?  Should they be parameters somewhere?
        self.alpha = 239.
        self.beta = 255.
        self.zeta = 0.743 #100. #was 0.743  but was jerky
        self.mu = 15. #80, 15 from sim anneal, was 34
        self.kappa = 1 #0.1 #1. #0.01 #1.

        if options.t_impulse is None:
            self.delta_t_impulse = 2.0 #0.5  for f_thresh of 15
        else:
            self.delta_t_impulse = options.t_impulse
        self.waypoint_mag = 1.0 #1.0 #0.08
        self.force_rate = 93.0 #0.001 #0.05 ##1.0 was working ok but jerky

        self.q_min = self.robot_state.kinematics.joint_lim_dict['min'].flatten().tolist()
        self.q_max = self.robot_state.kinematics.joint_lim_dict['max'].flatten().tolist()

        # i believe these are unused in Darci, but they were used in Kreacher - marc
        self.u_max_nom = 94.0  #10.
        self.u_slew_max = self.u_max_nom/50.0 #self.u_max_nom/100.0 #this worked pretty wel
        self.qd_max = [0.3]*self.n_jts  #[0.01]*3

        #########NEEDS UPDATING#####################
        # if use_orientation:
        #     self.orient_weight = 4.
        # else:
        #     self.orient_weight = 0.
        #########NEEDS UPDATING#####################

        if options.goal == None:
            self.x_g = self.x_h
        else:
            goal_string = options.goal.split(',')
            goal_ls = [float(coord) for coord in goal_string]
            self.x_g = np.matrix(goal_ls).reshape(3,1)

        print "GOAL is :", self.x_g
        current = PoseStamped()
        current.header.stamp = rospy.Time.now()
        current.header.frame_id = self.base_frame

        current.pose.position.x = self.robot_state.end_effector_position[0,0]
        current.pose.position.y = self.robot_state.end_effector_position[1,0]
        current.pose.position.z = self.robot_state.end_effector_position[2,0]
        current.pose.orientation.x = self.robot_state.end_effector_orient_quat[0]
        current.pose.orientation.y = self.robot_state.end_effector_orient_quat[1]
        current.pose.orientation.z = self.robot_state.end_effector_orient_quat[2]
        current.pose.orientation.w = self.robot_state.end_effector_orient_quat[3]
        self.publish_cur_pose.publish(current)

        # # this is the goal orientation input as a matrix
        # mat_g_orient = np.matrix(np.eye(3))

        # if mat_g_orient == None or self.orient_weight == 0:
        #     self.orient_weight = 0.
        #     self.q_g_orient = tr.matrix_to_quaternion(np.matrix(np.eye(3)))
        # else:
        #     self.q_g_orient = tr.matrix_to_quaternion(mat_g_orient)

        #printing options
        np.set_printoptions(precision=5)
        np.set_printoptions(suppress=True)


    def start_sequence_for_reaching(self, q_config):
        print "doing start sequence for reaching."
        cmd_list = [# [0]*7,
                    # [-0.0105, -0.0486, 1.266, 1.596, 0.2717, 0.0338, 0.0774],
                    # [-0.259, -0.7841, 1.232, 2.006, 0.0763, 0.217, 0.01807],
                    q_config]

        for cmd in cmd_list:
            self.robot_state.setDesiredJointAngles(cmd)
            self.robot_state.updateSendCmd()
            time.sleep(2.)

    def get_skin_data(self):
        start = time.time()

        self.skin.updateSkinInTorsoFrame(self.positions, self.rotations, self.q)
        locs = self.skin.getLocs()
        nrmls = self.skin.getNormals()
        values = self.skin.getValues()

        #print "getting data took :", time.time()-start
        next =time.time()
        locs_out = []
        values_out = []
        nrmls_out = []
        jts_buff = []
        Jc_l = []
        for topic in self.topic_list:
            if locs[topic] != []:
                if locs_out == []:
                    locs_out = locs[topic]
                    nrmls_out = nrmls[topic]
                    values_out = values[topic]
                else:
                    locs_out = np.vstack((locs_out,locs[topic]))
                    nrmls_out = np.vstack((nrmls_out, nrmls[topic]))
                    values_out = np.vstack((values_out,values[topic]))
                jts_buff = jts_buff + [self.jts_dict[topic]]*len(locs[topic].tolist())

        # #hacky test
        # pos, rot = self.robot_state.kinematics.FK(self.q, 7)
        # locs_out = [pos]
        # values_out = [np.matrix([-4.0, 0, 0])]
        # nrmls_out = [np.matrix([-1, 0, 0])]
        # jts_buff = [7]

        #print "doing vstack took :", time.time()-next
        next = time.time()

        Jc_l = [self.robot_state.kinematics.Jacobian(self.q, np.matrix(locs_out[i]).reshape(3,1))[0:3,:] for i in xrange(len(locs_out))]

        for i in xrange(len(locs_out)):
            Jc_l[i][:,jts_buff[i]+1:] = 0.

        # print "values are :\n", values_out
        # print "nrmls are :\n", nrmls_out
        # print "jacobians are :\n", Jc_l

        return values_out, nrmls_out, Jc_l

    def get_goal_cb(self, msg):
        with self.lock:
            self.x_g[0,0] = msg.pose.position.x
            self.x_g[1,0] = msg.pose.position.y
            self.x_g[2,0] = msg.pose.position.z

    def delta_jep_cb(self, msg):
        with self.lock:
            self.u_cur = msg.data

    # Called by control loop to update history of motion of end effector
    def updateEeMotion(self):
        # fit Gaussian to last 50 locations of the end effector.
        self.updateEeGaussianBuf(self.ee_gaussian_mn_buf_50,
                                 self.ee_gaussian_cov_buf_50, 50)
        # find distance (in 2D) between the means of the Gaussians
        # that are 200 samples apart.
        self.updateMeanMotion(self.ee_gaussian_mn_buf_50, step=200)

    def is_ee_moving(self, distance_thresh):
        if len(self.mean_motion_buf) > 0 and \
           self.mean_motion_buf[-1] < distance_thresh:
            n = min(len(self.mean_motion_buf), 5)
            return False
        return True

    def updateEeGaussianBuf(self, mn_buf, cov_buf, hist_size):
        if len(self.ee_pos_buf) < hist_size:
            return
        ee_hist = self.ee_pos_buf.get_last(hist_size)
        ee_hist = np.matrix(ee_hist).T
        mn_buf.append(np.mean(ee_hist, 1).A1)
        cov_buf.append(np.cov(ee_hist))

    def updateMeanMotion(self, mn_buf, step):
        if len(mn_buf) < step:
            return
        d = np.linalg.norm((mn_buf[-1] - mn_buf[-step])[0:2])
        self.mean_motion_buf.append(d)


    # def calc_com_jacobian(self, q, com, joint_ind):
    #     J_com = self.robot.kinematics.jacobian(q, com)
    #     J_com[:,joint_ind] = 0.
    #     return J_com

    def discretize(self, A, B):
        #I = np.eye(A.shape[0])
        #psy = I
        #N = 500   #this MAY NOT WORK FOR MULTI-LINK and is a limiting factor NEEDS TO BE MOVED TO cython or something
        #for ii in xrange(N-1):
        #    psy = I + A*self.delta_t/(N-ii)*psy
        #
        #A_d = I + A*self.delta_t*psy
        #B_d = self.delta_t*psy*B

        if np.linalg.cond(A) < 1/sys.float_info.epsilon:
            pass
        else:
            print "'A' matrix in continuous dynamics is VERY poorly conditioned (> 1/machine_epsilon)"
            print "EXITING ..."
            sys.exit()

        I = np.eye(A.shape[0])
        A_d = np.matrix(expm(A*self.delta_t, 13))
        B_d = np.linalg.solve(A, (A_d - I)*B) #this is the same as inverting A, but more efficient


        return A_d, B_d

    def generate_mpc_data(self, delta_x_g, J_h, q, q_dot, q_des_cur, u_cur,
                          Kp, Kd, Mass, Coriolis, Gravity, q_min, q_max, qd_max,
                          values_n, values_l, 
                          n_l, Jc_l, Kc_l):
        with self.lock:
            start_fill = time.time()
            num_joints = self.n_jts

            # change in goal position for end effector
            self.out_delta_x_d = (delta_x_g).A1.tolist()

            # end effector Jacobian
            self.out_J = J_h[0:3,:].flatten('F').A1.tolist()

            # initial state in joint angles and velocities
            self.out_q_0 = list(q) 
            self.out_qd_0 = list(q_dot) #[0]*7 #list(q_dot)

            self.out_q_des_cur_0 = list(q_des_cur)

            # joint impedance parameters
            self.Kp_pass = Kp.flatten('F').tolist()
            self.Kd_pass = Kd.flatten('F').tolist()

            # mass matrix
            self.out_mass = Mass.flatten('F').A1.tolist()

            # maximum and minimum available joint torque and this includes available torque due to gravity
            self.torque_max = (np.array([14.6, 16.7, 9.6, 9.6, 1.9, 2.3, 2.3]) - np.array(self.Gravity)).tolist()
            self.torque_min = (-np.array([14.6, 16.7, 9.6, 9.6, 1.9, 2.3, 2.3]) - np.array(self.Gravity)).tolist()

            # we have to saturate this to be physically plausible from
            # previous calculation (i.e. our gravity estimate is not
            # perfect and therefore we sometimes get both max and min
            # with same sign for example).
            for i in xrange(len(self.torque_max)):
                if self.torque_max[i] < 0:
                    self.torque_max[i] = 0
                if self.torque_min[i] > 0:
                    self.torque_min[i] = 0


            # joint limits
            self.out_q_max = q_max
            self.out_q_min = q_min

            # max allowable change in commanded joint angle
            u_max = [self.u_max_nom]*len(q_max) 
            self.out_u_max = u_max


            max_impulse_test = [self.f_thresh*0.02*delta_t_impulse]*len(q)
            self.out_tau_max_delta_t = max_impulse_test

            ##########this is code that was used when I was calculating the worst case scenario for each link differently #########################
                # for kk in xrange(len(q)):
                #     max_dist = 0
                #     max_pos = None
                #     for jj in xrange(len(q)+1):
                #         if jj > kk:
                #             #FIX ME START HERE
                #             #this seemed to work better without the negative 1, I SHOULD CHECK THIS
                #             z_dir = -1*self.jt_rot[jj][:,self.jt_axis[jj]].T  # negative is due to Advait's original convention which is opposite meka's
                #             dist_nom = self.jt_pos[jj] - self.jt_pos[kk]
                #             dist_orth = dist_nom.A1 - np.dot(dist_nom.A1, z_dir.A1)*z_dir.A1
                #             dist = np.linalg.norm(dist_orth)
                #             if dist > max_dist:
                #                 max_dist = dist
                #                 max_pos = self.jt_pos[jj]

                #     # this works better if you follow the DH parameter convention can just take z-axis (3rd column of rotation matrix)
                #     z_dir_axis = -1*self.jt_rot[kk][:,self.jt_axis[kk]].T

                #     dist = max_pos-self.jt_pos[kk]
                #     dist_orth = dist.A1 - np.dot(dist.A1, z_dir_axis.A1)*z_dir_axis.A1

                #     f_dir = np.cross(z_dir_axis, dist_orth)
                #     tau_impulse_max = np.linalg.norm(np.cross(dist_orth, f_dir*self.f_thresh))

                #     tau_impulse_ls.append(tau_impulse_max)
                #     max_impulse_test.append(tau_impulse_max*delta_t_impulse)
            ##########this is code that was used when I was calculating the worst case scenario for each link differently #########################

            # filling the matrices for estimating change in force from predicted change in joint angles
            # as well as the desired decrease in contact force when forces are above threshold
            n_K_J_all_buff = []
            delta_force_max_buff = []
            delta_rate_force_max_buff = []

            # total current joint torques caused by contact
            tau_cont_sum_0 = np.matrix(np.zeros(num_joints)).reshape(num_joints, 1)
            
            # summation of all contact stiffnesses in joint space
            all_J_T_K_J = np.matrix(np.zeros((num_joints, num_joints)))

            #this calculation can be made into matrix multiplication
            #to increase efficiency outside of for loop if it is bottleneck

            for ii in xrange(len(Jc_l)):
                # summing joint torques due to contact
                tau_cont_sum_0 = tau_cont_sum_0 - Jc_l[ii].T*values_l[ii].T #this is minus due to sign convention in skin client

                # summation of all contact stiffnesses in joint space
                all_J_T_K_J = all_J_T_K_J + Jc_l[ii].T*Kc_l[ii]*Jc_l[ii]

                # difference between allowable threshold and current force
                diff = self.f_thresh - values_n[ii,0]

                buff = n_l[ii]*Kc_l[ii]*Jc_l[ii]  

                # this scaling is hacky, it works with CVXGEN, but
                # should be more rigorous
                if abs(diff) < 0.01:
                    scaling_factor = 1.0
                else:
                    scaling_factor = 1./(abs(diff)*100.0)

                # allowable rate of change and change in contact force
                delta_rate_force_max_buff.append(self.force_rate*scaling_factor)
                delta_force_max_buff.append(diff*scaling_factor)

                buff_list = (scaling_factor*buff).A1.tolist()
                n_K_J_all_buff.append(buff_list)

            # if the number of contacts is greater than the amount
            # specified in the CVXGEN solver, we sort the forces and
            # enforce the cost for the top X number of
            # contacts. However, all contacts are used to predict the
            # dynamics.
            forces_sorted = False
            if len(values_n) > num_poss_contacts:
                forces_sorted = True
                sorted_values, n_K_J_all_buff, delta_force_max_buff, delta_rate_force_max_buff = (list(t) for t in zip(*sorted(zip(values_n,
                                                                                                                                   n_K_J_all_buff,
                                                                                                                                   delta_force_max_buff,
                                                                                                                                   delta_rate_force_max_buff))))
            self.out_tau_cont_sum_0 = tau_cont_sum_0.A1.tolist()

            cor_mat = Coriolis

            #these are the continuous time state space equations at the current time step
            A = np.matrix(np.vstack((np.hstack((np.linalg.solve(Mass, -Kd-cor_mat), np.linalg.solve(Mass, -Kp-all_J_T_K_J))),
                          np.hstack((np.eye(num_joints), np.zeros((num_joints, num_joints)))))))

            I_mass = np.eye(num_joints)
            B = np.matrix(np.vstack((
                          np.hstack((np.linalg.solve(Mass, Kp), np.linalg.solve(Mass, I_mass), np.linalg.solve(Mass, all_J_T_K_J))),
                          np.zeros((num_joints, 3*num_joints)))))

            # this function discretizes the continuous time equations
            A_d, B_d = self.discretize(A, B)

            # this is a check that our discretization is still stable, if not the controller dies
            eigval, eigvec = np.linalg.eig(A_d)
            if np.max(np.abs(eigval)) > 1.0:
                print "EIGEN VALUES FOR A ARE UNSTABLE (i.e. greater than one)"
                print "A is :\n", A
                print "mass is :\n", Mass
                print "Kd is :\n", Kd
                #print "cor_mat is :\n", cor_mat
                print "Kp is :\n", Kp
                print "all_J_T_K_J :\n", all_J_T_K_J
                print "EXITING ..."
                sys.exit()

            # we have to break the discrete time state space matrices
            # into sub-matrices for use in CVXGEN.
            A_tl = A_d[0:num_joints, 0:num_joints]
            A_tr = A_d[0:num_joints, num_joints:num_joints*2]
            A_bl = A_d[num_joints:num_joints*2, 0:num_joints]
            A_br = A_d[num_joints:num_joints*2, num_joints:num_joints*2]

            B_t1 = B_d[0:num_joints, 0:num_joints]
            B_t2 = B_d[0:num_joints, num_joints:num_joints*2]
            B_t3 = B_d[0:num_joints, num_joints*2:num_joints*3]
            B_b1 = B_d[num_joints:num_joints*2, 0:num_joints]
            B_b2 = B_d[num_joints:num_joints*2, num_joints:num_joints*2]
            B_b3 = B_d[num_joints:num_joints*2, num_joints*2:num_joints*3]

            self.out_A_tl = A_tl.flatten('F').A1.tolist()
            self.out_A_tr = A_tr.flatten('F').A1.tolist()
            self.out_A_bl = A_bl.flatten('F').A1.tolist()
            self.out_A_br = A_br.flatten('F').A1.tolist()
            self.out_B_t1 = B_t1.flatten('F').A1.tolist()
            self.out_B_t2 = B_t2.flatten('F').A1.tolist()
            self.out_B_t3 = B_t3.flatten('F').A1.tolist()
            self.out_B_b1 = B_b1.flatten('F').A1.tolist()
            self.out_B_b2 = B_b2.flatten('F').A1.tolist()
            self.out_B_b3 = B_b3.flatten('F').A1.tolist()

            #### this term can probably disappear now, but need to check cvxgen first ############
            self.out_all_J_T_K_J = all_J_T_K_J.flatten('F').A1.tolist()

            # this is a check for any contact at all before populating
            # matrices for the maximum allowed rate of change in
            # contact force
            if delta_rate_force_max_buff == []:
                delta_rate_force_max = np.ones(num_poss_contacts)*0.01
            elif forces_sorted == True:
                delta_rate_force_max = np.array(delta_rate_force_max_buff)[-num_poss_contacts:]
            else:
                delta_rate_force_max = np.hstack((np.array(delta_rate_force_max_buff),
                                                  0.01*np.ones(num_poss_contacts-len(delta_rate_force_max_buff))))

            #this is a check for any contact at all before populating
            #matrices for the maximum allowed change in contact force
            if delta_force_max_buff == []:
                delta_force_max = np.ones(num_poss_contacts)*1
            elif forces_sorted == True:
                delta_rate_force_max = np.array(delta_force_max_buff)[-num_poss_contacts:]
            else:
                delta_force_max = np.hstack((np.array(delta_force_max_buff),
                                             1.*np.ones(num_poss_contacts-len(delta_force_max_buff))))

            # this is the projection of the stiffness times the
            # contact Jacobian along the surface normal
            if n_K_J_all_buff == []:
                n_K_J_all = np.zeros((num_poss_contacts, len(q)))
            elif forces_sorted == True:
                n_K_J_all = np.array(n_K_J_all_buff)[-num_poss_contacts:, :]
            else:
                n_K_J_all = np.array(n_K_J_all_buff)
                n_K_J_all = np.vstack((n_K_J_all, np.zeros((num_poss_contacts-n_K_J_all.shape[0], len(q)))))

            # assigning the output from the last three calculations
            self.out_n_K_J_all = n_K_J_all.flatten('F').tolist()
            self.out_delta_f_max = delta_force_max.tolist()
            self.out_delta_rate_f_max = delta_rate_force_max.tolist()

            ############ all of the code in the following section was used for controller and prediction debugging ##########
            # save_for_prediction_mismatch = False
            # if save_for_prediction_mismatch:
            #     self.q_s.append(q)
            #     self.qd_s.append(q_dot)
            #     self.A_tl_s.append(A_tl)
            #     self.A_bl_s.append(A_bl)
            #     self.A_tr_s.append(A_tr)
            #     self.A_br_s.append(A_br)
            #     self.B_t1_s.append(B_t1)
            #     self.B_t2_s.append(B_t2)
            #     self.B_t3_s.append(B_t3)
            #     self.B_b1_s.append(B_b1)
            #     self.B_b2_s.append(B_b2)
            #     self.B_b3_s.append(B_b3)
            #     self.q_des_cur_s.append(list(q_des_cur))
            #     self.times.append(time.time())

            # save_matlab = False
            # if save_matlab:
            #     matlab_dict = {}
            #     self.q_s.append(q)
            #     self.qd_s.append(q_dot)
            #     if values_n.shape[0] == 0:
            #         self.max_forces.append(0.0)
            #     else:
            #         self.max_forces.append(np.max(values_n))
            #     self.times.append(rospy.get_time())

            #     matlab_dict['q_s'] = self.q_s
            #     matlab_dict['qd_s'] = self.qd_s
            #     matlab_dict['max_forces'] = self.max_forces
            #     matlab_dict['times'] = self.times
            #     matlab_dict['alpha'] = self.alpha
            #     matlab_dict['beta'] = self.beta
            #     matlab_dict['kappa'] = self.kappa
            #     matlab_dict['zeta'] = self.zeta
            #     matlab_dict['mu'] = self.mu
            #     matlab_dict['A'] = A
            #     matlab_dict['B'] = B
            #     matlab_dict['A_tl'] = A_tl
            #     matlab_dict['A_tr'] = A_tr
            #     matlab_dict['A_bl'] = A_bl
            #     matlab_dict['A_br'] = A_br
            #     matlab_dict['B_t1'] = B_t1
            #     matlab_dict['B_t2'] = B_t2
            #     matlab_dict['B_t3'] = B_t3
            #     matlab_dict['B_b1'] = B_b1
            #     matlab_dict['B_b2'] = B_b2
            #     matlab_dict['B_b3'] = B_b3
            #     matlab_dict['delta_x_d'] = delta_x_g[0:3]
            #     matlab_dict['J'] = J_h[0:3,:]
            #     matlab_dict['q_0'] = q
            #     matlab_dict['qd_0'] = q_dot
            #     matlab_dict['q_des_cur_0'] = q_des_cur
            #     matlab_dict['q_min'] = q_min
            #     matlab_dict['q_max'] = q_max
            #     matlab_dict['qd_max'] = qd_max
            #     matlab_dict['u_max'] = u_max
            #     matlab_dict['delta_f_max'] = delta_force_max
            #     matlab_dict['delta_rate_force_max'] = delta_rate_force_max
            #     matlab_dict['u_slew_max'] = self.u_slew_max
            #     matlab_dict['f_max_delta_t'] = max_impulse
            #     matlab_dict['tau_max_delta_t'] = max_impulse_test
            #     matlab_dict['tau_cont_sum_0'] = tau_cont_sum_0
            #     matlab_dict['n_K_J_all'] = n_K_J_all
            #     matlab_dict['all_J_T_K_J'] = all_J_T_K_J
            #     matlab_dict['torque_max'] = self.torque_max
            #     matlab_dict['torque_min'] = self.torque_min
            #     matlab_dict['Kp'] = Kp
            #     matlab_dict['Kd'] = Kd
            #     matlab_dict['mass'] = Mass
            #     keys = matlab_dict.keys()
            #     keys.sort()
            #     for key in keys:
            #         print " key is :", key
            #         print matlab_dict[key]
            #     scipy.io.savemat('./darci_same_data_test.mat', matlab_dict)
            ############ all of the code in the following section was used for controller and prediction debugging ##########

    def get_contact_stiffnesses(self, n_l, k_default):
        Kc_l = []
        for n_ci in n_l:
            # n_ci is a unit vector located at the point of contact,
            # or at the center of the taxel, that is normal to the
            # surface of the robot and points away from the surface of
            # the robot.
            #
            # This should result in a stiffness matrix that has a
            # stiffness of k in the direction of the normal and a
            # stiffness of 0 in directions orthogonal to the normal.
            n_ci = np.nan_to_num(n_ci)
            K_ci = np.outer(n_ci, n_ci)
            K_ci = k_default * K_ci
            Kc_l.append(np.matrix(K_ci))
        return Kc_l

    def publish_goal_marker(self, goal_pos, frame):
        o = np.matrix([0.,0.,0.,1.]).T
        g_marker = hv.single_marker(goal_pos, o, 'sphere',
                            frame, color=(0., 1., 1., 1.),
                            scale = (0.02, 0.02, 0.02),
                            duration=0., m_id=1)

        g_marker.header.stamp = rospy.Time.now()
        self.goal_marker_pub.publish(g_marker)

        cur_marker = hv.single_marker(self.x_h, o, 'sphere',
                                      frame, color=(1., 0., 0., 1.),
                                      scale = (0.02, 0.02, 0.02),
                                      duration=0., m_id=2)
        self.cur_marker_pub.publish(cur_marker)

    def send_data_to_cvxgen(self):
        # marking the time before we call the optimization
        start = time.time()

        # calling the optimizaiton with formatted data
        u = cvxgen_controller.runController(self.alpha,
                                            self.beta,
                                            self.kappa,
                                            self.zeta,
                                            self.mu,
                                            self.out_delta_x_d,
                                            self.out_J,
                                            self.out_A_tl,
                                            self.out_A_tr,
                                            self.out_A_bl,
                                            self.out_A_br,
                                            self.out_B_t1,
                                            self.out_B_t2,
                                            self.out_B_t3,
                                            self.out_B_b1,
                                            self.out_B_b2,
                                            self.out_B_b3,
                                            self.out_q_0,
                                            self.out_qd_0,
                                            self.out_q_des_cur_0,
                                            self.out_q_min,
                                            self.out_q_max,
                                            self.torque_max,
                                            self.torque_min,
                                            self.Kp_pass,
                                            self.Kd_pass,
                                            self.out_tau_max_delta_t,
                                            self.out_tau_cont_sum_0,
                                            self.out_mass,
                                            self.out_delta_f_max,
                                            self.out_delta_rate_f_max,
                                            self.out_n_K_J_all)

        self.prev_q = self.q
        ### this code was used for checking optimization output and timing ############
        solver_time = time.time() - start
        # print "SOLVER TOOK :", solver_time, "seconds"
        #self.controller_time.append(solver_time)
        # for i in xrange(len(u)):
        #     print "u at time ", i, " is :", u[i]
        ### this code was used for checking optimization output and timing ############

        return u[0]

    def update_data(self):
        stop = ''

        with self.lock:
            # visualizing current goal
            self.publish_goal_marker(self.x_g, '/world')
            #start_update = time.time()
            self.updated_state = False

            # update the current robot data using the client (end
            # effector position, orientation, Jacobian, joint angles,
            # joint velocities, commanded joint angles, joint
            # stiffness and damping)
            self.robot_state.updateHapticState()
            self.x_h = self.robot_state.end_effector_position.reshape(3,1)
            self.q_h_orient = self.robot_state.end_effector_orient_quat
            self.J_h = self.robot_state.J_h
            self.q = self.robot_state.joint_angles
            self.q_dot = self.robot_state.joint_velocities
            self.jep = self.robot_state.desired_joint_angles
            self.kp = self.robot_state.joint_stiffness
            self.Kp = np.diag(self.kp)
            self.kd = self.robot_state.joint_damping
            self.Kd = np.diag(self.kd)

            ########## this code was only used when I varied the location of the worst possible impact ##################
            # self.jt_pos = []
            # self.jt_rot = []
            # for kk in xrange(len(self.q)+1):
            #     pos, rot = self.robot_state.kinematics.FK(self.q, kk)  
            #     self.jt_pos.append(pos)
            #     self.jt_rot.append(rot)
            ########## this code was only used when I varied the location of the worst possible impact ##################
          

            # This is used to get transform between the skin topic
            # frames and the robot base link.  This should eventually
            # move using the forward kinematics instead of TF.
            for topic in self.topic_list:
                self.tf_lstnr.waitForTransform(self.base_frame, self.skin.frames[topic], rospy.Time(0), rospy.Duration(20.0))
                t1, q1 = self.tf_lstnr.lookupTransform(self.base_frame, self.skin.frames[topic], rospy.Time(0))
                t1 = np.matrix(t1).reshape(3,1)
                r1 = tr.quaternion_to_matrix(q1)
                self.positions[topic] = t1
                self.rotations[topic] = r1
                
            # returns the current contact information in the base link (torso) frame
            self.values_l, self.n_l, self.Jc_l = self.get_skin_data()
            values_l = self.values_l
            n_l = self.n_l
            Jc_l = self.Jc_l

            # calculating the magnitude of each force
            values_n = np.matrix([np.linalg.norm(values_i) for values_i in values_l]).T
            self.cur_forces = values_n


            if self.planar == True:
                J_h = self.J_h[0:2]
            else:
                J_h = self.J_h[0:3]

                ######### this part should be adjusted and added back if orientation is finally included #########################
                # J_h = copy.copy(self.J_h)
                # T_quat = 0.5 * (q_h_orient[3]
                #                 * np.matrix(np.eye(3))
                #                 - mpc_util.get_skew_matrix(q_h_orient[0:3]))
                # J_h[3:] = T_quat*self.J_h[3:]
                ######### this part should be adjusted and added back if orientation is finally included #########################

            # this function returns a 3x3 matrix that represents
            # stiffness in a specific direction
            Kc_l = self.get_contact_stiffnesses(n_l, self.nom_stiffness)

            n = len(n_l)

            # calculate the current end effector error and magnitude (this would change with orientation)
            err = self.x_g-self.x_h
            err_mag = np.linalg.norm(err)

            # this if statement allows us to use integral control when
            # near the goal, but avoid wind-up or overshoot when far
            # away. This is currently a simple disturbance/error
            # integrator with somewhat smart saturation. It would be
            # better to have the disturbance be a function of the
            # model states.
            if err_mag < 0.08:
                # integral gain
                ki = 0.001  
                if self.dist_sum == None:
                    self.dist_sum = err
                else:
                    self.dist_sum += err*ki

                if self.planar:
                    goal_states = 2
                else:
                    goal_states = 3

                # if error times the current sum is negative, we set
                # it to zero in that direction for anti-windup
                for jj in xrange(goal_states):
                    if self.dist_sum[jj]*err[jj] < 0:
                        self.dist_sum[jj] = 0.0

                # we also cap the total integral term with "thresh"
                thresh = 0.08 
                if np.linalg.norm(self.dist_sum) > thresh:
                    self.dist_sum = self.dist_sum/np.linalg.norm(self.dist_sum)*thresh

            # We now can add the integral term onto our goal distance
            # if we are within the 8 cm threshold
            if err_mag < 0.08:
                if err_mag > self.waypoint_mag:
                    delta_pos_g = err/err_mag*self.waypoint_mag + self.dist_sum
                else:
                    delta_pos_g = err + self.dist_sum
            else:
                if err_mag > self.waypoint_mag:
                    delta_pos_g = err/err_mag*self.waypoint_mag
                else:
                    delta_pos_g = err


            # determine if arm is close enough to goal to just hold position
            self.keep_trying = True
            if values_n != []:
                if err_mag <= self.dist_thresh and np.max(values_n) < self.f_thresh:
                    self.keep_trying = False
            elif err_mag <= self.dist_thresh:
                self.keep_trying = False


            # we use this mostly in the batch trials for stopping criteria
            if self.monitor == True:
                # update history of how much the end effector is moving
                if self.check_ee_motion == True:
                    # fit Gaussian to last 50 locations of the end effector.
                    self.updateEeGaussianBuf(self.ee_gaussian_mn_buf_50,
                                             self.ee_gaussian_cov_buf_50, 50)
                    # find distance (in 2D) between the means of the Gaussians
                    # that are 1000 samples (10 seconds) apart.
                    self.updateMeanMotion(self.ee_gaussian_mn_buf_50, step=self.ee_timeout/self.delta_t)

                # arm got within 4 cm of goal
                if err_mag < 0.04:  #was 0.04 for canonical tests
                    print "SUCCESS"
                    self.result = 'success'
                    data = {'result':self.result}
                    ut.save_pickle(data, './result.pkl')
                    print "trying to die ... "
                    os._exit(0)
                    
                # arm didn't reach the goal within 20 seconds
                if (time.time()-self.start_time) > 20.:  #was 15.
                    print "OVERALL TIMEOUT"
                    self.result = 'timeout'
                    data = {'result':self.result}
                    ut.save_pickle(data, './result.pkl')
                    os._exit(0)
                    
                # arm had forces higher than 50 N as measured by skin
                if (values_n != []):
                    if np.max(values_n) > 50.:
                        print "HIGH FORCE FAILURE"
                        self.result = 'high_force_over_50N'
                        data = {'result':self.result}
                        ut.save_pickle(data, './result.pkl')
                        os._exit(0)

                # end effector moved less than 1 mm in 10 seconds
                if self.check_ee_motion == True:
                    if (self.is_ee_moving(0.001) == False):
                        self.result = 'not_moving'
                        data = {'result':self.result}
                        ut.save_pickle(data, './result.pkl')
                        print "EE is not moving"
                        os._exit(0)

            if self.planar:
                delta_x_g = delta_pos_g[0:2]
            else:
                delta_x_g = delta_pos_g
                ############### need to bring this back if add orientation again ################################
                # delta_orient_g = mpc_util.goal_orientation_in_quat(q_h_orient, q_g_orient, math.radians(0.1))
                # delta_x_g = np.vstack((delta_pos_g, delta_orient_g))
                ############### need to bring this back if add orientation again ################################

            # calculating mass, Coriolis and gravity terms
            self.Mass = np.matrix(self.M(self.params, self.q)).reshape(self.n_jts, self.n_jts)
            self.Coriolis = np.matrix(self.c_mat(self.params, self.q, self.q_dot))
            self.Gravity = self.g(self.params, self.q)

            # #print "time to prep up to filling message :", time.time()-start_update

            # this function does the last few calculations and then
            # flattens the data vectors to be one dimensional arrays
            # in column major format
            self.generate_mpc_data(delta_x_g,
                                   J_h,
                                   self.q, 
                                   self.q_dot, 
                                   self.jep,
                                   None, 
                                   self.Kp,
                                   self.Kd,
                                   self.Mass,
                                   self.Coriolis,
                                   self.Gravity,
                                   self.q_min,
                                   self.q_max,
                                   self.qd_max,
                                   values_n,
                                   values_l,
                                   n_l,
                                   Jc_l,
                                   Kc_l)
        return stop


def RunControlLoop(py2cvxgen, verbose=True):
    # these variables were used to check loop timing
    while_time = rospy.get_time()
    max_diff_time = 0.0

    try:
        # this is the control loop running at a rate specified in py2cvxgen object
        while not rospy.is_shutdown(): 
            ########### this is all for timing purposes and can be deleted or commented ######
            now = rospy.get_time()
            diff_time = now-while_time
            while_time = now
            if diff_time > max_diff_time:
                max_diff_time = diff_time
            ##################################################################################

            # this function calculates and formats the data for
            # passing to the CVXGEN optimization
            py2cvxgen.update_data()
            
            # this function calls the CVXGEN solver using boost python
            # and returns only the first set of control inputs which
            # is a change in commanded joint angles
            u_0 = py2cvxgen.send_data_to_cvxgen()

            # check if arm is within 5 mm of Cartesian position, otherwise, keep trying
            if py2cvxgen.keep_trying == True:
                # adding the change in commanded joint angles to the
                # current commanded angles
                py2cvxgen.robot_state.addDeltaToDesiredJointAngles(u_0)
                
                # sending the updated command through ROS
                py2cvxgen.robot_state.updateSendCmd()
            else:
                if verbose:
                    print "near goal with low forces, not moving"

            # sleeping at the specified rate, because this is python
            # and ROS, this is only approximate
            py2cvxgen.rate.sleep()

    except KeyboardInterrupt:
        print "killed by ctrl-c, exiting ..."
        sys.exit()

if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    ##### these two commented options may be useful again at some point when properly implemented ###########
    # p.add_option('--ignore_skin', '--is', action='store_true',
    #              dest='ignore_skin', help='ignore feedback from skin', default=False)

    # p.add_option('--use_orientation', '--uo', action='store_true', default=False,
    #              dest='use_orientation', help='try to go to commanded orientation in addition to position')
    # these two commented options may be useful again at some point when properly implemented
    #########################################################################################################

    # this option was used for batch mode, when the arm started with a
    # default config before each trial
    p.add_option('--q_config', action='store', dest='start_config', default = None,
                 help='first joint config before reaching')

    # again used for batch mode and allowed us to specify the goal
    # from another script that was randomly generating the goals
    p.add_option('--goal', action='store', dest='goal', default = None,
                 help='run with this goal')

    # this is the time duration of the impact (essentially becomes a
    # parameter to tune the speed at which the arm can move)
    p.add_option('--t_impulse', action='store', dest='t_impulse', default = None,
                 help='use this t_impulse', type='float')

    # allowable force threshold
    p.add_option('--f_thresh', action='store', dest='f_thresh', default = None,
                 help='use this force threshold value', type='float')

    # specify which robot to use
    p.add_option('--darci', action='store_true', dest='darci',
                 help = 'use 7 dof arm, darci ')
    p.add_option('--darci_sim', action='store_true', dest='darci_sim',
                 help = 'use 7 dof arm, darci_sim ')

    # specify which arm (left or right) to use
    p.add_option('--arm', action='store', dest='arm', default='l',
                 type='string', help='which arm to use (l or r)')

    # hide output for debugging purposes
    p.add_option('--noverbose', action='store_false', dest='verbose',
                 default=True, help = 'Print internal message ')

    opt, args = p.parse_args()
    rospy.init_node('cvxgen_data_formatter')

    if opt.darci == True:
        # this is CVXGEN controller that is wrapped using boost python
        import cvxgen_controller

        # this is the robot client for the robot DARCI
        import darci_client as dc

        # importing the functions to calculate the dynamics (mass,
        # gravity, Coriolis terms) of the robot arm
        if opt.arm == 'l':
            from darci_left_dynamics import M, c, c_mat, g
            from darci_left_dyn_params import params
        elif opt.arm == 'r':
            print "Right arm not supported on Darci.  Exiting..."
            sys.exit()

        # specify the base name of each skin topic and the proximal
        # joint for that sensor
        skin_topic_joints = {'fabric_forearm_sensor':4, 'fabric_wrist_sensor':6}

        # create object that calculates and formats data to pass to CVXGEN solver
        gen_data = Python2Cvxgen(M=M, c=c, c_mat=c_mat, g=g, params=params, robot_client=dc.DarciClient(),
                                 skin_topic_joints=skin_topic_joints,
                                 planar=False, options=opt, monitor=False)

    elif opt.darci_sim == True:
        import cvxgen_controller
        import darci_sim_client as dsc
        if opt.arm == 'l':
            from darci_left_dynamics import M, c, c_mat, g
            from darci_left_dyn_params import params
        elif opt.arm == 'r':
            print "Right arm not supported on darci_sim.  Exiting..."
            sys.exit()
        skin_topic_joints = {'forearmor_tactile_sensor':4, 'flipper_tactile_sensor':6}
        gen_data = Python2Cvxgen(M=M, c=c, c_mat=c_mat, g=g, params=params, robot_client=dsc.DarciSimClient(),
                                skin_topic_joints=skin_topic_joints,
                                planar=False, options=opt, monitor=False)

    # Killing the controller if other robot is not implemented
    else:
        print "please specify '--darci' as input, all other robots currently not implemented. Exiting ..."
        sys.exit()

    # starts the controller loop running at a specified rate, this function could be cleaned up and included in the Python2Cvxgen class
    RunControlLoop(gen_data, opt.verbose)

