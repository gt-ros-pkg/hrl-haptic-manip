#!/usr/bin/env python  

import sys, os
import numpy as np, math
import copy
from threading import RLock, Timer
import time
from math import pi
from utils import *

import roslib; 
#roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('sandbox_marc_darpa_m3')
import rospy
import itertools as it

import hrl_lib.transforms as tr
import hrl_lib.util as ut
import hrl_haptic_mpc.haptic_mpc_util as mpc_util
import hrl_haptic_mpc.multiarray_to_matrix as multiarray_to_matrix

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Quaternion, PointStamped
from hrl_haptic_manipulation_in_clutter_msgs.msg import MpcDynFormattedData, RobotHapticState, HapticMpcState
from hrl_haptic_manipulation_in_clutter_srvs.srv import HapticMPCLogging
from hrl_haptic_mpc.robot_haptic_state_node import RobotHapticStateServer
from threelink_dynamics import *


class CvxGenData():
    def __init__(self, robot_type, use_orientation=False):

        fake_args = ["-r", "sim3", "-s", "none", "-a", "l"]
        mpc_util.initialiseOptParser(p2)
        (opt, args) = p2.parse_args(fake_args)
        self.robot_state = RobotHapticStateServer(opt)
        # self.robot_state.updateHapticState()
        # print self.robot_state.joint_angles
        # assert(False)

        self.end_trial = False
        self.updated_state = False
        self.time_haptic_state = 0.
        self.time_populate_msg = 0.
        self.lock = RLock()
        self.J_h = None  # use Jeff's function to get msg.end_effector_jacobian in right format
        self.q = None  
        self.q_dot = None 
        self.kp = None 
        self.kd = None 
        self.Kp = None
        self.Kd = None
        self.J_cl = None ## use Jeff's function to get msg.contact_jacobians in right format
        self.values_l_msg = None #list of lists [[skin.values_x, skin.values_y, skin.values_z] for skin in msg.skins]
        self.n_l_msg = None #list of lists [[skin.normals_x, skin.normals_y, skin.normals_z] for skin in msg.skins]
        self.time_stamp_msg = None
        self.jep = None #desired_joint_angles: []
        self.x_h = None # hand_pose.position
        self.q_h_orient = None # hand_pose.orientation
        self.ma_to_m = multiarray_to_matrix.MultiArrayConverter()
        self.out_msg = MpcDynFormattedData() 
        #self.out_msg = None
        self.robot_type = robot_type

        if True:
            import gen_sim_arms as sim_robot
            import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as sim_robot_config
            self.robot_path = '/sim3'
            rospy.loginfo("Initialising Sim robot interface") 
            sim_config = sim_robot_config
            self.robot = sim_robot.ODESimArm(sim_config)
            rospy.sleep(2.)
            self.q = self.robot.get_joint_angles()
            self.kp, self.kd = self.robot.get_joint_impedance() 
            self.q_dot = self.robot.get_joint_velocities()
            self.jep = self.robot.get_ep()
            self.x_h, self.mat_h_orient = self.robot.kinematics.FK(self.q, len(self.q))
            self.q_h_orient = tr.matrix_to_quaternion(self.mat_h_orient)
            self.J_h = self.robot.kinematics.jacobian(self.q, self.x_h)
            self.Kp = np.diag(self.kp)
            self.Kd = np.diag(self.kd)
            


        self.publish_goal = rospy.Publisher('/haptic_mpc/goal_pose', PoseStamped, latch=True)

        #rospy.Subscriber('/haptic_mpc/robot_state', RobotHapticState,
                          #self.populate_robot_state_cb)


        if robot_type == "sim":
            self.planar = True
        else:
            self.planar = False

        self.rate = rospy.Rate(100)

        # rospy.loginfo("waiting for q ...")
        # while self.q == None:
        #     self.rate.sleep()
        # rospy.loginfo("got q.")


        ############all of this should come from the param server, but not yet#########
        self.stopping_dist_to_goal = 0.01
        self.stopping_ang_to_goal = 0.08

        self.f_thresh = 5.
        m1 = 11.34/4.
        m2 = 2.3
        m3 = 1.32

        r1 = [-0.098, 0., 0.]
        r2 = [-0.167, 0., 0.]
        r3 = [-0.144, 0., 0.]

        # %        Ixx     Iyy      Izz    Ixy     Iyz     Ixz
        I1 = [0.00031303125,   0.011214214375,   0.011214214375,    0.,   0.,   0.]
        I2 = [0.00025582627118644067,   0.024175454849340867,   0.024175454849340867,    0.,   0.,   0.]
        I3 = [0.00014657142857142855, 0.010524754285714285, 0.010524754285714285, 0., 0., 0.]

        L1 = I_to_matrix(I1)-m1*skew(r1)*skew(r1)
        L2 = I_to_matrix(I2)-m2*skew(r2)*skew(r2)
        L3 = I_to_matrix(I3)-m3*skew(r3)*skew(r3)

        self.params = [L1[0,0], -L1[0,1], -L1[0,2], L1[1,1], -L1[1,2], L1[2,2], r1[0]*m1, r1[1]*m1, r1[2]*m1, m1,
                       L2[0,0], -L2[0,1], -L2[0,2], L2[1,1], -L2[1,2], L2[2,2], r2[0]*m2, r2[1]*m2, r2[2]*m2, m2,
                       L3[0,0], -L3[0,1], -L3[0,2], L3[1,1], -L3[1,2], L3[2,2], r3[0]*m3, r3[1]*m3, r3[2]*m3, m3]

        self.delta_t = 0.01
        self.alpha = 2.

        self.vel_g = 0.05
        
        #delta_pos_g  = epcon.goal_motion_for_hand_advait(x_h, x_g, dist_g)
        #print "dist_goal is :", dist_goal
        #delta_pos_g = (x_g-x_h)/np.linalg.norm(x_g-x_h)*0.001
        self.dist_g = self.vel_g*self.delta_t  #velocity times time step

        self.qd_max = [0.4]*len(self.q)

        if robot_type == "sim":
            self.q_min = np.radians(np.array([-150., -63., 0.]))
            self.q_max = np.radians(np.array([150., 162., 159.]))

            # Q = np.matrix(np.diag([position_weight,
            #                        position_weight]))
        else:
            rospy.loginfo("didn't ask for sim, so dying, robot isn't implemented")

        # this could all be robot general, but it is now pr2 specific,
        # put it on the param server.
        #else:
            # Q = np.matrix(np.diag([position_weight,
            #                        position_weight,
            #                        position_weight,
            #                        orient_weight,
            #                        orient_weight,
            #                        orient_weight]))


            # self.q_min = np.radians(np.array([-26., -24., -41., -132, -270., -120., -180.]))
            # self.q_max = np.radians(np.array([109., 68., 220., 0.01, 270., 0.01, 180.]))





        ############all of this should come from the param server, but not yet#########





        #########NEEDS UPDATING#####################
        if use_orientation:
            self.orient_weight = 4.
        else:
            self.orient_weight = 0.
        #########NEEDS UPDATING#####################

        ################################################################
        #maybe I should define a max force increase in addition to max
        #overall force
        ################################################################

        # This is the weight on the importance of acheiving the
        # goal orientation.
        position_weight = 5.

        if True:  # opt.sim:
            rospy.loginfo('getting sim goal from param server')
            print "this should eventually be from the trajectory manager"
            while rospy.get_param('/m3/software_testbed/goal') == False:
                rospy.sleep(0.01)
            self.x_g = np.matrix([rospy.get_param('/m3/software_testbed/goal')]).reshape(3,1) 
            print "GOAL is :", self.x_g
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = '/torso_lift_link'
            goal.pose.position.x = self.x_g[0,0]
            goal.pose.position.y = self.x_g[1,0] 
            goal.pose.position.z = self.x_g[2,0]
            goal.pose.orientation.w = 1.
            self.publish_goal.publish(goal)

        else:
            print "this should eventually be from the trajectory manager"
            self.x_g = np.matrix([[0.7, 0.0, 0.0]]).reshape(3,1) 

        
        # this is the goal orientation input as a matrix
        mat_g_orient = np.matrix(np.eye(3))

        if mat_g_orient == None or self.orient_weight == 0:
            self.orient_weight = 0.
            self.q_g_orient = tr.matrix_to_quaternion(np.matrix(np.eye(3))) 
        else:
            self.q_g_orient = tr.matrix_to_quaternion(mat_g_orient) 

        #printing options
        np.set_printoptions(precision=5)
        np.set_printoptions(suppress=True)

    def calc_com_jacobian(self, q, com, joint_ind):
        J_com = self.robot.kinematics.jacobian(q, com)
        J_com[:,joint_ind] = 0.
        return J_com

    def get_data_local(self):
        self.q = self.robot.get_joint_angles()
        self.kp, self.kd = self.robot.get_joint_impedance() 
        self.qd = self.robot.get_joint_velocities()
        self.jep = self.robot.get_ep()
        self.x_h, self.mat_h_orient = self.robot.kinematics.FK(self.q, len(self.q))
        self.q_h_orient = tr.matrix_to_quaternion(self.mat_h_orient)
        self.J_h = self.robot.kinematics.jacobian(self.q, self.x_h)

    def populate_dyn_mpc_msg(self, delta_x_g, J_h, q, q_dot, q_des_cur, Kp, Kd, 
                             Mass, Coriolis, q_min, q_max, qd_max, 
                             values_n, values_l, delta_force_max, n_l, Jc_l, Kc_l):

        print "populate_time :", time.time() - self.time_populate_msg
        self.time_populate_msg = time.time()

        with self.lock:

            num_joints = len(q)
            inv_mass = np.linalg.inv(Mass)

            ######################################################################
            # THIS IS REALLY BAD, need to change so that do one of following:
            #
            # *1)give the christoffel components from M (can do this with partial deriv.)
            # 2)need to linearize expression for C or 
            # 3)use as an input instead. 
            # However, if I linearize this, I'll have to do it for M and others too.
            # * - this is the best option I think - marc Jan 2013
            ######################################################################

            cor_ls = []
            for jj in xrange(num_joints):
                if np.linalg.norm(q_dot[jj]) > 0.05:
                    cor_ls.append(Coriolis[jj,0]/q_dot[jj])
                else:
                    cor_ls.append(0)
            cor_mat = np.diag(cor_ls)

            # these are the continuous time state space equations at the current time step
            A = np.matrix(np.vstack((np.hstack((inv_mass*(-Kd-cor_mat), -inv_mass*Kp)),
                          np.hstack((np.eye(num_joints), np.zeros((num_joints, num_joints)))))))
            
            B = np.matrix(np.vstack((
                          np.hstack((inv_mass, inv_mass*Kp)),
                          np.zeros((num_joints, 2*num_joints)))))

            # following theory in Ogata's Digital controls book and example
            # in Brogan's Modern Control Theory we can discretize the above equations.
            I = np.eye(2*num_joints)
            psy = I
            N = 20
            for ii in xrange(N-1):
               psy = I + A*self.delta_t/(N-ii)*psy

            A_d = I + A*self.delta_t*psy
            B_d = self.delta_t*psy*B

            A_tl = A_d[0:num_joints, 0:num_joints]
            A_tr = A_d[0:num_joints, num_joints:num_joints*2]
            A_bl = A_d[num_joints:num_joints*2, 0:num_joints]
            A_br = A_d[num_joints:num_joints*2, num_joints:num_joints*2]

            B_tl = B_d[0:num_joints, 0:num_joints]
            B_tr = B_d[0:num_joints, num_joints:num_joints*2]
            B_bl = B_d[num_joints:num_joints*2, 0:num_joints]
            B_br = B_d[num_joints:num_joints*2, num_joints:num_joints*2]


            self.out_msg.header.stamp = rospy.get_rostime()
            self.out_msg.header.frame_id = '/torso_lift_link'

            # these gains for the multi-objective cost function were hand tuned
            # but seem to give reasonable results, this will need to be tuned
            self.out_msg.alpha = self.alpha
            self.out_msg.A_tl = A_tl.A1.flatten('F').tolist()
            self.out_msg.A_tr = A_tr.A1.flatten('F').tolist()
            self.out_msg.A_bl = A_bl.A1.flatten('F').tolist()
            self.out_msg.A_br = A_br.A1.flatten('F').tolist()
            self.out_msg.B_tl = B_tl.A1.flatten('F').tolist()
            self.out_msg.B_tr = B_tr.A1.flatten('F').tolist()
            self.out_msg.B_bl = B_bl.A1.flatten('F').tolist()
            self.out_msg.B_br = B_br.A1.flatten('F').tolist()


            #self.out_msg.delta_t = self.delta_t
            # self.out_msg.constr_weight = 1.0
            # self.out_msg.force_weight = 0

            # #these two are currently not assigned in cvxgen, but are at least place holders
            # self.out_msg.beta = 0 
            # self.out_msg.gamma = 0 


            # change in goal position for end effector
            self.out_msg.delta_x_d = (delta_x_g[0:2]).A1.tolist()

                #jacobian of end effector
                #self.out_msg.J = J_h[0:2,:].flatten('F').A1.tolist()
            self.out_msg.J = J_h.flatten('F').A1.tolist()

            #initial config of unknown state variables
            self.out_msg.q_0 = list(q)
            self.out_msg.qd_0 = list(q_dot)
            self.out_msg.q_des_cur_0 = list(q_des_cur)

            self.out_msg.Kp = Kp.flatten('F').tolist()
            self.out_msg.Kd = Kd.flatten('F').tolist()
            #self.out_msg.M = Mass.A1.flatten('F').tolist()

            # cor_mat = np.diag([Coriolis[0,0]/q_dot[0], 
            #                    Coriolis[1,0]/q_dot[1], 
            #                    Coriolis[2,0]/q_dot[2]])

            # self.out_msg.C = cor_mat.flatten('F').tolist()

            #limits on predicted joint angles (state)
            self.out_msg.q_min = q_min.tolist()
            self.out_msg.q_max = q_max.tolist()

            #limits on predicted joint velocities (state)
            self.out_msg.qd_max = qd_max

            self.out_msg.u_max = [ 0.0154]*len(q_max)

            ###need to make this a param on param server
            num_poss_contacts = 40

            # filling vectors for allowable change in force at each contact
            if np.where(values_n.A1 > 1.0)[0].tolist() == []:
                delta_force_max = np.matrix(np.zeros((num_poss_contacts, 1)))
            else:  #CHECK THOROUGHLY!!!!!!!
                delta_force_max = delta_force_max[np.where(values_n.A1 < self.f_thresh)[0].tolist()]
                delta_force_max = np.vstack((delta_force_max, np.zeros((num_poss_contacts-delta_force_max.shape[0], 1))))

            self.out_msg.delta_f_max = delta_force_max.A1.tolist()

            mass_n_J_com = []

            #WHERE DO I GET LINK MASS and COM???
            #NEED TO DEFINE THE FUNCTION calc_com_jacobian
            masses = [11.34/4.0, 2.3, 1.32]

            for kk in xrange(num_joints):
                if q_dot[kk] < 0.05:
                    mass_n_J_com.append(np.zeros(num_joints).tolist())
                else:
                    #this is a short-term hack for com, will need better solution
                    # should just be getting the offsets in link frame, rotating and 
                    # adding it to pt1 ...
                    pt1, _ = self.robot.kinematics.FK(q, kk)
                    pt2, _ = self.robot.kinematics.FK(q, kk+1)
                    com = pt1 + (pt2-pt1)*0.5
                    J_com = self.calc_com_jacobian(q, com, kk)
                    vel_norm = J_com*q_dot/np.linalg.norm(J_com*q_dot)
                    mass_n_J_com.append((masses[kk]*vel_norm.T*J_com).tolist())

            self.out_msg.mass_n_J_com = np.array(mass_n_J_com).flatten('F').tolist()

            max_impulse = (self.f_thresh*0.07*np.ones(num_joints)).tolist()
            self.out_msg.f_max_delta_t = max_impulse

            # filling the matrices for estimating change in force from predicted change in joint angles
            # as well as the desired decrease in contact force when forces are above threshold
            n_K_J_all_buff = []

            tau_cont_sum_0 = np.matrix(np.zeros(num_joints)).reshape(num_joints, 1)
            all_J_T_K_J = np.matrix(np.zeros((num_joints, num_joints)))

            #this calculation can be made into matrix multiplication 
            # to increase efficiency outside of for loop if it is bottleneck
            for ii in xrange(len(Jc_l)):
                tau_cont_sum_0 = tau_cont_sum_0 + Jc_l[ii].T*values_l[ii]
                all_J_T_K_J = all_J_T_K_J + Jc_l[ii].T*Kc_l[ii]*Jc_l[ii]
                if values_n[ii, 0] >= 1.0:
                    buff_list = (n_l[ii].T*Kc_l[ii]*Jc_l[ii]).A1.tolist()
                    n_K_J_all_buff.append(buff_list)

                # elif values_n[ii,0] < min_dist_mag:
                #         #des_force_decrease.append(max_force_mag - f_n[ii,0])
                #     buff_list = (n_l[ii].T*Jc_l[ii]).A1.tolist()
                #     n_J_ci_max_buff.append(buff_list)

            self.out_msg.tau_cont_sum_0 = tau_cont_sum_0.A1.tolist()

            self.out_msg.all_J_T_K_J = all_J_T_K_J.A1.flatten('F').tolist()

            # this is a check for any contact at all before populating matrices
            if n_K_J_all_buff == []:
                n_K_J_all = np.zeros((num_poss_contacts, len(q)))
            else:
                n_K_J_all = np.array(n_K_J_all_buff)
                n_K_J_all = np.vstack((n_K_J_all, np.zeros((num_poss_contacts-n_K_J_all.shape[0], len(q)))))

            self.out_msg.n_K_J_all = n_K_J_all.flatten('F').tolist()

            return self.out_msg



    def delta_force_bounds(self, values_n, f_thresh):

        # Compute how much the contact force can change before it hits
        # the maximum, and limit the expected increase in force by
        # this quantity.
        n = values_n.shape[0]  
        force_max = f_thresh * np.matrix(np.ones((n,1))) 

        # if this is negative, my constraint currently says that
        # change of force has to be less than that.  Is that a bad
        # thing??? - marc
        delta_force_max = values_n - force_max

        # If a force has exceeded the maximum use special constraints.
        over_max = values_n > f_thresh
        if over_max.any():
            # at least one of the contact forces is over the maximum
            # allowed
            delta_force_max[over_max] = 0.

        return delta_force_max


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

    def populate_robot_state_cb(self, msg):
        print "robot state update :", time.time() - self.time_haptic_state
        self.time_haptic_state = time.time()

        with self.lock:
            self.updated_state = True
            #print msg
            self.x_h = np.matrix([msg.hand_pose.position.x, 
                                  msg.hand_pose.position.y, 
                                  msg.hand_pose.position.z]).reshape(3,1)
            self.q_h_orient = [msg.hand_pose.orientation.x, 
                               msg.hand_pose.orientation.y, 
                               msg.hand_pose.orientation.z, 
                               msg.hand_pose.orientation.w] 
            self.q = msg.joint_angles
            self.q_dot = msg.joint_velocities
            self.jep = msg.desired_joint_angles
            self.kp = msg.joint_stiffness
            self.Kp = np.diag(self.kp)
            self.kd = msg.joint_damping
            self.Kd = np.diag(self.kd)
            self.time_stamp_msg = msg.header.stamp.to_time()
            self.values_l_msg = mpc_util.getValues(msg.skins, force=True, distance=True)
            self.n_l_msg = mpc_util.getNormals(msg.skins)
            self.J_h = self.ma_to_m.multiArrayToMatrixList(msg.end_effector_jacobian)[0]
            self.Jc_l = self.ma_to_m.multiArrayToMatrixList(msg.contact_jacobians)
        
    def update_data(self):
        stop = ''
        
        with self.lock:
            self.get_data_local()
            self.updated_state = False
            #msg_state = self.robot_state.getHapticStateMessage()
            msg_state = self.robot_state.updateHapticState()
            # self.populate_robot_state_cb(msg_state)

            # if False:  #use ignore skin option here!
            #     values_l, n_l= [], []
            #     time_stamp = 0.
            #     self.Jc_l = []
            # else:
            #     values_l, n_l, time_stamp  =  copy.copy(self.values_l_msg), copy.copy(self.n_l_msg), copy.copy(self.time_stamp_msg)
            


            # values_n = np.matrix([(n_i.T*values_i)[0,0] for n_i, values_i in it.izip(n_l, values_l)]).T
            # Jc_l_send = copy.copy(self.Jc_l)

            # if self.robot_type == "sim":
            #     J_h = copy.copy(self.J_h[0:2])
            # else:
            #     rospy.loginfo("only sim is implemented")
            #     assert(False)
            #     # J_h = copy.copy(self.J_h)
            #     # T_quat = 0.5 * (q_h_orient[3] 
            #     #                 * np.matrix(np.eye(3)) 
            #     #                 - mpc_util.get_skew_matrix(q_h_orient[0:3]))
            #     # J_h[3:] = T_quat*self.J_h[3:]

            #     # #print "delta_x_g is :", delta_x_g

            #     # J_h = J_h[:,0:Kp.shape[0]] # comes into play with Cody and the wrist cover

            # #Q = np.matrix(goal_state_weights)


            # # if close enough to goal declare success!
            # dist_goal_2D = np.linalg.norm((self.x_g - self.x_h)[0:2])
            # dist_goal_3D = np.linalg.norm(self.x_g - self.x_h)

            # #print "dist to goal is :", dist_goal_3D

            # if self.planar:
            #     dist_goal = dist_goal_2D
            # else:
            #     dist_goal = dist_goal_3D

            # angle_error = ut.quat_angle(self.q_h_orient, self.q_g_orient)

            # if self.orient_weight != 0:
            #     proportional_ball_radius = 0.1
            #     proportional_ball_dist_slope = 1.
            #     if dist_goal < proportional_ball_radius:
            #         position_weight = position_weight * dist_goal/ (proportional_ball_radius * proportional_ball_dist_slope)

            #     proportional_ball_angle = math.radians(30)
            #     proportional_ball_angle_slope = 1.
            #     if angle_error < proportional_ball_angle:
            #         self.orient_weight = self.orient_weight * angle_error/ (proportional_ball_angle * proportional_ball_angle_slope)

            #     if dist_goal < self.stopping_dist_to_goal and angle_error < self.stopping_ang_to_goal:
            #         stop = 'Reached'
            #         print 'Reached'
            #     else:
            #         stop = ''

            # else:
            #     # different stopping condition if ignoring orientation.
            #     if dist_goal < self.stopping_dist_to_goal:
            #         stop = 'Reached'
            #         self.end_trial = True
            #     else:
            #         stop = ''

            # Kc_l = self.get_contact_stiffnesses(n_l, 1000)


            # # n = the number of contacts
            # n = len(n_l)

            # # f_mag_list is a list of the magnitudes of the contact
            # # forces
            # values_mag_list  = [np.linalg.norm(val_vec) for val_vec in values_l]


            # #print "values_mag_list is :", values_mag_list

            # ######WE COULD MAYBE USE JOINT TORQUES WITHOUT THE SKIN TO ESTIMATE A STOPPING CONDITION################
            # # try:
            # #     if values_mag_list != [] and ep_gen.kill_controller_force > 0 and max(values_mag_list) < 0.001:
            # #         stop = 'small distance %f'%(max(f_mag_list))
            # #         raise EqPtGen(stop, ())
            # # except AttributeError:
            # #     # probably because of ep_gen.kill_controller_force not
            # #     # existing. This is a temporary thing.
            # #     pass
            # ######WE COULD MAYBE USE JOINT TORQUES WITHOUT THE SKIN TO ESTIMATE A STOPPING CONDITION################

            # # # calculate the normal components of the current contact
            # # # forces
            # # tmp_l = [R_ci * f_ci for R_ci, f_ci in it.izip(Rc_l, f_l)]
            # # f_n = np.matrix([tmp_i[0,0] for tmp_i in tmp_l]).T

            # # this is a hack for now, need a good fit function to map adc values to some sort of distance, should probably be conservative.

            # delta_force_max = self.delta_force_bounds(values_n, self.f_thresh)


            # err = self.x_g-self.x_h
            # err_mag = np.linalg.norm(err)
            # # scale = 30
            # # if err_mag > self.dist_g * scale:
            # #     delta_pos_g = self.dist_g * (err/err_mag)
            # # else:
            # #     delta_pos_g = err/scale
            # delta_pos_g = err


            # if self.planar:
            #     delta_x_g = delta_pos_g[0:2]
            # else:
            #     # delta_orient_g = mpc_util.goal_orientation_in_quat(q_h_orient, q_g_orient, math.radians(0.1))
            #     # delta_x_g = np.vstack((delta_pos_g, delta_orient_g))
            #     print "this is for sim"
            #     assert (False)

            # Mass = np.matrix(M(self.params, self.q)).reshape(3,3)
            # self.Mass = Mass

            # Coriolis = np.matrix(c(self.params, self.q, self.q_dot)).reshape(3,1)
            # self.Coriolis = Coriolis

            # msg = self.populate_dyn_mpc_msg(delta_x_g, 
            #                                 J_h, 
            #                                 self.q, 
            #                                 self.q_dot, 
            #                                 self.jep, #q_des_cur
            #                                 self.Kp, 
            #                                 self.Kd, 
            #                                 Mass, 
            #                                 Coriolis, 
            #                                 self.q_min, 
            #                                 self.q_max,
            #                                 self.qd_max, 
            #                                 values_n,
            #                                 values_l,
            #                                 delta_force_max,
            #                                 n_l,
            #                                 Jc_l_send,
            #                                 Kc_l)
            # #print msg
            
        return stop




if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()
    p2 = optparse.OptionParser()

    p.add_option('--ignore_skin', '--is', action='store_true',
                 dest='ignore_skin', help='ignore feedback from skin', default=False)

    p.add_option('--use_orientation', '--uo', action='store_true', default=False,
                 dest='use_orientation', help='try to go to commanded orientation in addition to position')

    p.add_option('--pr2', action='store_true', dest='pr2',
                 help='mid level for the pr2')
    
    p.add_option('--sim', action='store_true', dest='sim',
                 help='use 3 link planar capsule')

    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)

    opt, args = p.parse_args()

    rospy.init_node('test_dynamics_controller')

    gen_data = CvxGenData('sim', False)

    mpc_setup_pub = rospy.Publisher('/formatted_mpc_data', MpcDynFormattedData)
    # req = HapticMPCLogging()
    # req.log_start = "start"
    # req.log_dir = "./"
    # req.log_file = "reach_from_start_to_goal_data_log.pkl"

    # rospy.wait_for_service('haptic_mpc_logger_state')
    # try:
    #     start_logging = rospy.ServiceProxy('haptic_mpc_logger_state', HapticMPCLogging)
    #     resp = start_logging(req)
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s"%e
    #if True:

    gen_data.end_trial = False

    while gen_data.end_trial == False:
         gen_data.rate.sleep()
         gen_data.update_data()
         mpc_setup_pub.publish(gen_data.out_msg)





        #rospy.sleep(1.)

        #if stop == '':
            #print "got here"
            #print msg

        

        # try: # This needs to be added to a log and monitor node or to the robot haptic state
        #     if robot.kinematics.arm_type != 'simulated':
        #         try:
        #             if np.any(np.array(robot.get_joint_motor_temps()) >= 100):
        #                 stop = 'high motor temp'   
        #                 rospy.logerr('motor temps are high ... '+str(robot.get_joint_motor_temps()))
        #                 raise EqPtGen(stop, ())
        #         except:
        #             #this is a hack right now, should update so that perhaps it performs different
        #             #list of function checks on each robot (since they are different)
        #             pass
 


        # while not rospy.is_shutdown():
        #     mpc_setup_pub.publish(msg)
        #     rate.sleep()

    # req.log_start = "stop"
    # try:
    #     resp = start_logging(req)
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s"%e
