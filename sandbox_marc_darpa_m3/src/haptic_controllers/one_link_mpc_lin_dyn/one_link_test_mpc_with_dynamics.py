#!/usr/bin/env python  

import sys, os
import numpy as np, math
import scipy.io
import copy
from threading import RLock, Timer
#from multiprocessing import Lock
import time
from math import pi
from utils import *

import roslib; 
#roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('sandbox_marc_darpa_m3')
import rospy
import itertools as it

import hrl_lib.viz as hv
import hrl_lib.transforms as tr
import hrl_lib.util as ut
import hrl_lib.circular_buffer as cb
import hrl_haptic_mpc.haptic_mpc_util as mpc_util
import hrl_haptic_mpc.multiarray_to_matrix as multiarray_to_matrix
from hrl_software_simulation_darpa_m3.ode_sim_guarded_move import ode_SkinClient


from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Quaternion, PointStamped
from hrl_haptic_manipulation_in_clutter_msgs.msg import MpcDynFormattedData, RobotHapticState, HapticMpcState
from hrl_haptic_manipulation_in_clutter_srvs.srv import HapticMPCLogging, GetJep, GetJepResponse, MpcFormatted, MpcFormattedRequest
from hrl_haptic_mpc.robot_haptic_state_node import RobotHapticStateServer
from onelink_dynamics import *

num_poss_contacts = 10

class CvxGenData():
    def __init__(self, robot_type, use_orientation=False):

        fake_args = ["-r", "sim3", "-s", "none", "-a", "l"]
        mpc_util.initialiseOptParser(p2)
        (opt, args) = p2.parse_args(fake_args)
        # self.robot_state = RobotHapticStateServer(opt)
        # self.robot_state.updateHapticState()
        # print self.robot_state.joint_angles
        # assert(False)
        skin_topic_list = ['/skin/contacts']        
        self.goal_marker_pub = rospy.Publisher('/epc_skin/viz/goal', Marker)
        self.nom_stiffness = 48000. #48000.
        self.q_s = []
        self.qd_s = []
        self.max_forces = []
        self.times =[]
        self.scl = ode_SkinClient(skin_topic_list)
        self.cur_forces = []
        self.end_trial = False
        self.updated_state = False
        self.time_haptic_state = 0.
        self.time_populate_msg = 0.
        self.lock = RLock()
        #self.lock = Lock()
        self.J_h = None  # use Jeff's function to get msg.end_effector_jacobian in right format
        self.max_time_to_fill = 0
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
        self.time_update = 0.
        self.jep = None #desired_joint_angles: []
        self.x_h = None # hand_pose.position
        self.q_h_orient = None # hand_pose.orientation
        self.ma_to_m = multiarray_to_matrix.MultiArrayConverter() 
        self.out_msg = MpcDynFormattedData() 
        #self.out_msg = None
        self.robot_type = robot_type 

        if True:
            import hrl_software_simulation_darpa_m3.gen_sim_arms as sim_robot
            import hrl_common_code_darpa_m3.robot_config.one_link_planar_capsule as sim_robot_config
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
            self.x_g = copy.copy(self.x_h)
            self.q_h_orient = tr.matrix_to_quaternion(self.mat_h_orient)
            self.J_h = self.robot.kinematics.jacobian(self.q, self.x_h)
            self.Kp = np.diag(self.kp)
            self.Kd = np.diag(self.kd)
            
        self.hist_size = 20000 

        self.ee_pos_buf = cb.CircularBuffer(self.hist_size, (3,)) 
        self.max_force_buf = cb.CircularBuffer(self.hist_size, ()) 

        # magnitude of all contact forces.
        self.all_forces_buf = cb.CircularBuffer(self.hist_size, ())

        # locations of the contact forces
        self.all_forces_locs_buf = cb.CircularBuffer(self.hist_size, (3,))

        # normals of the contact forces
        self.all_forces_nrmls_buf = cb.CircularBuffer(self.hist_size, (3,))

        # jt num (as returned by the skin client).
        self.all_forces_jts_buf = cb.CircularBuffer(self.hist_size, ())

        # use this buffer to figure out mapping between joint angles
        # and all the contact forces corresponding to that arm
        # configuration
        self.num_contacts_at_time_instant_buf = cb.CircularBuffer(self.hist_size, ())

        self.n_jts = len(self.q) # TODO: Only 3 for sim. Need 7 for PR2.
        self.jep_buf = cb.CircularBuffer(self.hist_size, (self.n_jts,))
        self.q_buf = cb.CircularBuffer(self.hist_size, (self.n_jts,))
        self.qdot_buf = cb.CircularBuffer(self.hist_size, (self.n_jts,))

        self.time_stamp_buf = cb.CircularBuffer(self.hist_size, ())

        self.ee_gaussian_mn_buf_50 = cb.CircularBuffer(self.hist_size, (3,))
        self.ee_gaussian_cov_buf_50 = cb.CircularBuffer(self.hist_size, (3,3))
        self.mean_motion_buf = cb.CircularBuffer(self.hist_size, ())

        self.publish_goal = rospy.Publisher('/haptic_mpc/goal_pose', PoseStamped, latch=True)

        self.mpc_setup_pub = rospy.Publisher('/formatted_mpc_data', MpcDynFormattedData)

        #rospy.Subscriber('/haptic_mpc/robot_state', RobotHapticState,
                          #self.populate_robot_state_cb)


        if robot_type == "sim":
            self.planar = True
        else:
            self.planar = False

        #self.delta_t = 0.01
        self.delta_t = 0.01
        self.rate = rospy.Rate(1./self.delta_t)

        # rospy.loginfo("waiting for q ...")
        # while self.q == None:
        #     self.rate.sleep()
        # rospy.loginfo("got q.")


        ############all of this should come from the param server, but not yet#########
        self.stopping_dist_to_goal = 0.001
        self.stopping_ang_to_goal = 0.08

        self.f_thresh = 5.
        m1 = 0.416

        r1 = [0.078-0.13545, 0., 0.]

        # %        Ixx     Iyy      Izz    Ixy     Iyz     Ixz
        I1 = [0.0,   0.0,   0.00287,    0.,   0.,   0.]

        L1 = I_to_matrix(I1)-m1*skew(r1)*skew(r1)

        self.params = [L1[0,0], -L1[0,1], -L1[0,2], L1[1,1], -L1[1,2], L1[2,2], r1[0]*m1, r1[1]*m1, r1[2]*m1, m1]
                       

        # self.alpha = 10.
        # #self.beta = 10000000. #1. #1.
        # self.beta = 100000. #1. #1.
        # self.gamma = 1000. #1000000000000000000. #1000.
        # self.zeta = 0. #0.001 #1.
        # self.mu = 1000 #1000000.

        # new controller form
        self.alpha = 1.0 #0.1
        self.beta = 10000000. #100000000  value gives error = 4.0 for 3N over #*self.nom_stiffness #10000000000. #10000. #1. #1.
        #self.gamma = 10. #1000000000000000000. #1000.
        self.zeta = 10000000. #0.3 #0.01 #0.1 #10. #10 #0.001 #1.
        self.mu = 0.0001 #00003 #00002 #0.00003 #1000000.

        self.vel_g = 0.05
        
        #delta_pos_g  = epcon.goal_motion_for_hand_advait(x_h, x_g, dist_g)
        #print "dist_goal is :", dist_goal
        #delta_pos_g = (x_g-x_h)/np.linalg.norm(x_g-x_h)*0.001
        self.dist_g = self.vel_g*self.delta_t  #velocity times time step

        self.qd_max = [0.3]*self.n_jts  #[0.01]*3
        #self.qd_max = [0.1]*len(self.q)

        if robot_type == "sim":
            self.q_min = np.radians(np.array([-180.]))
            self.q_max = np.radians(np.array([180.]))

            # self.q_min = np.radians(np.array([-361., -361.,-361.]))
            # self.q_max = np.radians(np.array([361., 361., 361.]))

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
        np.set_printoptions(precision=9)
        np.set_printoptions(suppress=True)

        s = rospy.Service('new_data_ready', GetJep, self.waiting_for_data_from_sim)
        self.send_to_cvxgen = rospy.ServiceProxy("solve_qp_problem", MpcFormatted)


    def saveDataToDisk(self, success, final_state):
        log_dict = self.createLogDict("fast_mpc")
        ut.save_pickle(log_dict, "./reach_log.pkl")
        scipy.io.savemat("./reach_log.mat", log_dict)

        overall_dict = {}
        overall_dict['goal'] = self.x_g
        overall_dict['reached_goal'] = success
        overall_dict['final_state'] = final_state
        print 'FINAL RESULT:', success
        ut.save_pickle(overall_dict, 'overall_result.pkl')
        scipy.io.savemat("./overall_result.mat", overall_dict)


    # Pass in a list of SIM link names - this returns a corresponding list of joint indices
    # NB: SIM ONLY. PR2/Cody will differ
    def convertLinkNamesToJointNums(self, links_list):
        joints_list = []
        for link in links_list:
            if link == "link3":
                joints_list.append(2)
            elif link=="link2":
                joints_list.append(1)
            elif link=="link1":
                joints_list.append(0)
        return joints_list

    # Called by the main logger loop to push current data to the circular buffers.
    def logDataToBuffers(self):
        # Copy current state while locked.
        jep = copy.copy(self.desired_joint_angles)
        q = copy.copy(self.joint_angles)
        q_dot = copy.copy(self.joint_velocities)
        ee = copy.copy(self.end_effector_pos)
      
        # Log force info
        max_force = 0.0 
        if values_list:
            mat = np.column_stack(values_list)
            f_mag_l = ut.norm(mat).A1.tolist()
      
        for f_mag, nrml, loc, jt in zip(f_mag_l, normals_list, locations_list, joints_list):
            self.all_forces_buf.append(f_mag)
            self.all_forces_locs_buf.append(loc.A1)
            self.all_forces_nrmls_buf.append(nrml.A1)
            self.all_forces_jts_buf.append(jt)
      
        max_force = np.max(f_mag_l)
      
        self.max_force_buf.append(max_force)
        self.num_contacts_at_time_instant_buf.append(len(values_list))

        # Log robot state
        self.jep_buf.append(jep)
        self.q_buf.append(q)
        self.qdot_buf.append(q_dot)
        self.ee_pos_buf.append(ee.A1) # end effector position - not orientation

        # fit Gaussian to last 50 locations of the end effector.
        self.updateEeGaussianBuf(self.ee_gaussian_mn_buf_50,
                                 self.ee_gaussian_cov_buf_50, 50)
        # find distance (in 2D) between the means of the Gaussians
        # that are 200 samples apart.
        self.updateMeanMotion(self.ee_gaussian_mn_buf_50, step=200)
      
        self.time_stamp_buf.append(rospy.get_time())

    def is_ee_moving(self, distance_thresh):
        if len(self.mean_motion_buf) > 0 and \
           self.mean_motion_buf[-1] < distance_thresh:
            n = min(len(self.mean_motion_buf), 5)
            #rospy.loginfo('Mean is not moving anymore: %s'%(str(self.mean_motion_buf.get_last(n))))
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

  # Called when data should be dumped to disk (infrequent)
    def createLogDict(self, controller_type):
        d = {}

        d['max_force_list'] = self.max_force_buf.to_list()
        d['all_forces_list'] = self.all_forces_buf.to_list()
        d['all_forces_locs_list'] = self.all_forces_locs_buf.to_list()
        d['all_forces_nrmls_list'] = self.all_forces_nrmls_buf.to_list()
        d['all_forces_jts_list'] = self.all_forces_jts_buf.to_list()
        d['num_contacts_at_time_instant_list'] = self.num_contacts_at_time_instant_buf.to_list()
        d['ee_pos_list'] = self.ee_pos_buf.to_list()
        d['jep_list'] = self.jep_buf.to_list()
        d['q_list'] = self.q_buf.to_list()
        d['qdot_list'] = self.qdot_buf.to_list()
        d['ee_gaussian_50_mn_list'] = self.ee_gaussian_mn_buf_50.to_list()
        d['ee_gaussian_50_cov_list'] = self.ee_gaussian_cov_buf_50.to_list()
        d['mean_motion_list'] = self.mean_motion_buf.to_list()
        d['time_stamp_list'] = self.time_stamp_buf.to_list() 
        d['controller_type'] = controller_type

        return d

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
                             values_n, values_l, delta_force_max, delta_rate_force_max,
                             n_l, Jc_l, Kc_l):

        # print "populate_time :", time.time() - self.time_populate_msg
        # self.time_populate_msg = time.time()
        matlab_dict = {}



        if True:
        #with self.lock:

            start_fill = time.time()
            num_joints = self.n_jts

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
                if np.linalg.norm(q_dot[jj]) > 0.0001:
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
            N = 300
            for ii in xrange(N-1):
               psy = I + A*self.delta_t/(N-ii)*psy

            A_d = I + A*self.delta_t*psy
            B_d = self.delta_t*psy*B

            # print "A_d is \n", A_d
            # print "B_d is \n", B_d
            # print "mass is \n", Mass

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
            self.out_msg.beta = self.beta
            #self.out_msg.gamma = self.gamma
            self.out_msg.zeta = self.zeta
            self.out_msg.mu = self.mu
            self.out_msg.A_tl = A_tl.flatten('F').A1.tolist()
            self.out_msg.A_tr = A_tr.flatten('F').A1.tolist()
            self.out_msg.A_bl = A_bl.flatten('F').A1.tolist()
            self.out_msg.A_br = A_br.flatten('F').A1.tolist()
            self.out_msg.B_tl = B_tl.flatten('F').A1.tolist()
            self.out_msg.B_tr = B_tr.flatten('F').A1.tolist()
            self.out_msg.B_bl = B_bl.flatten('F').A1.tolist()
            self.out_msg.B_br = B_br.flatten('F').A1.tolist()

            # change in goal position for end effector
            self.out_msg.delta_x_d = (delta_x_g[0:2]).A1.tolist()
            goal_cross_mat = np.matrix([[0., 0., delta_x_g[1]],
                                        [0., 0., -delta_x_g[0]],
                                        [-delta_x_g[1], delta_x_g[0], 0.]])

            # goal_cross_mat = np.matrix([[0, -delta_x_g[2], delta_x_g[1]],
            #                             [delta_x_g[2], 0, -delta_x_g[0]],
            #                             [-delta_x_g[1], delta_x_g[0], 0]])

            J_h_augmented = np.vstack((J_h, np.zeros((1, num_joints))))
            vel_norm_J = goal_cross_mat*J_h_augmented
            #self.out_msg.vel_norm_J = vel_norm_J.flatten('F').A1.tolist()

            #print "self.out_msg_delta_x_d :", self.out_msg.delta_x_d
                #jacobian of end effector
                #self.out_msg.J = J_h[0:2,:].flatten('F').A1.tolist()
            self.out_msg.J = J_h.flatten('F').A1.tolist()

            #initial config of unknown state variables
            self.out_msg.q_0 = list(q)
            self.out_msg.qd_0 = list(q_dot)
            self.out_msg.q_des_cur_0 = list(q_des_cur)

            # self.out_msg.Kp = Kp.flatten('F').tolist()
            # self.out_msg.Kd = Kd.flatten('F').tolist()

            if False:
                self.out_msg.mass = np.zeros(9).tolist() #Mass.flatten('F').A1.tolist()
            else:
                self.out_msg.mass = Mass.flatten('F').A1.tolist()
            # cor_mat = np.diag([Coriolis[0,0]/q_dot[0], 
            #                    Coriolis[1,0]/q_dot[1], 
            #                    Coriolis[2,0]/q_dot[2]])

            # self.out_msg.C = cor_mat.flatten('F').tolist()


            if False:
                self.out_msg.q_min = [-100]*self.n_jts #q_min.tolist()
                self.out_msg.q_max = [100]*self.n_jts  #q_max.tolist()
            else:
                #limits on predicted joint angles (state)
                self.out_msg.q_min = q_min.tolist()
                self.out_msg.q_max = q_max.tolist()

            # if False:
            #     #limits on predicted joint velocities (state)
            #     self.out_msg.qd_max = [100]*self.n_jts #qd_max
            # else:
            #     #limits on predicted joint velocities (state)
            #     self.out_msg.qd_max = qd_max
                

            if False:
                #self.out_msg.u_max = [0.1]*len(q_max)
                #self.out_msg.u_max = [0.2]*len(q_max)
                u_max = [100]*self.n_jts #[0.2]*len(q_max)
                self.out_msg.u_max = u_max
            else:
                #self.out_msg.u_max = [0.2]*len(q_max)
                #u_max = [0.02]*len(q_max)
                u_max = [0.2]*len(q_max)
                self.out_msg.u_max = u_max

            ###need to make this a param on param server

            if False:
                self.out_msg.delta_f_max = np.zeros(num_poss_contacts).tolist() #delta_force_max.flatten('F').tolist()
            else:
                self.out_msg.delta_f_max = delta_force_max.flatten('F').tolist()
                print "the out_msg version:\n", self.out_msg.delta_f_max

            #self.out_msg.delta_cont_vel_max = (np.ones(num_poss_contacts)*0.001).tolist()

            self.out_msg.delta_rate_f_max = delta_rate_force_max.flatten('F').tolist()

            mass_n_J_com = []

            #WHERE DO I GET LINK MASS and COM???
            #NEED TO DEFINE THE FUNCTION calc_com_jacobian

            #masses = [0., 0., 0.]
            masses = [2.3]

            for kk in xrange(num_joints):
                if q_dot[kk] < 0.0001:
                    mass_n_J_com.append(np.zeros(num_joints).tolist())
                else:
                    #this is a short-term hack for com, will need better solution
                    # should just be getting the offsets in link frame, rotating and 
                    # adding it to pt1 ...
                    pt1, _ = self.robot.kinematics.FK(q, kk)
                    pt2, _ = self.robot.kinematics.FK(q, kk+1)
                    com = pt1 + (pt2-pt1)*0.5
                    J_com = self.calc_com_jacobian(q, com, kk)
                    # print "J_com :\n", J_com
                    # print "q_dot :\n", q_dot
                    q_dot_vec = np.matrix(q_dot).reshape(len(q), 1)
                    vel_norm = J_com[0:2,:]*q_dot_vec/np.linalg.norm(J_com[0:2,:]*q_dot_vec)
                    mass_n_J_com.append((masses[kk]*vel_norm.T*J_com[0:2,:]).A1.tolist())
            
            # #mass_n_J_com = Mass*0.5

            #self.out_msg.mass_n_J_com = np.array(mass_n_J_com).flatten('F').tolist()



            if False:
                max_impulse = (self.f_thresh*0.0*np.ones(num_joints)).tolist()
            else:
                #max_impulse = (self.f_thresh*0.0005*np.ones(num_joints)).tolist()
                max_impulse = (self.f_thresh*0.01*np.ones(num_joints)).tolist()

                #max_impulse = (self.f_thresh*0.07*np.ones(num_joints)).tolist()
                #max_impulse = (J_h.T*self.f_thresh*0.05*np.ones((num_joints-1,1))).A1.tolist()                

                #calc configuration dependent max impulse torque on joints
                max_impulse_test = []
                tau_impulse_ls = []
                delta_t_impulse = 0.04 #0.05 # .07 #0.02-3 worked pretty well (10N or so maximums)cally

                for kk in xrange(len(q)):
                    max_dist = 0
                    max_pos = None
                    for jj in xrange(len(q)+1):
                        if jj > kk:
                            dist = np.linalg.norm(self.jt_pos[jj] - self.jt_pos[kk])
                            if dist > max_dist: 
                                max_dist = dist 
                                max_pos = self.jt_pos[jj] 
                    f_dir = np.cross((max_pos-self.jt_pos[kk]).A1, [0, 0, 1]) 
                    tau_impulse_max = np.linalg.norm(np.cross(-f_dir*self.f_thresh, (self.jt_pos[kk]- max_pos).A1)) 
                    tau_impulse_ls.append(tau_impulse_max)
                    max_impulse_test.append(tau_impulse_max*delta_t_impulse) 

            #max_impulse_test[0] = max_impulse_test[0]
            # print "current_max_impulse \n", max_impulse
            # print "new max_impulse test \n", max_impulse_test

            q_dot_vec = np.matrix(q_dot).reshape(len(q), 1)

            # print "M*qdot :\n", Mass*q_dot_vec

            # print "tau_impulse_max :\n", tau_impulse_ls

            self.out_msg.tau_max_delta_t = max_impulse_test
            #self.out_msg.f_max_delta_t = max_impulse

            # filling the matrices for estimating change in force from predicted change in joint angles
            # as well as the desired decrease in contact force when forces are above threshold
            n_K_J_all_buff = []
            n_J_all_buff = []
            n_K_J_over_buff = []

            tau_cont_sum_0 = np.matrix(np.zeros(num_joints)).reshape(num_joints, 1)
            all_J_T_K_J = np.matrix(np.zeros((num_joints, num_joints)))
            print "Kc_l is :\n", Kc_l

            #this calculation can be made into matrix multiplication 
            #to increase efficiency outside of for loop if it is bottleneck
            for ii in xrange(len(Jc_l)):
                tau_cont_sum_0 = tau_cont_sum_0 - Jc_l[ii].T*values_l[ii] #this is minus due to sign convention
                all_J_T_K_J = all_J_T_K_J + Jc_l[ii].T*Kc_l[ii]*Jc_l[ii]  
                print "for i = ", ii, " all_J_T_K_J is \n:", Jc_l[ii].T*Kc_l[ii]*Jc_l[ii]
                
                # if values_n[ii, 0] >= 0.1:
                #     buff_list = (n_l[ii].T*Kc_l[ii]*Jc_l[ii]).A1.tolist()

                if values_n[ii, 0] > self.f_thresh:
                    #buff_list = (n_l[ii].T*Kc_l[ii]*18*Jc_l[ii]).A1.tolist()
                    #buff_list = (n_l[ii].T*Kc_l[ii]*Jc_l[ii]/self.nom_stiffness).A1.tolist()
                    buff_list = (self.nom_stiffness*n_l[ii].T*Jc_l[ii]/self.nom_stiffness).A1.tolist()
                    print "for ", ii, ":\n n_l is:\n", n_l[ii]
                    print "Jc_l is :\n", Jc_l[ii]

                    #n_K_J_over_buff.append(buff_list)
                    n_K_J_all_buff.append(buff_list)
                    n_J_all_buff.append((n_l[ii].T*Jc_l[ii]).A1.tolist())
                    # print "OVER THRESHOLD...."
                    # print "values_n ", ii, " :\n", values_n[ii,0]
                    # print "normal ", ii, " :\n", n_l[ii]
                    # print "jacobian ", ii, " :\n", Jc_l[ii]
                    # print "K ", ii, " :\n", Kc_l[ii]
                    # print "n_K_J_over_buff :\n", n_K_J_over_buff
                elif values_n[ii, 0] >= 0.01:
                    #buff_list = (n_l[ii].T*Kc_l[ii]*18*Jc_l[ii]).A1.tolist()
                    buff_list = (self.nom_stiffness*n_l[ii].T*Jc_l[ii]/self.nom_stiffness).A1.tolist()
                    n_K_J_all_buff.append(buff_list)
                    n_J_all_buff.append((n_l[ii].T*Jc_l[ii]).A1.tolist())
                    print "for ", ii, ":\n n_l is:\n", n_l[ii]
                    print "Jc_l is :\n", Jc_l[ii]

                    # print "values_n ", ii, " :\n", values_n[ii,0]
                    # print "normal ", ii, " :\n", n_l[ii]
                    # print "jacobian ", ii, " :\n", Jc_l[ii]
                    # print "K ", ii, " :\n", Kc_l[ii]

                # elif values_n[ii,0] < min_dist_mag:
                #         #des_force_decrease.append(max_force_mag - f_n[ii,0])
                #     buff_list = (n_l[ii].T*Jc_l[ii]).A1.tolist()
                #     n_J_ci_max_buff.append(buff_list)

            self.out_msg.tau_cont_sum_0 = tau_cont_sum_0.A1.tolist()

            print "all_J_T_K_J :\n", all_J_T_K_J
            print "same flattened :\n", all_J_T_K_J.flatten('F').A1.tolist()

            self.out_msg.all_J_T_K_J = all_J_T_K_J.flatten('F').A1.tolist()

            #this is a check for any contact at all before populating matrices
            if n_K_J_all_buff == []:
                n_K_J_all = np.zeros((num_poss_contacts, len(q)))
            else:
                n_K_J_all = np.array(n_K_J_all_buff)
                n_K_J_all = np.vstack((n_K_J_all, np.zeros((num_poss_contacts-n_K_J_all.shape[0], len(q)))))
            if n_J_all_buff == []:
                n_J_all = np.zeros((num_poss_contacts, len(q)))
            else:
                n_J_all = np.array(n_J_all_buff)
                n_J_all = np.vstack((n_J_all, np.zeros((num_poss_contacts-n_J_all.shape[0], len(q)))))

            if n_K_J_over_buff == []:
                n_K_J_over = np.zeros((num_poss_contacts, len(q)))
            else:
                n_K_J_over = np.array(n_K_J_over_buff)
                n_K_J_over = np.vstack((n_K_J_over, np.zeros((num_poss_contacts-n_K_J_over.shape[0], len(q)))))

            if False:
                n_K_J_all = np.zeros((num_poss_contacts, len(q)))
            else:
                pass

            self.out_msg.n_K_J_all = n_K_J_all.flatten('F').tolist()
            self.out_msg.n_J_all = n_J_all.flatten('F').tolist()
            #self.out_msg.n_K_J_over = n_K_J_over.flatten('F').tolist()

            self.q_s.append(q)
            self.qd_s.append(q_dot)
            if values_n.shape[0] == 0:
                self.max_forces.append(0.0)
            else:
                self.max_forces.append(np.max(values_n))
            self.times.append(rospy.get_time())
            if True:
                matlab_dict['q_s'] = self.q_s
                matlab_dict['qd_s'] = self.qd_s
                matlab_dict['max_forces'] = self.max_forces
                matlab_dict['times'] = self.times
                matlab_dict['alpha'] = self.alpha
                matlab_dict['beta'] = self.beta
                #matlab_dict['gamma'] = self.gamma
                matlab_dict['zeta'] = self.zeta
                matlab_dict['mu'] = self.mu
                matlab_dict['A'] = A
                matlab_dict['B'] = B
                matlab_dict['A_tl'] = A_tl
                matlab_dict['A_tr'] = A_tr
                matlab_dict['A_bl'] = A_bl
                matlab_dict['A_br'] = A_br
                matlab_dict['B_tl'] = B_tl
                matlab_dict['B_tr'] = B_tr
                matlab_dict['B_bl'] = B_bl
                matlab_dict['B_br'] = B_br
                matlab_dict['delta_x_d'] = delta_x_g[0:2]
                matlab_dict['J'] = J_h
                matlab_dict['q_0'] = q
                matlab_dict['qd_0'] = q_dot
                matlab_dict['q_des_cur_0'] = q_des_cur
                matlab_dict['q_min'] = q_min
                matlab_dict['q_max'] = q_max
                matlab_dict['qd_max'] = qd_max
                matlab_dict['u_max'] = u_max
                matlab_dict['delta_f_max'] = delta_force_max
                matlab_dict['delta_rate_force_max'] = delta_rate_force_max
                matlab_dict['mass_n_J_com'] = mass_n_J_com
                matlab_dict['f_max_delta_t'] = max_impulse
                matlab_dict['tau_max_delta_t'] = max_impulse_test
                matlab_dict['tau_cont_sum_0'] = tau_cont_sum_0
                matlab_dict['n_K_J_all'] = n_K_J_all
                #matlab_dict['n_K_J_over'] = n_K_J_over
                matlab_dict['all_J_T_K_J'] = all_J_T_K_J
                matlab_dict['mass'] = Mass
                scipy.io.savemat('/home/mkillpack/Desktop/dynamics_test.mat', matlab_dict)

            # print "time to fill :", time.time()-start_fill
            # if time.time()-start_fill > self.max_time_to_fill:
            #     self.max_time_to_fill = time.time()-start_fill
            # print "MAX TIME IS :", self.max_time_to_fill

                
        #print "\n\n\n\nOUT MESSAGE >>>>>>>>>>>>>>", self.out_msg, "OUT MESSAGE >>>>>>>>>>>>>>\n\n\n\n"

        return self.out_msg


    def delta_force_bounds(self, values_n, f_thresh):
        # Compute how much the contact force can change before it hits
        # the maximum, and limit the expected increase in force by
        # this quantity.

        #n = values_n.shape[0]  
        #force_max = 1000 * np.matrix(np.ones((n,1))) 

        n = num_poss_contacts
        #     
        # if this is negative, my constraint currently says that
        # change of force has to be less than that.  Is that a bad
        # thing??? - marc
        #delta_force_max = force_max

        delta_force_max_buff = []
        delta_rate_force_max_buff = []

        for i in xrange(len(values_n)):
            if values_n[i,0] > 0.01:
                diff = f_thresh - values_n[i,0]
                print "diff is :\n", diff/self.nom_stiffness

                # if diff >= 0.:
                #     delta_force_max_buff.append(diff/self.nom_stiffness)
                #     print "delta_force_max_buff :", delta_force_max_buff
                # else:
                #     delta_force_max_buff.append(diff/self.nom_stiffness)
                #     print "delta_force_max_buff :", delta_force_max_buff

                delta_rate_force_max_buff.append(0.1/self.nom_stiffness)
                delta_force_max_buff.append(diff/self.nom_stiffness)

                #delta_force_max_buff.append(diff/self.nom_stiffness)
                #delta_rate_force_max_buff.append(-0.5/self.nom_stiffness)


                # if diff < -0.1:
                #     delta_force_max_buff.append(-0.1)
                # else:
                #     delta_force_max_buff.append(diff)
                # if diff >= 0.0:
                #     delta_force_max_buff.append(diff/self.nom_stiffness)
                # else:
                #     if diff < -5.0:
                #         delta_force_des_buff.append(-20./self.nom_stiffness)
                #     else:
                #         delta_force_des_buff.append(-20.0/self.nom_stiffness)

        if delta_rate_force_max_buff == []:
            delta_rate_force_max = np.ones((n,1))*1000
        else:
            delta_rate_force_max = np.vstack((np.array([delta_rate_force_max_buff]).T, 
                                              1000*np.ones((n-len(delta_rate_force_max_buff),1))))

        if delta_force_max_buff == []:
            delta_force_max = np.zeros((n,1))
        else:
            delta_force_max = np.vstack((np.array([delta_force_max_buff]).T, 
                                         np.ones((n-len(delta_force_max_buff),1))*1000))
            print "delta_force_max after augmenting: \n", delta_force_max

        # # If a force has exceeded the maximum use special constraints.
        # over_max = values_n > f_thresh
        # if over_max.any():
        #     # at least one of the contact forces is over the maximum
        #     # allowed
        #     delta_force_max[over_max] = 0.

        return delta_force_max, delta_rate_force_max


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


    def populate_robot_state_cb(self, msg):
        #print "robot state update :", time.time() - self.time_haptic_state
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
        
        #if True:
        with self.lock:
            self.publish_goal_marker(self.x_g, '/world')
            self.time_update = rospy.get_time()
            start_update = time.time()
            #self.get_data_local()
            self.updated_state = False
            #self.robot_state.updateHapticState()

            # self.x_h = np.matrix([0., -0.3, 0.]).reshape(3,1)
            # self.q_h_orient = [0., 0., 0., 1.]
            # self.q = [-1.0471975511964893, 0.7853981633973213, 2.356194490192115]
            # self.q_dot = [-2.1671553440683023e-13, 2.54019028034236e-13, 4.313216450668733e-13]
            # self.jep = [-1.0471975511965976, 0.7853981633974483, 2.356194490192345]
            # self.kp = [30.0, 20.0, 15.0]
            # self.Kp = np.diag(self.kp)
            # self.kd = [15.0, 10.0, 8.0]
            # self.Kd = np.diag(self.kd)
            # self.time_stamp_msg = rospy.get_time() #msg.header.stamp.to_time()
            # self.values_l_msg = [] #mpc_util.getValues(self.robot_state.skins, force=True, distance=True)
            # self.n_l_msg = [] #mpc_util.getNormals(self.robot_state.skins)
            # self.J_h = np.matrix([[0.27661922598062766, 0.17861922598060925, -0.14399999999993796], [-0.006771223916033128, 0.16296975522570623, 0.24941531628995417]])
            # self.Jc_l = []
            
            # self.x_h = self.robot_state.end_effector_position.reshape(3,1)
            # self.q_h_orient = self.robot_state.end_effector_orient_quat
            # self.q = self.robot_state.joint_angles
            # self.q_dot = self.robot_state.joint_velocities
            # self.jep = self.robot_state.desired_joint_angles
            # self.kp = self.robot_state.joint_stiffness
            # self.Kp = np.diag(self.kp)
            # self.kd = self.robot_state.joint_damping
            # self.Kd = np.diag(self.kd)
            # self.time_stamp_msg = rospy.get_time() #msg.header.stamp.to_time()
            # self.values_l_msg = mpc_util.getValues(self.robot_state.skins, force=True, distance=True)
            # self.n_l_msg = mpc_util.getNormals(self.robot_state.skins)
            # self.J_h = self.robot_state.Je[0]
            # self.Jc_l = self.robot_state.Jc


            self.x_h, self.mat_h_orient = self.robot.kinematics.FK(self.q, len(self.q))
            print "position is :\n", self.x_h
            self.jt_pos = []
            for kk in xrange(len(self.q)+1):
                pos, _ = self.robot.kinematics.FK(self.q, kk)
                self.jt_pos.append(pos)
            self.q_h_orient = tr.matrix_to_quaternion(self.mat_h_orient)
            self.q = self.robot.get_joint_angles()
            self.q_dot = self.robot.get_joint_velocities()
            self.jep = self.robot.get_ep()
            self.kp, self.kd = self.robot.get_joint_impedance() 
            self.Kp = np.diag(self.kp)
            self.Kd = np.diag(self.kd)
            self.time_stamp_msg = rospy.get_time() #msg.header.stamp.to_time()
            self.values_l_msg = []
            self.n_l_msg = []
            self.J_h = self.robot.kinematics.jacobian(self.q, self.x_h)

            if False:
                f_l, n_l, loc_l, jt_l = [], [], [], []
                time_stamp = 0.
            else:
                values_l, n_l, loc_l, jt_l, time_stamp = self.scl.force_normal_loc_joint_list(
                    normal_component_only = True,
                    return_time = True)

            self.Jc_l = []
            for jt_li, loc_li in it.izip(jt_l, loc_l):
                Jc = self.robot.kinematics.jacobian(self.q, loc_li)
                Jc[:, jt_li+1:] = 0.0
                Jc = Jc[0:3, 0:len(self.q)]
                self.Jc_l.append(Jc)

            values_n = np.matrix([(n_i.T*values_i)[0,0] for n_i, values_i in it.izip(n_l, values_l)]).T

            jep = copy.copy(self.jep)
            q = copy.copy(self.q)
            q_dot = copy.copy(self.q_dot)
            ee = copy.copy(self.x_h)
      
            # Log force info
            max_force = 0.0 

            for f_mag, nrml, loc, jt in zip(values_n, n_l, loc_l, jt_l):
                self.all_forces_buf.append(f_mag)
                self.all_forces_locs_buf.append(loc.A1)
                self.all_forces_nrmls_buf.append(nrml.A1)
                self.all_forces_jts_buf.append(jt)
      
            if values_n != []:
                max_force = np.max(values_n)
      
            self.max_force_buf.append(max_force)
            self.num_contacts_at_time_instant_buf.append(len(values_n))

            # Log robot state
            self.jep_buf.append(jep)
            self.q_buf.append(q)
            self.qdot_buf.append(q_dot)
            self.ee_pos_buf.append(ee.A1) # end effector position - not orientation

            # fit Gaussian to last 50 locations of the end effector.
            self.updateEeGaussianBuf(self.ee_gaussian_mn_buf_50,
                                     self.ee_gaussian_cov_buf_50, 50)
            # find distance (in 2D) between the means of the Gaussians
            # that are 1000 samples (10 seconds) apart.

            self.updateMeanMotion(self.ee_gaussian_mn_buf_50, step=1000)
      
            self.time_stamp_buf.append(rospy.get_time())



            self.cur_forces = values_n
            #Jc_l_send = copy.copy(self.Jc_l)

            Jc_l_send = self.Jc_l

            if self.robot_type == "sim":
                #J_h = copy.copy(self.J_h[0:2])
                J_h = self.J_h[0:2]
            else:
                rospy.loginfo("only sim is implemented")
                assert(False)
                # J_h = copy.copy(self.J_h)
                # T_quat = 0.5 * (q_h_orient[3] 
                #                 * np.matrix(np.eye(3)) 
                #                 - mpc_util.get_skew_matrix(q_h_orient[0:3]))
                # J_h[3:] = T_quat*self.J_h[3:]

                # #print "delta_x_g is :", delta_x_g

                # J_h = J_h[:,0:Kp.shape[0]] # comes into play with Cody and the wrist cover

            #Q = np.matrix(goal_state_weights)


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

            Kc_l = self.get_contact_stiffnesses(n_l, self.nom_stiffness)
            #Kc_l = self.get_contact_stiffnesses(n_l, 48000)
            #Kc_l = self.get_contact_stiffnesses(n_l, 100)
            #Kc_l = self.get_contact_stiffnesses(n_l, 10000)


            # n = the number of contacts
            n = len(n_l)

            # f_mag_list is a list of the magnitudes of the contact
            # forces
            values_mag_list  = [np.linalg.norm(val_vec) for val_vec in values_l]


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

            # # this is a hack for now, need a good fit function to map adc values to some sort of distance, should probably be conservative.

            delta_force_max, delta_rate_force_max = self.delta_force_bounds(values_n, self.f_thresh)


            err = self.x_g-self.x_h
            # print "error: ", err
            # print "goal is :", self.x_g
            err_mag = np.linalg.norm(err)

            # if err_mag < 0.05:
            #     self.alpha = 1.0

            if False:
                #self.check_termination_criterion()
                if err_mag < 0.015:
                    print "SUCCESS"
                    self.saveDataToDisk(True, "reached")
                    print "trying to die ... "
                    os._exit(0)
                if len(self.times) > 2:
                    if (self.times[-1]-self.times[0]) > 100.:
                        print "OVERALL TIMEOUT"
                        self.saveDataToDisk(False, "timeout")
                        os._exit(0)
                if (values_n != []):
                    if np.max(values_n) > 50.:
                        print "HIGH FORCE FAILURE"
                        self.saveDataToDisk(False, "high force")
                        os._exit(0)
                if (self.is_ee_moving(0.001) == False):
                    self.saveDataToDisk(False, "end-effector not moving")
                    print "EE is not moving"
                    os._exit(0)

            # if err_mag < 0.05:
            #     self.alpha = 1.0
            # else:
            #     self.alpha = 0.1
           
            # #try this to see if works
            delta_pos_g = err

            # link_axis = self.jt_pos[-1] - self.jt_pos[-2]
            # link_axis_mag = np.linalg.norm(link_axis)
            # for ll in xrange(values_n.shape[0]):
            #     vec_angle = math.acos(np.dot(values_l[ll].A1, link_axis.A1)/(np.linalg.norm(values_l[ll].A1)*link_axis_mag))
            #     if  vec_angle < 3.14/4. and jt_l[ll] == 2:
            #         z_dir = np.cross(values_l[ll].A1/np.linalg.norm(values_l[ll].A1), link_axis.A1/link_axis_mag)
            #         delta_pos_g = err + 0.02*np.matrix(np.cross(z_dir, link_axis.A1/link_axis_mag)).reshape(3,1)

            if self.planar:
                delta_x_g = delta_pos_g[0:2]
            else:
                # delta_orient_g = mpc_util.goal_orientation_in_quat(q_h_orient, q_g_orient, math.radians(0.1))
                # delta_x_g = np.vstack((delta_pos_g, delta_orient_g))
                print "this is for sim"
                assert (False)

            Mass = np.matrix(M(self.params, self.q)).reshape(self.n_jts, self.n_jts)
            self.Mass = Mass

            Coriolis = np.matrix(c(self.params, self.q, self.q_dot)).reshape(self.n_jts, self.n_jts)
            self.Coriolis = Coriolis

            #print "time to prep up to filling message :", time.time()-start_update

            msg = self.populate_dyn_mpc_msg(delta_x_g, 
                                            J_h, 
                                            self.q, 
                                            self.q_dot, 
                                            self.jep, #q_des_cur
                                            self.Kp, 
                                            self.Kd, 
                                            Mass, 
                                            Coriolis, 
                                            self.q_min, 
                                            self.q_max,
                                            self.qd_max, 
                                            values_n,
                                            values_l,
                                            delta_force_max,
                                            delta_rate_force_max,
                                            n_l,
                                            Jc_l_send,
                                            Kc_l)
            # #print msg
            #print "all of update takes :", time.time()-start_update
            
        return stop

    def waiting_for_data_from_sim(self, req):
        #time.sleep(0.005)
        #print "got into waiting"
        self.update_data()
        cvxgen_req = MpcFormattedRequest()
        cvxgen_req.opt_data = self.out_msg
          
        #print "self.out_msg :\n", self.out_msg

        resp = self.send_to_cvxgen(cvxgen_req)

        # print "q_0 \n", self.out_msg.q_0
        # print "q_max \n", self.q_max
        # print "q_min \n", self.q_min

        # if np.linalg.norm(resp.delta_jep.data) <= 0.00001: 
        #     print "FAILED TO CONVERGE or step is small"
            #print self.out_msg
            # print "cond of A_bl is :", np.linalg.cond(np.matrix(self.out_msg.A_bl).reshape(self.n_jts,self.n_jts,order='F'))
            # print "cond of A_br is :", np.linalg.cond(np.matrix(self.out_msg.A_br).reshape(self.n_jts,self.n_jts,order='F'))
            # print "cond of B_bl is :", np.linalg.cond(np.matrix(self.out_msg.B_bl).reshape(self.n_jts,self.n_jts,order='F'))
            # print "cond of B_br is :", np.linalg.cond(np.matrix(self.out_msg.B_br).reshape(self.n_jts,self.n_jts,order='F'))

            #time.sleep(5.)

        #if np.any(self.cur_forces>4):
        if True:
            delta_f_max = np.matrix(self.out_msg.delta_f_max)
            delta_x_d = np.matrix(self.out_msg.delta_x_d).reshape(2, 1, order='F')
            n_K_J_all = np.matrix(self.out_msg.n_K_J_all).reshape(num_poss_contacts, self.n_jts, order='F')
            #n_K_J_over = np.matrix(self.out_msg.n_K_J_over).reshape(num_poss_contacts, self.n_jts, order='F')
            n_J_all = np.matrix(self.out_msg.n_J_all).reshape(num_poss_contacts, self.n_jts, order='F')
            all_J_T_K_J = np.matrix(self.out_msg.all_J_T_K_J).reshape(self.n_jts,self.n_jts,order='F')
            J = np.matrix(self.out_msg.J).reshape(2,self.n_jts,order='F')
            A_bl = np.matrix(self.out_msg.A_bl).reshape(self.n_jts,self.n_jts,order='F')
            A_br = np.matrix(self.out_msg.A_br).reshape(self.n_jts,self.n_jts,order='F')
            A_tl = np.matrix(self.out_msg.A_tl).reshape(self.n_jts,self.n_jts,order='F')
            A_tr = np.matrix(self.out_msg.A_tr).reshape(self.n_jts,self.n_jts,order='F')
            B_bl = np.matrix(self.out_msg.B_bl).reshape(self.n_jts,self.n_jts,order='F')
            B_br = np.matrix(self.out_msg.B_br).reshape(self.n_jts,self.n_jts,order='F')
            B_tl = np.matrix(self.out_msg.B_tl).reshape(self.n_jts,self.n_jts,order='F')
            B_tr = np.matrix(self.out_msg.B_tr).reshape(self.n_jts,self.n_jts,order='F')
            tau_cont_sum_0 = np.matrix(self.out_msg.tau_cont_sum_0).reshape(self.n_jts,1,order='F')
            q_des_cur_0 = np.matrix(self.out_msg.q_des_cur_0).reshape(self.n_jts,1,order='F')
            qd_0 = np.matrix(self.out_msg.qd_0).reshape(self.n_jts,1,order='F')
            q_0 = np.matrix(self.out_msg.q_0).reshape(self.n_jts,1,order='F')
            q_next = A_bl*qd_0 + A_br*q_0 + B_br*(q_des_cur_0 + np.matrix(resp.delta_jep.data).reshape(self.n_jts,1)) + B_bl*tau_cont_sum_0

            delta_force = n_K_J_all[0:10]*(q_next - q_0)
            #delta_force_des = n_K_J_over*(q_next - q_0)

            print "\n\n\n\n\n LOOK HERE##################"
            print "n_K_J_all :\n", n_K_J_all[0:10,:]
            

            final_q = np.matrix(resp.final_q.data).reshape(self.n_jts,1)
            #qd_eps = np.matrix(resp.qd_eps.data).reshape(self.n_jts,1)
            f_eps = np.matrix(resp.f_eps.data).reshape(num_poss_contacts,1)

            
            print "delta_f_max :\n", delta_f_max[0:10]
            print "n_K_J_delta_q :\n", (n_K_J_all*(final_q - q_0))[0:10]
            print "f_eps :\n", f_eps[0:10]
            #print "qd_eps :\n", qd_eps
            print "distance cost :\t",  (final_q - q_0).T*J.T*J*(final_q - q_0)
            print "input cost :\t", resp.input_cost
            #print "qd_eps cost :\t", np.linalg.norm(qd_eps)
            print "f_eps cost :\t", np.linalg.norm(f_eps)
            print "total_input_change :\n", resp.total_input_change, "\n END!!!!!!!!!!!!!! \n\n\n"

            #print "final_q-q_0 \t", (final_q - q_0)
            print "product is :\n", n_K_J_all[0,:]*(final_q - q_0)
            print "lim is :\n", delta_f_max[0,0], "or \t", self.out_msg.delta_f_max[0]
            print "delta for on step :", delta_force, "\n\n"
            #assert(False)

            #alpha*quad(delta_x_d-J*(q[T+1]-q[0])) + gamma*quad(qd_eps) + mu*sum[t=0..T](quad(u[t])) + beta*quad(f_eps) + zeta

            #print "forces about 4 N, here's the predicted change\n", delta_force[0:5,:]
            # print "\n\n\nNEXT ROUND OF DATA"
            # print "delta_forcce\n", delta_force[0:8,:]
            # print "delta_force_des\n", delta_force_des[0:8,:]
            # print "n_J_all \n", n_J_all[0:8,:]
            # print "n_K_J_over \n", n_K_J_over[0:8,:]
            # print "q_des_cur \n", q_des_cur_0
            # print "tau_cont_sum \n", tau_cont_sum_0
            # print "delta_f_max \n", self.out_msg.delta_f_max
            # print "delta_f_des \n", self.out_msg.delta_f_des
            # print "all_J_T_K_J", all_J_T_K_J

            # print "A_bl \n", A_bl 
            # print "A_br \n", A_br 
            # print "A_tl \n", A_tl 
            # print "A_tr \n", A_tr 
            # print "B_bl \n", B_bl 
            # print "B_br \n", B_br 
            # print "B_tl \n", B_tl 
            # print "B_tr \n", B_tr 

            #time.sleep(3)

        #self.mpc_setup_pub(self.out_msg)

        #service call sending data to optimization and waiting for response
        #SHOULD UPDATE THIS SERVICE TO RETURN A JEP

        #print "got a callback"
        #print "time before sleep ", rospy.get_time()
        #time.sleep(1.)
        #print "time after sleep ", rospy.get_time()

        #print "resp.delta_jep :", resp.delta_jep

        #resp.delta_jep.data = input('give me the data ... please\n')
        # resp.delta_jep.data = [0, 0, 0]

        #return GetJepResponse(resp.delta_jep)
        # if len(self.q_s) <= 70:
        #     resp.delta_jep.data = [0.01, 0.01, 0.01]            
        # elif len(self.q_s) == 71:
        #     resp.delta_jep.data = [self.q[0]- self.jep[0]-0.03, 
        #                            self.q[1]- self.jep[1]-0.03,
        #                            self.q[2]- self.jep[2]-0.03]
        # elif len(self.q_s) <=140:
        #     resp.delta_jep.data = [-0.03, -0.03, -0.03]
        # elif len(self.q_s) == 141:
        #     resp.delta_jep.data = [self.q[0]- self.jep[0]+0.03, 
        #                            self.q[1]- self.jep[1]+0.03,
        #                            self.q[2]- self.jep[2]+0.03]
        # else:
        #     resp.delta_jep.data = [0.03, 0.03, 0.03] 

        return GetJepResponse(resp.delta_jep)

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

    #while_time = time.time()
    while_time = rospy.get_time()
    max_diff_time = 0.0

    # while not rospy.is_shutdown():
    #     time.sleep(0.001)

    rospy.spin()

    # while gen_data.end_trial == False:
    #     # now = rospy.get_time()
    #     # diff_time = now-while_time
    #     # while_time = now
    #     # print "diff time is :", diff_time
    #     # if diff_time > max_diff_time:
    #     #     max_diff_time = diff_time
    #     # print "MAX DIFF TIME IS:", max_diff_time
    #     #time.sleep(0.0001)
    #     # gen_data.update_data()
    #     # while gen_data.time_update == rospy.get_time():
    #     #     gen_data.update_data()
    #     #     time.sleep(0.001)
    #     # mpc_setup_pub.publish(gen_data.out_msg)

    #     time.sleep(0.001)

        #gen_data.rate.sleep()




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
