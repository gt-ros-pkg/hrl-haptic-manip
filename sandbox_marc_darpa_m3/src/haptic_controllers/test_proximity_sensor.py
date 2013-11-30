#!/usr/bin/env python  

import sys, os
import numpy as np, math
import copy

import roslib; 
roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('sandbox_marc_darpa_m3')
import rospy
import itertools as it

import hrl_lib.transforms as tr
import hrl_lib.util as ut

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Quaternion, PointStamped
from hrl_haptic_manipulation_in_clutter_msgs.msg import MPC_FormattedData, RobotHapticState, HapticMpcState
from hrl_haptic_manipulation_in_clutter_srvs.srv import HapticMPCLogging


import haptic_mpc_util as mpc_util
import multiarray_to_matrix
from threading import RLock, Timer

lock = RLock()
#should add a threading lock when populating these in callback
J_ee = None  # use Jeff's function to get msg.end_effector_jacobian in right format
q = None  #joint_angles: [-7.8966438977090547e-05, 0.0054940426394445296, -0.005249620338954486, -0.48367051608877443, 0.0011575556186009806, -0.8148536259131971, -0.00015100633665542305]
q_dot = None #joint_velocities: [-0.0019971823513416334, -0.0010371635095498878, -0.0022963724800969766, 0.0023053530526745774, 0.0017352001918239152, 0.0046732629138189095, 0.00069227411179615501]
kp = None #joint_stiffness: [2400.0, 1200.0, 1000.0, 700.0, 50.0, 50.0, 50.0]
kd = None #joint_damping: [18.0, 10.0, 6.0, 4.0, 6.0, 4.0, 4.0]
#K_j = np.diag(kp)
K_p = None
K_d = None


J_c = None ## use Jeff's function to get msg.contact_jacobians in right format
####both of the next two are wrong, because values_x can have multiple values in x-direction for multiple contacts, look at old code that got these in skin client
values_l_msg = None #this is a list of lists [[skin.values_x, skin.values_y, skin.values_z] for skin in msg.skins]
n_l_msg = None #this is a list of lists [[skin.normals_x, skin.normals_y, skin.normals_z] for skin in msg.skins]
time_stamp_msg = None
jep = None #desired_joint_angles: []
x_h = None # hand_pose.position
q_h_orient = None # hand_pose.orientation
#joint_names: ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
ma_to_m = multiarray_to_matrix.MultiArrayConverter()
# state = None
# error = None


# def controller_state_cb(msg):
#     global state, error
#     with lock:
#         state = copy.copy(msg.state)
#         error = copy.copy(msg.error)

def populate_proximity_msg(beta, delta_x_g, J_h, K_p, Jc_l, n_l, q, values_n, min_dist_mag, delta_dist_min, delta_dist_max, Q, q_min, q_max):
    with lock:
        msg = MPC_FormattedData() 

        # these gains for the multi-objective cost function were hand tuned
        # but seem to give reasonable results
        msg.alpha = 2.
        msg.beta = beta #0.000000001 #0.001 
        msg.gamma = 0.5 # 0.1 #0.1

        #weighting on the states (i.e. position is weighted more than orientation)
        msg.Q = Q.flatten('F').A1.tolist()



        # change in goal position for end effector
        msg.delta_x_d = delta_x_g #(delta_x_g[0:2]).A1.tolist()
        # print "msg.delta_x_d :\n", msg.delta_x_d            

            #jacobian of end effector
            #msg.J = J_h[0:2,:].flatten('F').A1.tolist()
        msg.J = J_h.flatten('F').A1.tolist()

        #initial config of joint angles
        msg.x_0 = list(q)
        # print "msg.x_0 :\n", msg.x_0                 

        # weighting matrix to limit the change in torque at each time step
        msg.KP_t_KP = (K_p.T*K_p).flatten('F').tolist()
        # print "msg.KP_t_KP :\n", msg.KP_t_KP

        # print "q_min :", q_min
        # print "q_max :", q_max

        # print "q is :", q

        #limits on predicted joint angles (state)
        msg.q_min = q_min
        msg.q_max = q_max

        # msg.q_min = [-6.14]*len(q_min)
        # msg.q_max = [6.14]*len(q_max)


        # print "msg.q_min :\n", msg.q_min
        # print "msg.q_max :\n", msg.q_max

        # limits on change in equilibrium angles (these numbers are probably too big)
        msg.u_min = [-0.50]*len(q_min)
        msg.u_max = [0.50]*len(q_max)
        # msg.u_min = [-0.50]*len(q_min)
        # msg.u_max = [0.50]*len(q_max)

        # print "msg.u_min :\n", msg.u_min
        # print "msg.u_max :\n", msg.u_max

        # print "msg.B :\n", msg.B

        ###need to fix this if not Cody!!!
        num_poss_contacts = 160

        # filling vectors for allowable change in force at each contact
        if np.where(values_n.A1 < 0.33)[0].tolist() == []:
            delta_dist_min = np.matrix(np.zeros((num_poss_contacts, 1)))
            delta_dist_max = np.matrix(np.zeros((num_poss_contacts, 1)))
        else:  #THis should be CHECKED THOROUGHLY
            delta_dist_min = delta_dist_min[np.where(values_n.A1 > min_dist_mag)[0].tolist()]
            delta_dist_min = np.vstack((delta_dist_min, np.zeros((num_poss_contacts-delta_dist_min.shape[0], 1))))
            delta_dist_max = delta_dist_max[np.where(values_n.A1 > min_dist_mag)[0].tolist()]
            delta_dist_max = np.vstack((delta_dist_max, np.zeros((num_poss_contacts-delta_dist_max.shape[0], 1))))

        # msg.dist_min = (np.ones(num_poss_contacts)*-1000000).tolist()
        # msg.dist_max = (np.ones(num_poss_contacts)*1000000).tolist()

        # print "dist_min", msg.dist_min
        # print "dist_max", msg.dist_max

        msg.dist_min = delta_dist_min.A1.tolist()
        msg.dist_max = delta_dist_max.A1.tolist()

        # print "msg.dist_min is :\n", msg.dist_min
        # print "msg.dist_max is :\n", msg.dist_max

        # print "msg.f_min :\n", msg.f_min
        # print "msg.f_max :\n", msg.f_max

        # filling the matrices for estimating change in force from predicted change in joint angles
        # as well as the desired decrease in contact force when forces are above threshold
        n_J_ci_buff = []
        n_J_ci_max_buff = []
        des_dist_increase = []

        # print "length of Jc_l is :", len(Jc_l)
        # print "Jc_l is :\n", Jc_l
        # print "length of values_n is :", values_n.shape
        # print "values_n is :\n", values_n
        for ii in xrange(len(Jc_l)):
            if values_n[ii, 0] >= min_dist_mag:
                buff_list = (n_l[ii].T*Jc_l[ii]).A1.tolist()
                n_J_ci_buff.append(buff_list)
            elif values_n[ii,0] < min_dist_mag:
                    #des_force_decrease.append(max_force_mag - f_n[ii,0])
                des_dist_increase.append(-0.000001)
                buff_list = (n_l[ii].T*Jc_l[ii]).A1.tolist()
                n_J_ci_max_buff.append(buff_list)

        # this is a check for any contact at all before populating matrices
        if n_J_ci_buff == []:
            n_J_ci = np.zeros((num_poss_contacts, len(q)))
        else:
            n_J_ci = np.array(n_J_ci_buff)
            n_J_ci = np.vstack((n_J_ci, np.zeros((num_poss_contacts-n_J_ci.shape[0], len(q)))))
        if n_J_ci_max_buff == []:
            n_J_ci_max = np.zeros((num_poss_contacts, len(q)))                
        else:
            n_J_ci_max = np.array(n_J_ci_max_buff)
            n_J_ci_max = np.vstack((n_J_ci_max, np.zeros((num_poss_contacts-n_J_ci_max.shape[0], len(q)))))

        msg.n_J_ci = n_J_ci.flatten('F').tolist()
        msg.n_J_ci_max = n_J_ci_max.flatten('F').tolist()

        # print "msg.n_K_ci_J_ci :\n", msg.n_K_ci_J_ci
        # print "msg.n_K_ci_J_ci_max :\n", msg.n_K_ci_J_ci_max


        if des_dist_increase != []:
            np_des_dist_increase = np.array(des_dist_increase).reshape(len(des_dist_increase),1)

            # this is ugly right now, but I am concatenating desired distance 
            # increases (scaled by the number of distances violating constraints) with extra zeros.
            msg.desired_dist_increase = (np.vstack((np_des_dist_increase, np.zeros((num_poss_contacts - np_des_dist_increase.shape[0], 1))))).flatten().tolist()
            #msg.desired_dist_increase = (np.vstack((np_des_dist_increase/np_des_dist_increase.shape[0], np.zeros((num_poss_contacts - np_des_dist_increase.shape[0], 1))))).flatten().tolist()
        else:
            msg.desired_dist_increase = (np.zeros(num_poss_contacts)).tolist()

        # print "msg.desired_dist_increase :\n", msg.desired_dist_increase
        #print "msg.desired_force_decrease :\n", msg.desired_force_decrease

        return msg



def delta_dist_bounds(values_n, min_dist, 
                      min_increase_when_under_min_dist = 0.,
                      max_increase_when_under_min_dist = 0.03):
    # Compute bounds for delta_f:  delta_f_max and delta_f_min
    #
    # all inputs should be positive
    # assert (max_pushing_force >= 0.0), "delta_f_bounds: max_pushing_force = %f < 0.0" % max_pushing_force
    # assert (max_pushing_force_increase >= 0.0), "delta_f_bounds: max_pushing_force_increase = %f < 0.0" % max_pushing_force_increase
    # assert (max_pushing_force_decrease >= 0.0), "delta_f_bounds: max_pushing_force_decrease = %f < 0.0" % max_pushing_force_decrease
    # assert (max_pulling_force >= 0.0), "delta_f_bounds: max_pulling_force = %f < 0.0" % max_pulling_force
    # assert (min_decrease_when_over_max_force >= 0.0), "delta_f_bounds: min_decrease_when_over_max_force = %f < 0.0" % min_decrease_when_over_max_force
    # assert (max_decrease_when_over_max_force >= 0.0), "delta_f_bounds: max_decrease_when_over_max_force = %f < 0.0" % max_decrease_when_over_max_force
    
    # Set delta_f_max. The change to the normal components of
    # the contact forces must be less than these
    # values. delta_f_max limits the magnitude of the force
    # with which the robot can push on the environment at each
    # of its contact locations.
    #
    # f_max is size n x 1
    #
    # Compute how much the contact force can change before it hits
    # the maximum, and limit the expected increase in force by
    # this quantity.
    n = values_n.shape[0]  
    dist_min = min_dist * np.matrix(np.ones((n,1)))

    #####I think i NEED TO MUCH MORE CAREFUL ABOUT SIGN, MAX AND MIN, this should do for initial testing though.
    delta_dist_max = values_n - dist_min
    #####I think i NEED TO MUCH MORE CAREFUL ABOUT SIGN, MAX AND MIN, this should do for initial testing though.

    

    # Also incorporate constraint on the expected increase in the
    # contact force for each contact. This limits the rate of
    # increase in pushing force.
    # delta_f_max = np.minimum(delta_f_max, 
    #                          max_pushing_force_increase * np.matrix(np.ones((n,1))))


    # Set delta_f_min. The change to the normal components of
    # the contact forces must be greater than these
    # values. delta_f_min limits the magnitude of the force
    # with which the robot can pull on the environment at each
    # of its contact locations.
    #
    # f_min is size n x 1
    #        
    #f_min = -max_pulling_force * np.matrix(np.ones((n,1)))
    delta_dist_min =   -0.30 * np.matrix(np.ones((n,1)))

    # Also incorporate constraint on the expected change of
    # the contact force for each contact
    # delta_dist_min = np.maximum(delta_f_min, 
    #                          -max_pushing_force_decrease * np.matrix(np.ones((n,1))))

    # # Setting negative values of delta_f_min to large negative
    # # numbers so that adhesive forces are not a binding constraint
    #delta_f_min[np.where(delta_f_min<=0)]=-10000

    # If a force has exceeded the maximum use special constraints.
    under_min = values_n < min_dist
    if under_min.any():
        # at least one of the contact forces is over the maximum allowed
        delta_dist_max[under_min] = 0.
        delta_dist_min[under_min] = -0.001

    # print "delta_dist_min = \n", delta_dist_min
    # print "delta_dist_max = \n", delta_dist_max

    return delta_dist_min, delta_dist_max

def populate_robot_state_cb(msg):
    with lock:
        global x_h, q_h_orient, q, q_dot, jep, kp, K_p, kd, K_d, time_stamp_msg, values_l_msg, n_l_msg, J_ee, Jc_l
        x_h = np.matrix([msg.hand_pose.position.x, 
                         msg.hand_pose.position.y, 
                         msg.hand_pose.position.z]).reshape(3,1)
        q_h_orient = [msg.hand_pose.orientation.x, 
                      msg.hand_pose.orientation.y, 
                      msg.hand_pose.orientation.z, 
                      msg.hand_pose.orientation.w] 
        q = msg.joint_angles
        q_dot = msg.joint_velocities
        jep = msg.desired_joint_angles
        kp = msg.joint_stiffness
        K_p = np.diag(kp)
        kd = msg.joint_damping
        K_d = np.diag(kd)
        time_stamp_msg = msg.header.stamp.to_time()
        values_l_msg = mpc_util.getValues(msg.skins, force=True, distance=True)
        n_l_msg = mpc_util.getNormals(msg.skins)
        J_ee = ma_to_m.multiArrayToMatrixList(msg.end_effector_jacobian)
        Jc_l = ma_to_m.multiArrayToMatrixList(msg.contact_jacobians)

if __name__ == '__main__':

    import optparse
    p = optparse.OptionParser()

    p.add_option('--ignore_skin', '--is', action='store_true',
                 dest='ignore_skin', help='ignore feedback from skin', default=False)

    p.add_option('--use_orientation', '--uo', action='store_true',
                 dest='use_orientation', help='try to go to commanded orientation in addition to position')

    p.add_option('--pr2', action='store_true', dest='pr2',
                 help='mid level for the pr2')
    
    p.add_option('--sim', action='store_true', dest='sim',
                 help='use 3 link planar capsule')

    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)

    opt, args = p.parse_args()

    rospy.init_node('test_proximity_sensor')

    mpc_setup_pub = rospy.Publisher('/formatted_mpc_data', MPC_FormattedData)

    rospy.Subscriber('/haptic_mpc/robot_state', RobotHapticState,
                     populate_robot_state_cb)

    publish_goal = rospy.Publisher('/haptic_mpc/goal_pose', PoseStamped, latch=True)

    if opt.sim == True:
        planar = True
    else:
        planar = False

    # if True:
    #     roslib.load_manifest('hrl_tactile_controller')

    if True:

        if opt.use_orientation:
            orientation_weight = 4.
        else:
            orientation_weight = 0.

        if opt.sim:
            jerk_opt_weight = 0.0001
        else:
            jerk_opt_weight = 0.0001
        save_state = False

        # # fast shelf speed
        # goal_velocity_for_hand = 0.3
        # ee_motion_threshold = 0.000

#        # ok shelf speed
#        goal_velocity_for_hand = 0.2
#        ee_motion_threshold = 0.000

       # this was for the pr2
       # # body speed
       #  goal_velocity_for_hand = 0.1
       #  ee_motion_threshold = 0.000

        stopping_dist_to_goal = 0.015
        stopping_ang_to_goal = 0.0349,

        # joint number of the point that we are controlling to go to a
        # goal location.
        # end effector = 7, elbow = 3.
        # x_h and J_h will be computed using this.
        control_point_joint_num = 7

        time_step = 0.01

        ############### Controller Specific Parameters #################
        # Specify whether or not to use sensing of normal forces
        # **** WARNING: THIS FEATURE MAY NOT BE IMPLEMENTED ****
        only_use_normal_force_sensing = True

        #Specify whether to vary impedance or not
        #use_var_impedance = ep_gen.use_var_impedance

        # Prior to optimization, the goal position for the hand, x_g,
        # is set to be at most dist_g * time_period away from the
        # current location of the hand.
        if opt.sim:
            vel_g = 0.02
        else:
            vel_g = 0.2

        # The controller attempts to constrain the magnitude of the
        # force at the contact locations to be less than
        # allowable_contact_force
        min_dist_mag = 0.02

        # The controller attempts to constrain the magnitude of the
        # change in the force at each contact location to be less
        # than max_delta_force_mag Newtons/second.
        max_delta_dist_mag = time_step * min_dist_mag

        # This weights the importance of minimizing the jerk versus
        # achieving the goal position of the hand x_g. 0.0 would mean
        # that this part of the optimization is ignored. 1.0 would
        # mean jerk optimization is as important as x_h optimization
        
        # This is the weight on the importance of acheiving the
        # goal orientation.
        position_weight = 5.
        orient_weight = orientation_weight

        #this 
        if opt.sim:
            rospy.loginfo('getting sim goal from param server')
            while rospy.get_param('/m3/software_testbed/goal') == False:
                rospy.sleep(0.01)
            x_g = np.matrix([rospy.get_param('/m3/software_testbed/goal')]).reshape(3,1) 
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = '/torso_lift_link'
            goal.pose.position.x = x_g[0,0]
            goal.pose.position.y = x_g[1,0] 
            goal.pose.position.z = x_g[2,0]
            goal.pose.orientation.w = 1.
            publish_goal.publish(goal)

        else:
            print "this should eventually be from the trajectory manager"
            x_g = np.matrix([[0.7, 0.0, 0.0]]).reshape(3,1) 
        
        # this is the goal orientation input as a matrix
        mat_g_orient = np.matrix(np.eye(3))

        if mat_g_orient == None or orient_weight == 0:
            orient_weight = 0.
            q_g_orient = tr.matrix_to_quaternion(np.matrix(np.eye(3))) 
        else:
            q_g_orient = tr.matrix_to_quaternion(mat_g_orient) 

        # Specify the precision with which the numpy arrays should be
        # printed.
        np.set_printoptions(precision=5)

        # Suppress the use of scientific notation for small numbers
        # when printing numpy arrays
        np.set_printoptions(suppress=True)

    rate = rospy.Rate(50)

    

    print "waiting for q ..."
    while q == None:
        rate.sleep()
    print "got q."

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
    end_trial = False
    while end_trial == False:
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
 
        with lock:
            theta = np.matrix(q).T

            stop = ''

            # if close enough to goal declare success!
            dist_goal_2D = np.linalg.norm((x_g - x_h)[0:2])
            dist_goal_3D = np.linalg.norm(x_g - x_h)

            #print "dist to goal is :", dist_goal_3D

            if planar:
                dist_goal = dist_goal_2D
            else:
                dist_goal = dist_goal_3D

            angle_error = ut.quat_angle(q_h_orient, q_g_orient)

            if orient_weight != 0:
                proportional_ball_radius = 0.1
                proportional_ball_dist_slope = 1.
                if dist_goal < proportional_ball_radius:
                    position_weight = position_weight * dist_goal/ (proportional_ball_radius * proportional_ball_dist_slope)

                jerk_opt_weight = max(jerk_opt_weight * position_weight * 2, jerk_opt_weight)

                proportional_ball_angle = math.radians(30)
                proportional_ball_angle_slope = 1.
                if angle_error < proportional_ball_angle:
                    orient_weight = orient_weight * angle_error/ (proportional_ball_angle * proportional_ball_angle_slope)

                if dist_goal < stopping_dist_to_goal and angle_error < stopping_ang_to_goal:
                    stop = 'Reached'
                    print 'Reached'
                else:
                    stop = ''

            else:
                # different stopping condition if ignoring orientation.
                if dist_goal < stopping_dist_to_goal:
                    stop = 'Reached'
                    end_trial = True
                else:
                    stop = ''

            if opt.ignore_skin:
                values_l, n_l= [], []
                time_stamp = 0.
                Jc_l = []
            else:
                values_l, n_l, time_stamp  =  values_l_msg, n_l_msg, time_stamp_msg 

            # n = the number of contacts
            n = len(n_l)

            # f_mag_list is a list of the magnitudes of the contact
            # forces
            values_mag_list  = [np.linalg.norm(val_vec) for val_vec in values_l]


            #print "values_mag_list is :", values_mag_list

            ######WE COULD MAYBE USE JOINT TORQUES WITHOUT THE SKIN TO ESTIMATE A STOPPING CONDITION################
            # try:
            #     if values_mag_list != [] and ep_gen.kill_controller_force > 0 and max(values_mag_list) < 0.001:
            #         stop = 'small distance %f'%(max(f_mag_list))
            #         raise EqPtGen(stop, ())
            # except AttributeError:
            #     # probably because of ep_gen.kill_controller_force not
            #     # existing. This is a temporary thing.
            #     pass
            ######WE COULD MAYBE USE JOINT TORQUES WITHOUT THE SKIN TO ESTIMATE A STOPPING CONDITION################

            # phi_curr = phi[t] = current equilibrium angles for 
            # the virtual springs on the robot's arm
            phi_curr = (np.matrix(jep).T)[0:len(q)]

            # compute contact Jacobians

            # for jt_li, loc_li in it.izip(jt_l, loc_l):
            #     Jc = robot.kinematics.Jacobian(q, loc_li)
            #     Jc[:, jt_li+1:] = 0.0
            #     Jc = Jc[0:3, 0:m]
            #     Jc_l.append(Jc)


            #print "Jc_l is :", Jc_l
            ## to make it general should do something like this, but right now we only have normal
            # Rc_l = self.contact_force_transformation_matrix(n_l)

            # # calculate the normal components of the current contact
            # # forces
            # tmp_l = [R_ci * f_ci for R_ci, f_ci in it.izip(Rc_l, f_l)]
            # f_n = np.matrix([tmp_i[0,0] for tmp_i in tmp_l]).T

            # this is a hack for now, need a good fit function to map adc values to some sort of distance, should probably be conservative.
            with lock:
                # print "values_l is :", values_l
                # print "n_l is :", n_l
                # for n_i in n_l:
                #     print "n_i.T is :", n_i.T
                # for value_i in values_l:
                #     print "value_i is :", value_i
                values_n = np.matrix([(n_i.T*values_i)[0,0] for n_i, values_i in it.izip(n_l, values_l)]).T
                #print "forces at :\n", np.where(values_n > 1.0)

                #this is a hack for now, want to separate out the two kinds of sensing.
                values_n[np.where(values_n > 1.0)] = 0.34
                Jc_l_send = copy.copy(Jc_l)
            ###########start here redefining f_bounds to distance bounds

            delta_dist_min, delta_dist_max = delta_dist_bounds(values_n, min_dist_mag, 
                                                               min_increase_when_under_min_dist = 0.0,
                                                               max_increase_when_under_min_dist = 0.03)

            # print "delta_dist_min is :", delta_dist_min
            # print "delta_dist_max is :", delta_dist_max
            

            #delta_pos_g  = epcon.goal_motion_for_hand_advait(x_h, x_g, dist_g)

            #print "dist_goal is :", dist_goal

            #delta_pos_g = (x_g-x_h)/np.linalg.norm(x_g-x_h)*0.001
            dist_g = vel_g*0.01  #velocity times time step

            err = x_g-x_h
            err_mag = np.linalg.norm(err)
            scale = 30
            if err_mag > dist_g * scale:
                delta_pos_g = dist_g * (err/err_mag)
            else:
                delta_pos_g = err/scale

            # if dist_goal >= 0.01:
            #     delta_pos_g = (x_g-x_h)/np.linalg.norm(x_g-x_h)*0.001
            # elif dist_goal >= 0.001:
            #     delta_pos_g = (x_g-x_h)/np.linalg.norm(x_g-x_h)*0.0001
            # else:
            #     delta_pos_g = (x_g-x_h)/np.linalg.norm(x_g-x_h)*0.00001
            # else:
            #     delta_pos_g = np.matrix([0., 0., 0.]).reshape(3,1)

            # if angle_error >= math.radians(10):
            #     delta_orient_g = mpc_util.goal_orientation_in_quat(q_h_orient, q_g_orient, math.radians(1.0))
            # if angle_error >= math.radians(1):
            #     delta_orient_g = mpc_util.goal_orientation_in_quat(q_h_orient, q_g_orient, math.radians(0.1))
            # if angle_error >= math.radians(0.1):
            #     delta_orient_g = mpc_util.goal_orientation_in_quat(q_h_orient, q_g_orient, math.radians(0.01))

            # print "error is ", x_g-x_h
            # print "delta_pos_g is :", delta_pos_g

            if planar:
                delta_x_g = delta_pos_g[0:2]
            else:
                delta_orient_g = mpc_util.goal_orientation_in_quat(q_h_orient, q_g_orient, math.radians(0.1))
                delta_x_g = np.vstack((delta_pos_g, delta_orient_g))

            J_h = copy.copy(J_ee[0])

            # print "J_h before is :", J_h

            if opt.sim:
                J_h = J_h[0:2]
            else:
                T_quat = 0.5 * (q_h_orient[3] 
                                * np.matrix(np.eye(3)) 
                                - mpc_util.get_skew_matrix(q_h_orient[0:3]))
                J_h[3:] = T_quat*J_h[3:]

                #print "delta_x_g is :", delta_x_g

                J_h[:, control_point_joint_num:] = 0.
                J_h = J_h[:,0:K_p.shape[0]] # comes into play with Cody and the wrist cover

            #print "J_h is :\n", J_h

            # WILL NEED TO FIX THIS TOO FOR THE ZERO WEIGHT IF CHANGE ABOVE
            # WILL NEED TO FIX THIS TOO FOR THE ZERO WEIGHT IF CHANGE ABOVE

            #Q = np.matrix(goal_state_weights)


            #put this on param server like jeff

            if opt.sim:
                q_min = np.radians(np.array([-150., -63., 0.]))
                q_max = np.radians(np.array([150., 162., 159.]))
                Q = np.matrix(np.diag([position_weight,
                                       position_weight]))
            else:
                Q = np.matrix(np.diag([position_weight,
                                       position_weight,
                                       position_weight,
                                       orient_weight,
                                       orient_weight,
                                       orient_weight]))

                q_min = np.radians(np.array([-26., -24., -41., -132, -270., -120., -180.]))
                q_max = np.radians(np.array([109., 68., 220., 0.01, 270., 0.01, 180.]))

            #print "Q is :\n", Q

            #print "K_p is :\n", K_p
            msg = populate_proximity_msg(jerk_opt_weight, 
                                         delta_x_g, 
                                         J_h, 
                                         K_p, 
                                         Jc_l_send, 
                                         n_l, 
                                         q, 
                                         values_n, 
                                         min_dist_mag, 
                                         delta_dist_min, 
                                         delta_dist_max, 
                                         Q, 
                                         q_min.tolist(), 
                                         q_max.tolist())

            if stop == '':
                mpc_setup_pub.publish(msg)

        rate.sleep()


        # while not rospy.is_shutdown():
        #     mpc_setup_pub.publish(msg)
        #     rate.sleep()

    # req.log_start = "stop"
    # try:
    #     resp = start_logging(req)
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s"%e
