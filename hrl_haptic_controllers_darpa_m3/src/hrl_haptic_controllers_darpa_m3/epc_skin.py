
import time

import numpy as np, math
import copy
from threading import RLock

import scipy.optimize as so
import itertools as it

import roslib; roslib.load_manifest('hrl_haptic_controllers_darpa_m3')
import rospy

import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_lib.viz as hv
import hrl_lib.circular_buffer as cb
import equilibrium_point_control.epc as epc

import hrl_haptic_controllers_darpa_m3.epc_skin_math as esm

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool

from hrl_srvs.srv import FloatArray_Float
from std_srvs.srv import Empty as Empty_srv
from hrl_haptic_manipulation_in_clutter_srvs.srv import LogAndMonitorInfo, LogAndMonitorInfoRequest
from hrl_haptic_manipulation_in_clutter_srvs.srv import ServiceBasedMPC, ServiceBasedMPCRequest
from hrl_haptic_manipulation_in_clutter_msgs.msg import MechanicalImpedanceParams
from hrl_haptic_manipulation_in_clutter_msgs.msg import SkinContact

class MPCData():
    def __init__(self, q, x_h, x_g, dist_g, 
                 q_h_orient, q_g_orient, 
                 position_weight, orient_weight, 
                 control_point_joint_num, 
                 Kc_l, Rc_l, Jc_l,
                 delta_f_min, delta_f_max, 
                 phi_curr, K_j, 
                 loc_l, n_l, f_l, f_n,
                 jerk_opt_weight, max_force_mag,
                 jep, time_step, save_state, stop):

        self.q = q
        self.x_h = x_h
        self.x_g = x_g
        self.dist_g = dist_g
        self.q_h_orient = q_h_orient
        self.q_g_orient = q_g_orient
        self.position_weight = position_weight
        self.orient_weight = orient_weight
        self.control_point_joint_num = control_point_joint_num
        self.Kc_l = Kc_l
        self.Rc_l = Rc_l
        self.Jc_l = Jc_l
        self.delta_f_min = delta_f_min
        self.delta_f_max = delta_f_max
        self.phi_curr = phi_curr
        self.K_j = K_j
        self.loc_l = loc_l
        self.n_l = n_l 
        self.f_l = f_l
        self.f_n = f_n
        self.jerk_opt_weight = jerk_opt_weight
        self.max_force_mag = max_force_mag
        self.jep = jep
        self.time_step = time_step
        self.save_state = save_state
        self.stop = stop

class ControllerTime():
    def __init__(self, hist_len=100):
        self.time_buf = cb.CircularBuffer(hist_len, ())
        self.av_period = None
        self.n_updates = 0
    
    def update(self):
        self.n_updates = self.n_updates + 1
        curr_time = rospy.get_time()
        self.time_buf.append(curr_time)
        period = self.prev_period()
        if self.av_period is None:
            self.av_period = period
        else:
            self.av_period = ((self.av_period * (self.n_updates-1)) + period)/self.n_updates

    def prev_period(self):
        if len(self.time_buf) >= 2:
            return self.time_buf[-1] - self.time_buf[-2] 
        else:
            return None

    def average_period(self, window_length=None):
        if window_length is None:
            return self.av_period
        else:
            avp = None
            mem_length = len(self.time_buf)
            if (mem_length - 1) < window_length:
                window_length = mem_length - 1
            if window_length >= 2:
                cnt = 0
                for t0, t1 in it.izip(self.time_buf[-window_length:], self.time_buf[-(window_length-1):]):
                    if avp is None:
                        cnt = 1
                        avp = t1 - t0
                    else:
                        avp = ((avp * cnt) + (t1 - t0))/(cnt + 1.0)
                        cnt = cnt + 1
            return avp
      
#######################################

class SkinClient():
    def __init__(self, skin_topic_list):
        self.lock = RLock()
        self.pts_list_dict = {}
        self.force_list_dict = {}
        self.normal_list_dict = {}
        self.link_names_dict = {}
        self.loc_list_dict = {}
        self.stamp_dict = {}
        self.frame = None

        self.n_topics = len(skin_topic_list)
        for i, st in enumerate(skin_topic_list):
            rospy.Subscriber(st, SkinContact, self.skin_cb, i)

    def skin_cb(self, sc, idx):
        with self.lock:
            self.link_names_dict[idx] = sc.link_names

            frm_id = sc.header.frame_id
            if 'torso_lift_link' not in frm_id:
                raise RuntimeError('Incorrect frame for SkinContact message: %s'%(frm_id))
            self.frame = frm_id
            self.stamp_dict[idx] = sc.header.stamp.to_time() # convert to time.time() format

            pts_list = []
            force_list = []
            normal_list = []
            loc_list = []

            for i in range(len(sc.link_names)):
                ff = sc.forces[i]
                f = np.matrix([ff.x, ff.y, ff.z]).T
                #if np.linalg.norm(f) < 0.1:
                #    # ignoring very small forces.
                #    # sometimes in software simulation, the force is 0.
                #    continue
                force_list.append(f)

                pts = np.matrix([sc.pts_x[i].data, sc.pts_y[i].data,
                                 sc.pts_z[i].data])
                pts_list.append(pts)

                l = sc.locations[i]
                mn = np.matrix([l.x, l.y, l.z]).T
                loc_list.append(mn)


                nn = sc.normals[i]
                n = np.matrix([nn.x, nn.y, nn.z]).T
                normal_list.append(n)

            self.pts_list_dict[idx] = pts_list
            self.force_list_dict[idx] = force_list
            self.normal_list_dict[idx] = normal_list
            self.loc_list_dict[idx] = loc_list

    def get_snapshot(self):
        with self.lock:
            f_l = []
            n_l = []
            nm_l = []
            l_l = []
            # TODO - what should the stamp be? min of all the time stamps?
            stamp = self.stamp_dict[0]

            for i in range(self.n_topics):
                f_l.extend(self.force_list_dict[i])
                n_l.extend(self.normal_list_dict[i])
                nm_l.extend(self.link_names_dict[i])
                l_l.extend(self.loc_list_dict[i])

            return f_l, n_l, nm_l, l_l, stamp


    # returns the following:
    # list of force vectors, list of normal vectors, list of joint number after which the
    # joint torque will have no effect on the contact force, and
    # optionally a time stamp
    def force_normal_loc_joint_list(self, normal_component_only, return_time=False):
        raise RuntimeError('Unimplemented funtion.')

#___________________________________________________________________________________

class EqPtGen(Exception):
    def __init__(self, stop, control_args):
        self.stop = stop
        self.control_args = control_args

#___________________________________________________________________________________

class Skin_EPC(epc.EPC):
    def __init__(self, robot, skin_client):
        epc.EPC.__init__(self, robot)
        self.scl = skin_client

        # ROS stuff
        self.lock = RLock()

        self.get_contact_stiffness = rospy.ServiceProxy('/contact_memory/estimate_contact_stiffness', FloatArray_Float)
        self.get_delta_phi_opt = rospy.ServiceProxy('get_mpc_fast', ServiceBasedMPC)
        self.wait_for_logger = rospy.ServiceProxy('/epc_skin/wait_for_logger', Empty_srv)
        self.log_and_monitor_info = rospy.ServiceProxy('/epc_skin/log_and_monitor_info', LogAndMonitorInfo)

        self.goal_marker_pub = rospy.Publisher('/epc_skin/viz/goal', Marker)

        # for synchronization with the log_and_monitor node
        self.reduce_force_running_pub = rospy.Publisher('/epc_skin/local_controller/reduce_force/running', Bool)
        self.reach_to_location_running_pub = rospy.Publisher('/epc_skin/local_controller/reach_to_location/running', Bool)
        self.pull_out_to_location_running_pub = rospy.Publisher('/epc_skin/local_controller/pull_out_to_location/running', Bool)
        self.torso_pose_pub = rospy.Publisher('/epc_skin/local_controller/start_torso_pose', Pose)

        # for updating impedance parameters in controller on the fly and at run time
        # self.K_j = None
        # self.K_d = None
        # self.var_imped_sub = rospy.Subscriber('/sim_arm/joint_impedance', MechanicalImpedanceParams, self.get_impedance)

        #this needs to be made dynamic - Marc

        # imped_msg = None
        # imped_msg = rospy.wait_for_message('/sim_arm/joint_impedance', MechanicalImpedanceParams, 10)
        # if imped_msg == None:
        #     rospy.logerr("Never got MechanicalImpedanceParam message from robot ... exiting")
        #     assert(False)
        # self.K_j = np.matrix(np.diag(imped_msg.k_p.data))
        # self.K_d = np.matrix(np.diag(imped_msg.k_d.data))

    # def get_impedance(self, msg):
    #     with self.lock:
    #         k_p = list(msg.k_p.data)
    #         k_d = list(msg.k_d.data)
    #         self.K_j = np.matrix(np.diag(k_p))
    #         self.K_d = np.matrix(np.diag(k_d))

    #----------------- visualization ------------------

    # goal that the arm is trying to reach to.
    def publish_goal_marker(self, goal_pos, frame):
        o = np.matrix([0.,0.,0.,1.]).T
        g_marker = hv.single_marker(goal_pos, o, 'sphere',
                            frame, color=(0., 1., 1., 1.),
                            scale = (0.02, 0.02, 0.02),
                            duration=0., m_id=1)
        
        g_marker.header.stamp = rospy.Time.now()
        self.goal_marker_pub.publish(g_marker)

    # intermediate waypoint that mid-level control or teleoperator
    # might generate.
    def publish_way_point_marker(self, wp_pos, frame):
        o = np.matrix([0.,0.,0.,1.]).T
        g_marker = hv.single_marker(wp_pos, o, 'sphere',
                            frame, color=(1., 1., 0., 1.),
                            scale = (0.02, 0.02, 0.02),
                            duration=0., m_id=2)
        
        g_marker.header.stamp = rospy.Time.now()
        self.goal_marker_pub.publish(g_marker)


    #----------------------
    def publish_goal(self, goal_pos, frame):
        self.publish_goal_marker(goal_pos, frame)

    def publish_way_point(self, wp_pos, frame):
        self.publish_way_point_marker(wp_pos, frame)

    #=============== jep generators ===============

###
###
###    Delta QP Controller Helper Functions
###
###

    def contact_stiffness_matrix(self, n_l, loc_l, time_stamp, k_default, 
                                 estimate_contact_stiffness,
                                 k_est_min=100.0, k_est_max=100000.0):
        # This computes the contact stiffness matrices, K_ci
        #
        # K_ci are size 3 x 3
        # K_c is size 3n x 3n
        Kc_l = []
        for n_ci, loc in it.izip(n_l, loc_l):
            # n_ci is a unit vector located at the point of contact,
            # or at the center of the taxel, that is normal to the
            # surface of the robot and points away from the surface of
            # the robot.
            #
            # This should result in a stiffness matrix that has a
            # stiffness of k in the direction of the normal and a
            # stiffness of 0 in directions orthogonal to the normal.
            #
            # If the contact location moves away from the current
            # contact location along the arm's surface normal, then
            # the magnitude of the force applied by the robot to the
            # environment should increase proportionally to the
            # distance the contact location has moved along the
            # surface normal. Given our conventions, this motion
            # projected onto the surface normal, n_ci, would result in
            # a positive value. Moreover, since we represent forces as
            # the force applied by the robot to the environment, the
            # resulting force vector should point in the same
            # direction.
            #
            n_ci = np.nan_to_num(n_ci)
            K_ci = np.outer(n_ci, n_ci)
            if estimate_contact_stiffness:
                k = self.get_contact_stiffness(np.concatenate((loc.A1, [time_stamp]))).val
                if k == -1:
                    k = k_default
                #k = k_default
                K_ci = k * K_ci
            else:
                K_ci = k_default * K_ci

            Kc_l.append(np.matrix(K_ci))
        return Kc_l


    def contact_force_transformation_matrix(self, n_l):        
        # Compute R_c, which as originally conceived, would consist of
        # rotation matrices that transform contact forces to the local
        # contact frame R_c f_c_global = f_c_local
        #
        # For now, R_c instead ignores all forces other than the force
        # normal to the surface of the robot's arm at the point of
        # contact. R_c is only used for the constraint matrices. So,
        # R_ci recovers the component of the contact force, f_ci, that
        # is normal to the surface. If f_ci points toward the arm,
        # then it represents the robot pulling on the environment. If
        # f_ci points away from the arm, then it represents the robot
        # pushing on the environment.
        #
        # R_ci f_ci = f_norm_scalar
        # R_ci is size 1 x 3
        # R_c is size n x 3n
        # 
        Rc_l = []
        for n_ci in n_l:
            # n_ci is a unit vector that is normal to the surface of
            # the robot and points away from the surface of the robot.
            R_ci = np.matrix(np.zeros((1,3)))
            R_ci[:] = n_ci[:].T
            Rc_l.append(R_ci)
        return Rc_l


    def delta_f_bounds(self, f_n, 
                       max_pushing_force, max_pulling_force,
                       max_pushing_force_increase, max_pushing_force_decrease, 
                       min_decrease_when_over_max_force, 
                       max_decrease_when_over_max_force):
        # Compute bounds for delta_f:  delta_f_max and delta_f_min
        #
        # all inputs should be positive
        assert (max_pushing_force >= 0.0), "delta_f_bounds: max_pushing_force = %f < 0.0" % max_pushing_force
        assert (max_pushing_force_increase >= 0.0), "delta_f_bounds: max_pushing_force_increase = %f < 0.0" % max_pushing_force_increase
        assert (max_pushing_force_decrease >= 0.0), "delta_f_bounds: max_pushing_force_decrease = %f < 0.0" % max_pushing_force_decrease
        assert (max_pulling_force >= 0.0), "delta_f_bounds: max_pulling_force = %f < 0.0" % max_pulling_force
        assert (min_decrease_when_over_max_force >= 0.0), "delta_f_bounds: min_decrease_when_over_max_force = %f < 0.0" % min_decrease_when_over_max_force
        assert (max_decrease_when_over_max_force >= 0.0), "delta_f_bounds: max_decrease_when_over_max_force = %f < 0.0" % max_decrease_when_over_max_force
        
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
        n = f_n.shape[0]
        f_max = max_pushing_force * np.matrix(np.ones((n,1)))
        delta_f_max = f_max - f_n
        # Also incorporate constraint on the expected increase in the
        # contact force for each contact. This limits the rate of
        # increase in pushing force.
        delta_f_max = np.minimum(delta_f_max, 
                                 max_pushing_force_increase * np.matrix(np.ones((n,1))))

        # Set delta_f_min. The change to the normal components of
        # the contact forces must be greater than these
        # values. delta_f_min limits the magnitude of the force
        # with which the robot can pull on the environment at each
        # of its contact locations.
        #
        # f_min is size n x 1
        #        
        f_min = -max_pulling_force * np.matrix(np.ones((n,1)))
        delta_f_min = f_min - f_n
        # Also incorporate constraint on the expected change of
        # the contact force for each contact
        delta_f_min = np.maximum(delta_f_min, 
                                 -max_pushing_force_decrease * np.matrix(np.ones((n,1))))

        # # Setting negative values of delta_f_min to large negative
        # # numbers so that adhesive forces are not a binding constraint
        delta_f_min[np.where(delta_f_min<=0)]=-10000

        # If a force has exceeded the maximum use special constraints.
        over_max = f_n > max_pushing_force
        if over_max.any():
            # at least one of the contact forces is over the maximum allowed
            delta_f_max[over_max] = -min_decrease_when_over_max_force
            delta_f_min[over_max] = -max_decrease_when_over_max_force

        return delta_f_min, delta_f_max


    ## The original goal_velocity_for_hand results in delta_x_g being
    # behind the current position and sometimes very erratic motion.
    # maybe because it takes the component of the velocity in the
    # direction of the error.
    # Advait finally debugged this on Sept 26, 2011 and instead of
    # attempting to fix it, is using a much simpler verison that does
    # not correct for velocity. It seems ok for the nominal stiffness
    # settings.
    def goal_motion_for_hand_advait(self, x_h, x_g, dist_g):
        err = x_g - x_h
        err_mag = np.linalg.norm(err)

        # Advait: reducing the step size as the arm reaches close to
        # the goal. scale is something that I picked empirically on
        # Cody (I printed out err_mag and dist_g)
        #scale = 4.
        scale = 30.
        if err_mag > dist_g * scale:
            delta_x_g = dist_g * (err/err_mag)
        else:
            delta_x_g = err/scale

        return delta_x_g

    ## This is very similar to advaits function above, I changed
    # gains for PR2 tests and want to make sure I don't lose 
    # his settings for Cody - marc (14 Jun 2012)
    def goal_motion_for_pr2(self, x_h, x_g, dist_g):
        err = x_g - x_h
        err_mag = np.linalg.norm(err)
        # Advait: reducing the step size as the arm reaches close to
        # the goal. scale is something that I picked empirically on
        # Cody (I printed out err_mag and dist_g)
        scale = 1.
        if err_mag > dist_g * 60.:  # this was dist_g * scale
            delta_x_g = dist_g * (err/err_mag)
        else:
            #rospy.loginfo('err_mag is less than dist_g')
            delta_x_g = err*0.0001 # this was err/scale before

        return delta_x_g

    ## This is an analogous function to goal_motion_for_hand_advait
    # except that it defines a step in orientation 
    def goal_orientation_for_pr2(self,q_h_orient, q_g_orient, max_ang_step):
        euler_cur = tr.tft.euler_from_quaternion(q_h_orient)
        
        q_diff = tr.tft.quaternion_multiply(tr.tft.quaternion_inverse(q_h_orient), q_g_orient)
        mat_diff = tr.quaternion_to_matrix(q_diff)
        
        ang, direc = tr.matrix_to_axis_angle(mat_diff)
        
        ang_mag = abs(ang)
        step_fraction = 0.01
        if step_fraction * ang_mag > max_ang_step:
            # this is pretty much always true, can clean up the code.
            step_fraction = max_ang_step / ang_mag

        interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_fraction)

#        if abs(step/ang) < 0.5:
#            step_percent = abs(step/ang) # this is saying that I want to never step more than 0.001 radians through axis
#        else:
#            step_percent = step/10.0
#        interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_percent)  #started with 0.001 

        euler_int_goal = tr.tft.euler_from_quaternion(interp_q_goal) 
        desired_euler = np.matrix(np.array(euler_int_goal)-np.array(euler_cur)).reshape(3,1)
        return desired_euler


    def goal_orientation_in_quat(self,q_h_orient, q_g_orient, max_ang_step):
        ang = ut.quat_angle(q_h_orient, q_g_orient)

        ang_mag = abs(ang)
        step_fraction = 0.001
        if step_fraction * ang_mag > max_ang_step:
            # this is pretty much always true, can clean up the code.
            step_fraction = max_ang_step / ang_mag

        #interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_fraction)
        interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_fraction)
        delta_q_des = tr.tft.quaternion_multiply(interp_q_goal, tr.tft.quaternion_inverse(q_h_orient))

#        if abs(step/ang) < 0.5:
#            step_percent = abs(step/ang) # this is saying that I want to never step more than 0.001 radians through axis
#        else:
#            step_percent = step/10.0
#        interp_q_goal = tr.tft.quaternion_slerp(q_h_orient, q_g_orient, step_percent)  #started with 0.001 

        return np.matrix(delta_q_des[0:3]).T

    def get_skew_matrix(self, vec):
        return np.matrix([[0, -vec[2], vec[1]],
                          [vec[2], 0, -vec[0]], 
                          [-vec[1], vec[0], 0]])


    ## This is a common function call for all model predictive 
    # controllers as they all use the same setup params for now
    def mpc_setup(self, ep_gen):
        time_step = ep_gen.time_step
        stopping_dist_to_goal = ep_gen.stopping_dist_to_goal
        stopping_ang_to_goal = ep_gen.stopping_ang_to_goal

        # joint number of the point that we are controlling to go to a
        # goal location.
        # end effector = 7, elbow = 3.
        # x_h and J_h will be computed using this.
        control_point_joint_num = ep_gen.control_point_joint_num

        ############### Controller Specific Parameters #################
        # Specify whether or not to use sensing of normal forces
        # **** WARNING: THIS FEATURE MAY NOT BE IMPLEMENTED ****
        only_use_normal_force_sensing = ep_gen.only_use_normal_force_sensing

        #Specify whether to vary impedance or not
        #use_var_impedance = ep_gen.use_var_impedance

        # Prior to optimization, the goal position for the hand, x_g,
        # is set to be at most dist_g * time_period away from the
        # current location of the hand.
        vel_g = ep_gen.goal_velocity_for_hand
        dist_g = time_step * vel_g # Advait doesn't want dist_g to become larger when there are multiple contacts (and the controller runs slower)

        # For now, each contact uses the same stiffness matrix, which
        # is defined such that there is a stiffness of k in the
        # direction of the surface normal, and a stiffness of 0 in
        # directions orthogonal to the normal. Consequently, shear
        # forces are currently ignored.
        k = ep_gen.k
        estimate_contact_stiffness = ep_gen.estimate_contact_stiffness

        # The controller attempts to constrain the magnitude of the
        # force at the contact locations to be less than
        # allowable_contact_force
        max_force_mag = ep_gen.allowable_contact_force

        # The controller attempts to constrain the magnitude of the
        # change in the force at each contact location to be less
        # than max_delta_force_mag Newtons/second.
        max_delta_force_mag = time_step * ep_gen.max_delta_force_mag

        # This weights the importance of minimizing the jerk versus
        # achieving the goal position of the hand x_g. 0.0 would mean
        # that this part of the optimization is ignored. 1.0 would
        # mean jerk optimization is as important as x_h optimization
        jerk_opt_weight = ep_gen.jerk_opt_weight
        
        # This is the weight on the importance of acheiving the
        # goal orientation.
        orient_weight = ep_gen.orientation_weight
        position_weight = 5.

        # this is the goal position
        x_g = ep_gen.goal_pos     

        # this is the goal orientation input as a matrix
        mat_g_orient = ep_gen.goal_orientation 

        if mat_g_orient == None or orient_weight == 0 :
            orient_weight = 0.
            q_g_orient = tr.matrix_to_quaternion(np.matrix(np.eye(3)))            
        else:
            q_g_orient = tr.matrix_to_quaternion(mat_g_orient)


        # save the state required to run the optimization at every
        # time step. can use this state to re-run the optimization
        # offline, with different parameters.
        save_state = ep_gen.save_state

        planar = ep_gen.planar

        # Specify the precision with which the numpy arrays should be
        # printed.
        np.set_printoptions(precision=4)

        # Suppress the use of scientific notation for small numbers
        # when printing numpy arrays
        np.set_printoptions(suppress=True)

        kp,_ = self.robot.get_joint_impedance()
        K_j = np.diag(kp)
        m = K_j.shape[0]

        ###########################################

        q = self.robot.get_joint_angles()
        # theta is a vector with shape n x 1
        theta = np.matrix(q).T

        # x_h is the current position of the hand
        # x_h_orient is the current orientation of the hand
        x_h, x_h_orient = self.robot.kinematics.FK(q, control_point_joint_num)
        q_h_orient = tr.matrix_to_quaternion(x_h_orient)

        stop = ''

        # if close enough to goal declare success!
        dist_goal_2D = np.linalg.norm((x_g - x_h)[0:2])
        dist_goal_3D = np.linalg.norm(x_g - x_h)
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
                raise EqPtGen(stop, ())
        else:
            # different stopping condition if ignoring orientation.
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

        if ep_gen.ignore_skin:
            f_l, n_l, loc_l, jt_l = [], [], [], []
            time_stamp = 0.
        else:
            f_l, n_l, loc_l, jt_l, time_stamp = self.scl.force_normal_loc_joint_list(
                normal_component_only = only_use_normal_force_sensing, 
                return_time = True)            

        # n = the number of contacts
        n = len(n_l)

        # f_mag_list is a list of the magnitudes of the contact
        # forces
        f_mag_list  = [np.linalg.norm(f_vec) for f_vec in f_l]

        try:
            if f_mag_list != [] and ep_gen.kill_controller_force > 0 and max(f_mag_list) > ep_gen.kill_controller_force:
                stop = 'high force %f'%(max(f_mag_list))
                raise EqPtGen(stop, ())
        except AttributeError:
            # probably because of ep_gen.kill_controller_force not
            # existing. This is a temporary thing.
            pass

        jep = np.array(self.robot.get_ep())
        # phi_curr = phi[t] = current equilibrium angles for 
        # the virtual springs on the robot's arm
        phi_curr = (np.matrix(jep).T)[0:m]

        # compute contact Jacobians
        Jc_l = []
        for jt_li, loc_li in it.izip(jt_l, loc_l):
            Jc = self.robot.kinematics.jacobian(q, loc_li)
            Jc[:, jt_li+1:] = 0.0
            Jc = Jc[0:3, 0:m]
            Jc_l.append(Jc)

        k_est_max = max(k, 1000.)
        Kc_l = self.contact_stiffness_matrix(n_l, loc_l, time_stamp, 
                                             k, estimate_contact_stiffness,
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
                                                       min_decrease_when_over_max_force = 0.,
                                                       max_decrease_when_over_max_force = 10.0)

        return MPCData(q, x_h, x_g, dist_g, 
                       q_h_orient, q_g_orient, 
                       position_weight, orient_weight, 
                       control_point_joint_num, 
                       Kc_l, Rc_l, Jc_l,
                       delta_f_min, delta_f_max, 
                       phi_curr, K_j, 
                       loc_l, n_l, f_l, f_n, 
                       jerk_opt_weight, max_force_mag,
                       jep, time_step, save_state,
                       stop)


###
###
###    Delta QP Controller
###
###
    def delta_qp_jep_gen(self, ep_gen):
        """
        This controller is based on a quasi-static model with locally
        linear spring contacts with a quadratic programming (QP)
        solution that works with the change of various parameters,
        including the change in the jep.

        Documentation can be found in the following document:    

         "Quasi-static Model with Quadratic Programming
          Optimization for Compliant Multi-contact Reaching
          in Clutter (The Delta Exploration)"

        * July 6, 2011 : Charlie got this working in simulation before
          noon. In the afternoon, Advait got it working on Cody! It
          has better performance than the absolute QP controller!

        * July 5, 2011 at 10:15pm : Charlie copied qp_jep_gen and then
          attempted to convert it to a delta version, based on the
          recent notes he checked in on "The Delta Exploration"
 
        TO DO:
                 + handle when a feasible solution is not found
                 + investigate Domain Error
                 + investigate possibility of running the optimization
                   multiple times with different model parameters. this 
                   could be used to fit the model to experience, or to 
                   handle uncertainty by selecting an approach that works
                   well across many possible worlds
                 + add constraint on delta theta? 
                   delta_theta = theta[t+1] - theta[t]
                 + add constraint on initial joint torques? 
                   tau_init = K_j (phi[t+1] - theta[t])
        """
        
           
        try:
            if self.robot.kinematics.arm_type != 'simulated':
                try:
                    if np.any(np.array(self.robot.get_joint_motor_temps()) >= 100):
                        stop = 'high motor temp'   
                        rospy.logerr('motor temps are high ... '+str(self.robot.get_joint_motor_temps()))
                        raise EqPtGen(stop, ())
                except:
                    #this is a hack right now, should update so that perhaps it performs different
                    #list of function checks on each robot (since they are different)
                    pass
            
            mpc_dat = self.mpc_setup(ep_gen)
            J_all = self.robot.kinematics.jacobian(mpc_dat.q, mpc_dat.x_h)
            delta_pos_g  = self.goal_motion_for_hand_advait(mpc_dat.x_h, 
                                                            mpc_dat.x_g, 
                                                            mpc_dat.dist_g)

            #delta_orient_g = self.goal_orientation_for_pr2(q_h_orient, q_g_orient, math.radians(0.1))
            delta_orient_g = self.goal_orientation_in_quat(mpc_dat.q_h_orient, 
                                                           mpc_dat.q_g_orient, 
                                                           math.radians(0.1)) 

            if mpc_dat.position_weight == 0.:
                delta_x_g = delta_orient_g
                J_h = J_all[3:]
                T_quat = 0.5 * (mpc_dat.q_h_orient[3] 
                                * np.matrix(np.eye(3)) 
                                - self.get_skew_matrix(mpc_dat.q_h_orient[0:3]))
                J_h = T_quat*J_h

            elif mpc_dat.orient_weight == 0.:
                delta_x_g = delta_pos_g
                J_h = J_all[0:3]
            else:
                delta_x_g = np.vstack((delta_pos_g*math.sqrt(mpc_dat.position_weight), 
                                       delta_orient_g*math.sqrt(mpc_dat.orient_weight)))
                J_h = J_all
                T_quat = 0.5 * (mpc_dat.q_h_orient[3] 
                                * np.matrix(np.eye(3)) 
                                - self.get_skew_matrix(mpc_dat.q_h_orient[0:3]))
                J_h[3:] = T_quat*J_h[3:]
                J_h[0:3] = J_h[0:3] * math.sqrt(mpc_dat.position_weight)
                J_h[3:] = J_h[3:] * math.sqrt(mpc_dat.orient_weight)

                # delta_x_g = np.vstack((delta_pos_g, 
                #                        delta_orient_g))
                # J_h = J_all
                # T_quat = 0.5 * (mpc_dat.q_h_orient[3] 
                #                 * np.matrix(np.eye(3)) 
                #                 - self.get_skew_matrix(mpc_dat.q_h_orient[0:3]))
                # J_h[3:] = T_quat*J_h[3:]
                # J_h[0:3] = J_h[0:3]
                # J_h[3:] = J_h[3:]


            J_h[:, mpc_dat.control_point_joint_num:] = 0.
            J_h = J_h[:,0:mpc_dat.K_j.shape[0]] # comes into play with Cody and the wrist cover

            if mpc_dat.save_state:
                delta_qp_opt_dict = {}
                delta_qp_opt_dict['J_h'] = J_h
                delta_qp_opt_dict['Jc_l'] = mpc_dat.Jc_l
                delta_qp_opt_dict['Kc_l'] = mpc_dat.Kc_l
                delta_qp_opt_dict['Rc_l'] = mpc_dat.Rc_l
                delta_qp_opt_dict['delta_f_min'] = mpc_dat.delta_f_min
                delta_qp_opt_dict['delta_f_max'] = mpc_dat.delta_f_max
                delta_qp_opt_dict['phi_curr'] = mpc_dat.phi_curr
                delta_qp_opt_dict['delta_x_g'] = mpc_dat.delta_x_g
                delta_qp_opt_dict['K_j'] = mpc_dat.K_j
                delta_qp_opt_dict['loc_l'] = mpc_dat.loc_l
                delta_qp_opt_dict['n_l'] = mpc_dat.n_l
                delta_qp_opt_dict['f_l'] = mpc_dat.f_l
                delta_qp_opt_dict['f_n'] = mpc_dat.f_n # normal component of the force.
                delta_qp_opt_dict['q'] = mpc_dat.q

                ut.save_pickle(delta_qp_opt_dict, 'delta_qp_opt_dict.pkl')

            cost_quadratic_matrices, cost_linear_matrices, \
            constraint_matrices, \
            constraint_vectors, lb, ub = esm.convert_to_qp(J_h, 
                                                           mpc_dat.Jc_l,
                                                           mpc_dat.K_j, 
                                                           mpc_dat.Kc_l, 
                                                           mpc_dat.Rc_l,
                                                           mpc_dat.delta_f_min,
                                                           mpc_dat.delta_f_max,
                                                           mpc_dat.phi_curr,
                                                           delta_x_g, mpc_dat.f_n, mpc_dat.q,
                                                           self.robot.kinematics,
                                                           mpc_dat.jerk_opt_weight,     #between 0.000000001 and .0000000001, cool things happened
                                                           mpc_dat.max_force_mag)

            delta_phi_opt, opt_error, feasible = esm.solve_qp(cost_quadratic_matrices, 
                                                              cost_linear_matrices, 
                                                              constraint_matrices, 
                                                              constraint_vectors, 
                                                              lb, ub, 
                                                              debug_qp=False)

            #print 'delta_phi_opt:', np.degrees(delta_phi_opt.A1)
            # update the JEP
            mpc_dat.jep[0:mpc_dat.K_j.shape[0]] = (mpc_dat.phi_curr + delta_phi_opt).A1

            #ut.get_keystroke('Hit a key to send the delta JEP')

            # warn if JEP goes beyond joint limits
            if self.robot.kinematics.within_joint_limits(mpc_dat.jep) == False:
                rospy.logwarn('Outside joint limits. They will be clamped later...')

            raise EqPtGen(mpc_dat.stop, (mpc_dat.jep.tolist(), mpc_dat.time_step*2.5))

        except EqPtGen as e:
            # return string with stopping condition and tuple with new JEP and timestep
            return e.stop, e.control_args

###
###
###    END of Delta QP Controller
###
###



    def qs_mpc_fast(self, ep_gen):

        try:
            if self.robot.kinematics.arm_type != 'simulated':
                try:
                    if np.any(np.array(self.robot.get_joint_motor_temps()) >= 100):
                        stop = 'high motor temp'   
                        rospy.logerr('motor temps are high ... '+str(self.robot.get_joint_motor_temps()))
                        raise EqPtGen(stop, ())
                except:
                    #this is a hack right now, should update so that perhaps it performs different
                    #list of function checks on each robot (since they are different)
                    pass

            mpc_dat = self.mpc_setup(ep_gen)
            J_all = self.robot.kinematics.jacobian(mpc_dat.q, mpc_dat.x_h)
            delta_pos_g  = self.goal_motion_for_hand_advait(mpc_dat.x_h, mpc_dat.x_g, mpc_dat.dist_g)
            #delta_orient_g = self.goal_orientation_for_pr2(q_h_orient, q_g_orient, math.radians(0.1))
            delta_orient_g = self.goal_orientation_in_quat(mpc_dat.q_h_orient, mpc_dat.q_g_orient, math.radians(0.1))

            if mpc_dat.position_weight == 0.:
                delta_x_g = delta_orient_g
                J_h = J_all[3:]
         
                T_quat = 0.5 * (mpc_dat.q_h_orient[3] 
                                * np.matrix(np.eye(3)) 
                                - self.get_skew_matrix(mpc_dat.q_h_orient[0:3]))
                J_h = T_quat*J_h

            elif mpc_dat.orient_weight == 0.:
                delta_x_g = delta_pos_g
                J_h = J_all[0:3]
            else:
                delta_x_g = np.vstack((delta_pos_g, delta_orient_g))
                J_h = J_all

                T_quat = 0.5 * (mpc_dat.q_h_orient[3] 
                                * np.matrix(np.eye(3)) 
                                - self.get_skew_matrix(mpc_dat.q_h_orient[0:3]))
                J_h[3:] = T_quat*J_h[3:]

                J_h[0:3] = J_h[0:3]
                J_h[3:] = J_h[3:] 

            J_h[:, mpc_dat.control_point_joint_num:] = 0.
            J_h = J_h[:,0:mpc_dat.K_j.shape[0]] # comes into play with Cody and the wrist cover

            # WILL NEED TO FIX THIS TOO FOR THE ZERO WEIGHT IF CHANGE ABOVE
            # WILL NEED TO FIX THIS TOO FOR THE ZERO WEIGHT IF CHANGE ABOVE

            if mpc_dat.save_state:
                delta_qp_opt_dict = {}
                delta_qp_opt_dict['J_h'] = J_h
                delta_qp_opt_dict['Jc_l'] = mpc_dat.Jc_l
                delta_qp_opt_dict['Kc_l'] = mpc_dat.Kc_l
                delta_qp_opt_dict['Rc_l'] = mpc_dat.Rc_l
                delta_qp_opt_dict['delta_f_min'] = mpc_dat.delta_f_min
                delta_qp_opt_dict['delta_f_max'] = mpc_dat.delta_f_max
                delta_qp_opt_dict['phi_curr'] = mpc_dat.phi_curr
                delta_qp_opt_dict['delta_x_g'] = mpc_dat.delta_x_g
                delta_qp_opt_dict['K_j'] = mpc_dat.K_j
                delta_qp_opt_dict['loc_l'] = mpc_dat.loc_l
                delta_qp_opt_dict['n_l'] = mpc_dat.n_l
                delta_qp_opt_dict['f_l'] = mpc_dat.f_l
                delta_qp_opt_dict['f_n'] = mpc_dat.f_n # normal component of the force.
                delta_qp_opt_dict['q'] = mpc_dat.q

                ut.save_pickle(delta_qp_opt_dict, 'delta_qp_opt_dict.pkl')

           
            # delta_phi_opt, opt_error, feasible = esm.solve_qp(cost_quadratic_matrices, 
            #                                                   cost_linear_matrices, 
            #                                                   constraint_matrices, 
            #                                                   constraint_vectors, 
            #                                                   lb, ub, 
            #                                                   debug_qp=False)

            # if save_state:
            #     delta_qp_opt_dict['delta_phi_opt'] = delta_phi_opt

            # self.advait_prep_time.append(time.time()-start_cont)
            #fill_mat_time = time.time()


            #Q = np.matrix(goal_state_weights)
            Q = np.matrix(np.diag([mpc_dat.position_weight,
                                   mpc_dat.position_weight,
                                   mpc_dat.position_weight,
                                   mpc_dat.orient_weight,
                                   mpc_dat.orient_weight,
                                   mpc_dat.orient_weight]))

            q_min, q_max = self.robot.kinematics.get_joint_limits()

            req = self.populate_request(delta_x_g, 
                                        J_h, 
                                        mpc_dat.K_j, 
                                        mpc_dat.Kc_l, 
                                        mpc_dat.Jc_l, 
                                        mpc_dat.n_l, 
                                        mpc_dat.q, 
                                        mpc_dat.f_n, 
                                        mpc_dat.max_force_mag, 
                                        mpc_dat.delta_f_min, 
                                        mpc_dat.delta_f_max, 
                                        Q, 
                                        q_min.tolist(), 
                                        q_max.tolist())
            try:
                #this is the service call to the mpc optimization code (c++)
                #start = time.time()
                res = self.get_delta_phi_opt(req)
                #print "time to compute is: ",time.time()-start
                mpc_dat.jep[0:mpc_dat.K_j.shape[0]] = (mpc_dat.phi_curr + np.matrix(res.delta_phi_opt).reshape(mpc_dat.K_j.shape[0],1)).A1

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                    #if the service call failed, don't change the q_eq
                mpc_dat.jep[0:mpc_dat.K_j.shape[0]] = (mpc_dat.phi_curr).A1


            # no_adhesion = False
            # while no_adhesion == False:
            #     no_adhesion = True

            #     req = self.populate_request(delta_x_g, J_h, K_j, Kc_l, Jc_l, n_l, q, f_n, max_force_mag, delta_f_min, delta_f_max)                
              
            #     try:
            #         #this is the service call to the mpc optimization code (c++)
            #         res = self.get_delta_phi_opt(req)
            #         jep[0:m] = (phi_curr + np.matrix(res.delta_phi_opt).reshape(3,1)).A1

            #     except rospy.ServiceException, e:
            #         print "Service call failed: %s"%e
            #         #if the service call failed, don't change the q_eq
            #         jep[0:m] = (phi_curr).A1

            #     for ii in xrange(len(Jc_l)):
            #     #f_min <= n_K_ci_J_ci*(x[1]-x[0]) <= f_max  #limits on allowable forces
            #         if n_l[ii].T*Kc_l[ii]*Jc_l[ii]*np.array(res.predicted_joint_angles) <= 0:
            #             no_adhesion = False
            #             #########start right here##!!!!!!!!!!!!!
            #             #remove this contact for n_l, f_n, Kc_l and Jc_l

            # warn if JEP goes beyond joint limits
            if self.robot.kinematics.within_joint_limits(mpc_dat.jep) == False:
                rospy.logwarn('Outside joint limits. They will be clamped later...')

            #print "jep is :\n", jep.tolist()
            raise EqPtGen(mpc_dat.stop, (mpc_dat.jep.tolist(), mpc_dat.time_step*2.5))#, delta_qp_opt_dict)

        except EqPtGen as e:
            # return string with stopping condition and tuple with new JEP and timestep
            return e.stop, e.control_args


    def populate_request(self, delta_x_g, J_h, K_j, Kc_l, Jc_l, n_l, q, f_n, max_force_mag, delta_f_min, delta_f_max, Q, q_min, q_max):
        req = ServiceBasedMPCRequest()
        # these gains for the multi-objective cost function were hand tuned
        # but seem to give reasonable results
        req.alpha = 100.0
        req.beta = 0.001
        req.gamma = 0.1
        
        #weighting on the states (i.e. position is weighted more than orientation)
        req.Q = Q.flatten('F').A1.tolist()
        # print "req.Q :\n", req.Q


        # change in goal position for end effector
        req.delta_x_d = delta_x_g #(delta_x_g[0:2]).A1.tolist()
        # print "req.delta_x_d :\n", req.delta_x_d            

        #jacobian of end effector
        #req.J = J_h[0:2,:].flatten('F').A1.tolist()
        req.J = J_h.flatten('F').A1.tolist()
        # print "req.J :\n", req.J                    

        #initial config of joint angles
        req.x_0 = list(q)
        # print "req.x_0 :\n", req.x_0                 

        # weighting matrix to limit the change in torque at each time step
        req.KP_t_KP = (K_j.T*K_j).flatten('F').tolist()
        # print "req.KP_t_KP :\n", req.KP_t_KP
        
        #limits on predicted joint angles (state)
        req.q_min = q_min
        req.q_max = q_max
        
        # print "req.q_min :\n", req.q_min
        # print "req.q_max :\n", req.q_max

        # limits on change in equilibrium angles (these numbers are probably too big)
        req.u_min = [-0.50]*len(q_min)
        req.u_max = [0.50]*len(q_max)

        # print "req.u_min :\n", req.u_min
        # print "req.u_max :\n", req.u_max
        
        # calculation of the input weighting matrix for B*q_eq
        sum_matrix = np.matrix(np.zeros((3,3)))
        if Jc_l != []:
            for ii in xrange(len(Jc_l)):
                sum_matrix = sum_matrix+Jc_l[ii].T*Kc_l[ii]*Jc_l[ii]
        else:
            sum_matrix = np.matrix(np.zeros((len(q), len(q))))

        req.B = (np.linalg.inv(K_j+sum_matrix)*K_j).flatten('F').A1.tolist() #A1.tolist()

        # print "req.B :\n", req.B

        ###need to fix this if not Cody!!!
        num_poss_contacts = 60
            
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

        # print "req.f_min :\n", req.f_min
        # print "req.f_max :\n", req.f_max

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
                #des_force_decrease.append(max_force_mag - f_n[ii,0])
                des_force_decrease.append(-0.2)
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

        # print "req.n_K_ci_J_ci :\n", req.n_K_ci_J_ci
        # print "req.n_K_ci_J_ci_max :\n", req.n_K_ci_J_ci_max


        if des_force_decrease != []:
            np_des_force_decrease = np.array(des_force_decrease).reshape(len(des_force_decrease),1)
            req.desired_force_decrease = (np.vstack((np_des_force_decrease, np.zeros((num_poss_contacts - np_des_force_decrease.shape[0], 1))))).flatten().tolist()
        else:
            req.desired_force_decrease = (np.zeros(num_poss_contacts)).tolist()

        #print "req.desired_force_decrease :\n", req.desired_force_decrease

        return req

#------------------------------------------
# local behaviors.
#------------------------------------------

    ##
    # control_param_dict - specify all the parameters of this controller.
    #                      Should be sufficient to run this controller again.
    # logging_name - name that will be used while saving the pkl
    #                (useful when we have multiple controllers as part of one trial,
    #                to associate logs with the specific run of the controller.)
    #
    # monitor_param_dict - parameters used while monitoring this particular run of the
    #                      local controller. Will go to the log_and_monitor node via ROS
    #                      service call.
    #
    # eq_gen_type - 'mpc_qs_1'
    def greedy_to_goal(self, goal_pos, goal_orientation,
                       control_param_dict, timeout, eq_gen_type,
                       logging_name, monitor_param_dict):
        controller_name = 'reach_to_location'
        self.announce_controller_start(controller_name, goal_pos,
                                       logging_name,
                                       monitor_param_dict)

        control_param_dict['goal_pos'] = goal_pos
        control_param_dict['goal_orientation'] = goal_orientation
        time_step = control_param_dict['time_step']

        if eq_gen_type == 'mpc_qs_1':
            eq_gen_func = self.delta_qp_jep_gen
        elif eq_gen_type == 'qs_mpc_fast':
            eq_gen_func = self.qs_mpc_fast
        else:
            raise RuntimeError('Unknown eq_gen_type: %s'%eq_gen_type)

        if self.robot.kinematics.arm_type == 'real':
            #Cody or the PR2
            ep_gen = epc.EP_Generator(eq_gen_func, self.robot.set_ep,
                                      self.robot.kinematics.clamp_to_joint_limits)
        else:
            # not clamping the JEPs in software simulation.
            ep_gen = epc.EP_Generator(eq_gen_func, self.robot.set_ep)

        ep_gen.__dict__.update(control_param_dict)

        ep_gen.reaching_in = True
        if eq_gen_type == 'mpc_qs_1':
            ep_gen.control_point_joint_num = self.robot.kinematics.n_jts
        elif eq_gen_type == 'qs_mpc_fast':
            ep_gen.control_point_joint_num = self.robot.kinematics.n_jts

        res = self.epc_motion(ep_gen, time_step, timeout = timeout)
        self.announce_controller_stop(controller_name, logging_name,
                                      control_param_dict)
        return res

    def pull_out_elbow(self, elbow_goal_pos, control_param_dict, timeout,
                       eq_gen_type, logging_name, monitor_param_dict):
        controller_name = 'pull_out_to_location'
        self.announce_controller_start(controller_name,
                                       elbow_goal_pos, logging_name,
                                       monitor_param_dict)

        control_param_dict['goal_pos'] = elbow_goal_pos
        time_step = control_param_dict['time_step']

        if eq_gen_type == 'mpc_qs_1':
            eq_gen_func = self.delta_qp_jep_gen
        else:
            raise RuntimeError('%s is unimplemented in pull_out_elbow.'%eq_gen_type)

        ep_gen = epc.EP_Generator(eq_gen_func, self.robot.set_ep,
                                  self.robot.kinematics.clamp_to_joint_limits)
        ep_gen.__dict__.update(control_param_dict)

        ep_gen.reaching_in = False
        if eq_gen_type == 'mpc_qs_1':
            if self.robot.kinematics.n_jts == 3:
                ep_gen.control_point_joint_num = 3
            else:
                ep_gen.control_point_joint_num = 3

        res = self.epc_motion(ep_gen, time_step, timeout = timeout)

        self.announce_controller_stop(controller_name, logging_name,
                                      control_param_dict)
        return res

    # slowly change the JEP to become equal to the current joint
    # angles. as a contingency measure to keep the forces low.
    def reduce_force(self, control_param_dict, logging_name,
                     monitor_param_dict):
        q = self.robot.get_joint_angles()
        local_goal = self.robot.kinematics.FK(q)[0]

        controller_name = 'reduce_force'
        self.announce_controller_start(controller_name, local_goal,
                                       logging_name,
                                       monitor_param_dict)

        time_step = 0.1
        jep_mag = math.radians(1.)
        target_q = np.array(q)

        def eq_gen_func(eq_gen):
            jep = np.array(self.robot.get_ep())
            diff = target_q - jep
            diff_mag = np.linalg.norm(diff)
            if diff_mag < 2*jep_mag:
                stop = 'done'
                return stop, ()

            delta_jep = (diff / diff_mag) * jep_mag
            jep += delta_jep
            return '', (jep.tolist(), time_step*1.5)

        ep_gen = epc.EP_Generator(eq_gen_func, self.robot.set_ep)
        res = self.epc_motion(ep_gen, time_step)
        self.announce_controller_stop(controller_name, logging_name,
                                      control_param_dict)
        return res


#----------------------------------------------------------------
# synchronization with the log_and_monitor node
#----------------------------------------------------------------

    def announce_controller_start(self, controller, local_goal,
                                  logging_name, monitor_param_dict):
        if controller == 'reduce_force':
            self.reduce_force_running_pub.publish(True)
        if controller == 'reach_to_location':
            self.reach_to_location_running_pub.publish(True)
        if controller == 'pull_out_to_location':
            self.pull_out_to_location_running_pub.publish(True)

        p, rot = self.current_torso_pose()
        pose = Pose()

        pose.position.x = p[0,0]
        pose.position.y = p[1,0]
        pose.position.z = p[2,0]

        q = tr.matrix_to_quaternion(rot)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        lg = Point()
        lg.x = local_goal[0,0]
        lg.y = local_goal[1,0]
        lg.z = local_goal[2,0]

        req = LogAndMonitorInfoRequest(**monitor_param_dict)
        req.logging_name = logging_name
        req.torso_pose = pose
        req.local_goal = lg

        self.log_and_monitor_info(req)

    def announce_controller_stop(self, controller, logging_name,
                                 control_param_dict):
        if controller == 'reduce_force':
            self.reduce_force_running_pub.publish(False)
        if controller == 'reach_to_location':
            self.reach_to_location_running_pub.publish(False)
        if controller == 'pull_out_to_location':
            self.pull_out_to_location_running_pub.publish(False)

        # remove things that I added within the local controller
        # function.
        control_param_dict.pop('goal_pos', None)
        ut.save_pickle(control_param_dict, logging_name+'_param_dict.pkl')

        #rospy.loginfo('Waiting for logger after %s'%controller)
        self.wait_for_logger()
        #rospy.loginfo('Done')

    #-----------------------------------------------------------------
    # functions for cases with mobile base (override when you actually
    # have a mobile base.)
    #-----------------------------------------------------------------

    # return position and orientation of torso in the world coord
    # frame.
    def current_torso_pose(self):
        p = np.matrix([0.,0.,0.]).T
        rot = np.matrix(np.eye(3))
        return p, rot

    def torso_to_world(self, p):
        trans, rot = self.current_torso_pose()
        p_world = trans + rot * p
        return p_world
        
    def torso_to_world_rotation(self, r):
        trans, rot = self.current_torso_pose()
        r_world = rot * r
        return r_world
        
    def world_to_torso(self, p):
        trans, rot = self.current_torso_pose()
        p_torso = rot.T * (p - trans)
        return p_torso

    def world_to_torso_rotation(self, r):
        trans, rot = self.current_torso_pose()
        r_torso = rot.T * r
        return r_torso







#######################################################################
# this is the old euler angle version of orientation control, got     #
# stuck less in local minima, sometimes had weird behavior            #
#######################################################################
# euler_cur = tr.tft.euler_from_quaternion(q_h_orient)
# roll = euler_cur[0]
# pitch = euler_cur[1]
# yaw = euler_cur[2]

# T = np.matrix([[1.,  0., math.sin(pitch)],
#                [0.,  math.cos(roll),  -math.cos(pitch)*math.sin(roll)],
#                [0.,  math.sin(roll),  math.cos(pitch)*math.cos(roll)]])
# J_h = T.T*J_h
#######################################################################
# this is the old euler angle version of orientation control, got     #
# stuck less in local minima, sometimes had weird behavior            #
#######################################################################


#    def greedy_eq_gen(self, ep_gen):
#        time_step = ep_gen.time_step
#        max_jep_mag = ep_gen.max_jep_mag
#        proportional_jep_mag_dist = ep_gen.proportional_jep_mag_dist
#        stopping_dist_to_goal = ep_gen.stopping_dist_to_goal
#
#        goal_pos = ep_gen.goal_pos
#        planar = ep_gen.planar
#        ignore_wrist_joints = ep_gen.ignore_wrist_joints
#        m = self.robot.kinematics.n_jts
#        if ignore_wrist_joints and m == 7:
#            m = 4
#
#        try:
#            q = self.robot.get_joint_angles()
#            # position and rotation of the hand 
#            p, r = self.robot.kinematics.FK(q)
#            # Jacobian for left/right arm, joint angles, and position on arm
#            J_all = self.robot.kinematics.Jacobian(q, p)
#            J = J_all[0:3, 0:m]
#
#            # compute and log error distance between hand position goal and current hand position
#            err = goal_pos - p
#            dist_goal_2D = np.linalg.norm(err[0:2,:])
#            dist_goal_3D = np.linalg.norm(err)
#
#            vel_hand = ep_gen.goal_velocity_for_hand
#            v = self.goal_motion_for_hand_advait(p, goal_pos, vel_hand * time_step)
#
#            # delta JEP = pseudo-inverse of Jacobian * err
#            #d_jep = np.linalg.pinv(J) * (goal_pos - p)
#            d_jep = np.linalg.pinv(J) * v
#
#            d_jep_mag = np.linalg.norm(d_jep)
#            d_jep_mag_cutoff = min(d_jep_mag, max_jep_mag)
#
#            # for most distances updates jep by a fixed magnitude, unless close, which does proportional
#            d_jep_scaled = d_jep / d_jep_mag * d_jep_mag_cutoff
#            # get the current JEP (jointspace equilibrium point)
#            jep = np.array(self.robot.get_ep())
#            # increment current JEP
#            jep[0:m] = jep[0:m] + d_jep_scaled.A1
#
#            ######## STOPPING CONDITIONS FOLLOW THIS LINE ########
#            stop = ''
#
#            # if close enough to goal declare success!
#            if planar:
#                dist_goal = dist_goal_2D
#            else:
#                dist_goal = dist_goal_3D
#
#            if dist_goal < stopping_dist_to_goal:
#                stop = 'Reached'
#            
#            # clamp JEP to be within joint limits
#            jep = self.robot.kinematics.clamp_to_joint_limits(jep)
#
#            raise EqPtGen(stop, (jep.tolist(), time_step*1.5))
#
#        except EqPtGen as e:
#            # return string with stopping condition and tuple with new JEP and timestep
#            return e.stop, e.control_args




