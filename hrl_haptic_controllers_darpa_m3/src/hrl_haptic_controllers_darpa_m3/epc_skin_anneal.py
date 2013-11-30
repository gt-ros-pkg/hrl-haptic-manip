
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

import hrl_haptic_controllers_darpa_m3.epc_skin_math_anneal as esm

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool

from hrl_srvs.srv import FloatArray_Float
from std_srvs.srv import Empty as Empty_srv
from hrl_haptic_manipulation_in_clutter_srvs.srv import LogAndMonitorInfo, LogAndMonitorInfoRequest
from hrl_haptic_manipulation_in_clutter_msgs.msg import MechanicalImpedanceParams
from hrl_haptic_manipulation_in_clutter_msgs.msg import SkinContact



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
            if self.stamp_dict == {}:
                stamp = 0.0
            else:
                stamp = self.stamp_dict[0]

                for i in range(self.n_topics):
                    f_l.extend(self.force_list_dict[i])
                    n_l.extend(self.normal_list_dict[i])
                    nm_l.extend(self.link_names_dict[i])
                    l_l.extend(self.loc_list_dict[i])

            return f_l, n_l, nm_l, l_l, stamp

    #EDIT by dpark 20120929
    def get_pts_snapshot(self):
        with self.lock:
            f_l = []
            n_l = [] 
            p_l = [] 

            if self.stamp_dict == {}:
                stamp = 0.0
            else:
                stamp = self.stamp_dict[0]

                for i in range(self.n_topics):
                    f_l.extend(self.force_list_dict[i])
                    n_l.extend(self.normal_list_dict[i])
                    p_l.extend(self.pts_list_dict[i])

            return f_l, n_l, p_l, stamp


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

        ## used by delta QP controller
        self.prev_x_h = None
        self.controller_time = ControllerTime(1000)

        ## Precompute the vertices for various tessellations of a 3D sphere for
        ## use with 3D constraints.
        ##[(n_subdivisions, n_faces), ...] = [(1, 42), (2, 162), (3, 642), (4, 2562), (5, 10242), (6, 40962)]
        #self.spheres = [ss.sphere_vertices(n_subdivisions) for n_subdivisions in range(1,6)]
        #self.spheres = [(len(s), s) for s in self.spheres]

        # ROS stuff
        self.lock = RLock()

        self.get_contact_stiffness = rospy.ServiceProxy('/contact_memory/estimate_contact_stiffness', FloatArray_Float)
        self.wait_for_logger = rospy.ServiceProxy('/epc_skin/wait_for_logger', Empty_srv)
        self.log_and_monitor_info = rospy.ServiceProxy('/epc_skin/log_and_monitor_info', LogAndMonitorInfo)

        self.goal_marker_pub = rospy.Publisher('/epc_skin/viz/goal', Marker)

        # for synchronization with the log_and_monitor node
        self.reduce_force_running_pub = rospy.Publisher('/epc_skin/local_controller/reduce_force/running', Bool)
        self.reach_to_location_running_pub = rospy.Publisher('/epc_skin/local_controller/reach_to_location/running', Bool)
        self.pull_out_to_location_running_pub = rospy.Publisher('/epc_skin/local_controller/pull_out_to_location/running', Bool)
        self.torso_pose_pub = rospy.Publisher('/epc_skin/local_controller/start_torso_pose', Pose)

        # for synchronization with simulation
        self.controller_running_pub = rospy.Publisher('/epc_skin/local_controller/controller_running', Bool)
        # self.finished_optimization_pub = rospy.Publisher('/epc_skin/local_controller/finish_optimization', Bool)
        # self.got_start_sub = rospy.Subscriber('/epc_skin/local_controller/got_start_optimization', Bool, self.got_start_cb)
        # self.got_start = False
        # self.got_finish_sub = rospy.Subscriber('/epc_skin/local_controller/got_finish_optimization', Bool, self.got_finish_cb)
        # self.got_finish = False

        # for updating impedance parameters in controller on the fly and at run time
        self.K_j = None
        self.var_imped_sub = rospy.Subscriber('/sim_arm/joint_impedance', MechanicalImpedanceParams, self.get_impedance)

        #this needs to be made dynamic - Marc

        if self.robot.kinematics.n_jts == 3 or self.robot.kinematics.n_jts == 6:
            msg = rospy.wait_for_message('/sim_arm/joint_impedance', MechanicalImpedanceParams, 10)
            self.K_j = np.matrix(np.diag(msg.k_p.data))
            # k1 = 30
            # k2 = 20
            # k3 = 15
            # self.K_j = np.matrix(np.diag([k1,k2,k3]))

    def got_start_cb(self, msg):
        self.got_start = msg.data

    def got_finish_cb(self, msg):
        self.got_finish = msg.data

    def get_impedance(self, msg):
        with self.lock:
            k_p = list(msg.k_p.data)
            k_d = list(msg.k_d.data)
            self.K_j = np.matrix(np.diag(k_p))

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
            K_ci = k_default * K_ci

            ###############################################################
            # this was breaking the rate of the controller, will need to update
            # to publishers if we want to do this online
            ###############################################################
            # if estimate_contact_stiffness:
            #     k = self.get_contact_stiffness(np.concatenate((loc.A1, [time_stamp]))).val
            #     if k == -1:
            #         k = k_default
            #     #k = k_default
            #     K_ci = k * K_ci
            # else:
            #     K_ci = k_default * K_ci

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

        # If a force has exceeded the maximum use special constraints.
        over_max = f_n > max_pushing_force
        if over_max.any():
            # at least one of the contact forces is over the maximum allowed
            delta_f_max[over_max] = -min_decrease_when_over_max_force
            delta_f_min[over_max] = -max_decrease_when_over_max_force

        return delta_f_min, delta_f_max


#    def delta_phi_bounds(self, phi_curr):
#        m = phi_curr.shape[0]
#        # Compute delta_phi_min and delta_phi_max
#        min_phi, max_phi = self.robot.kinematics.get_joint_limits()
#        #small term to subtract off since the constraint is <=
#        #padding = (2.0 * np.pi)/10000.0 # radians
#        min_phi = (np.matrix(min_phi).T)[0:m]  #+ padding
#        max_phi = (np.matrix(max_phi).T)[0:m]  #- padding
#
#        # Also incorporate equilibrium angle limits into this
#        # constraint. Ideally, this should eliminate the need for
#        # clamping.
#        delta_phi_max = max_phi - phi_curr
#        delta_phi_min = min_phi - phi_curr
#        return delta_phi_min, delta_phi_max

    
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
        scale = 1.
        if err_mag > dist_g * scale:
            delta_x_g = dist_g * (err/err_mag)
        else:
            #rospy.loginfo('err_mag is less than dist_g')
            delta_x_g = err/scale

        return delta_x_g

    def goal_motion_for_hand(self, x_h, x_g, x_h_vel, vel_g, dist_g, 
                             estimated_time_step, use_proportional_control, 
                             proportional_constant,
                             debug=False):
        # Define delta_x_g
        #
        # Create the goal for the change in position of the hand
        err = x_g - x_h
        err_mag = np.linalg.norm(err)
        t = estimated_time_step

        # correct for current hand velocity
        if not (x_h_vel is None):
            if err_mag > 0.000001:
                current_speed = (err/err_mag).T * x_h_vel
                current_speed = current_speed[0,0]
                current_speed = np.linalg.norm(x_h_vel)
                vel_err = vel_g - current_speed
                if use_proportional_control:
                    dist_g = t * vel_err *  proportional_constant
                else:
                    dist_g = t * vel_err

        # Advait: reducing the step size as the arm reaches close to
        # the goal. scale is something that I picked empirically on
        # Cody (I printed out err_mag and dist_g)
        scale = 10.
        if err_mag > dist_g * scale:
            delta_x_g = dist_g * (err/err_mag)
        else:
            #rospy.loginfo('err_mag is less than dist_g')
            delta_x_g = err/scale

        return delta_x_g

    

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

        if False:
            print
            print 'average period() =', avp
            if not (avp is None):
                print 'average freq =', 1.0/avp
            avp = self.controller_time.average_period(5)
            print 'average period(5) =', avp
            if not (avp is None):
                print 'average freq =', 1.0/avp
            avp = self.controller_time.prev_period()
            print 'prev_period() =', avp
            if not (avp is None):
                print 'average freq =', 1.0/avp
            print

        time_step = ep_gen.time_step
        stopping_dist_to_goal = ep_gen.stopping_dist_to_goal

        x_g = ep_gen.goal_pos
        
        fragile_center = ep_gen.__dict__.get('fragile_center', None)
        fragile_radius = ep_gen.__dict__.get('fragile_radius', None)

        # joint number of the point that we are controlling to go to a
        # goal location.
        # end effector = 7, elbow = 3.
        # x_h and J_h will be computed using this.
        control_point_joint_num = ep_gen.control_point_joint_num

#        ep_gen.fragile_center = np.matrix([0.4, -0.38, 0.]).T
#        ep_gen.fragile_radius = 0.2
        fragile_force_mag = 2.

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
                "jerk_opt_weight" : 0.00001, # 0.0 means ignore
                #"jerk_opt_weight" : 0.01, # 0.0 means ignore

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

        #Specify whether to vary impedance or not
        use_var_impedance = param_dict["use_var_impedance"]

        # Prior to optimization, the goal position for the hand, x_g,
        # is set to be at most dist_g * time_period away from the
        # current location of the hand.
        vel_g = ep_gen.goal_velocity_for_hand
        #dist_g = t * vel_g
        dist_g = time_step * vel_g # Advait doesn't want dist_g to become larger when there are multiple contacts (and the controller runs slower)

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

        # treat certain regions as fragile and apply less force on
        # contacts in those regions. the fragile region specification
        # is in the ep_gen object.
        enable_fragile_regions = param_dict['enable_fragile_regions']
        if fragile_center == None or fragile_radius == None:
            enable_fragile_regions = False

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
        
            # used to synch controller and simulation.  Will need to update when merge branch with
            # master branch - marc
            if not robot_name == "Cody":
                self.controller_running_pub.publish(Bool(True))

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
            if use_var_impedance == True:
                pass

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

            if enable_fragile_regions:
                # compute which of these forces are in the 'fragile'
                # region.
                center = ep_gen.fragile_center
                r = ep_gen.fragile_radius
                fragile_flag_l = [np.linalg.norm((loc-center)[0:2]) < r for loc in loc_l]
                # this should have the same effect as a max force of
                # fragile_force_mag for the forces that were marked as
                # being in the 'fragile' region.
                if fragile_flag_l != []:
                    f_n[np.where(fragile_flag_l)] += (max_force_mag - fragile_force_mag)

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

            if save_state:
                delta_qp_opt_dict = {}
                delta_qp_opt_dict['J_h'] = J_h
                delta_qp_opt_dict['Jc_l'] = Jc_l
                delta_qp_opt_dict['Kc_l'] = Kc_l
                delta_qp_opt_dict['Rc_l'] = Rc_l
                delta_qp_opt_dict['delta_f_min'] = delta_f_min
                delta_qp_opt_dict['delta_f_max'] = delta_f_max
                delta_qp_opt_dict['phi_curr'] = phi_curr
                delta_qp_opt_dict['delta_x_g'] = delta_x_g
                delta_qp_opt_dict['K_j'] = K_j
                delta_qp_opt_dict['loc_l'] = loc_l
                delta_qp_opt_dict['n_l'] = n_l
                delta_qp_opt_dict['f_l'] = f_l
                delta_qp_opt_dict['f_n'] = f_n # normal component of the force.
                delta_qp_opt_dict['q'] = q

                ut.save_pickle(delta_qp_opt_dict, 'delta_qp_opt_dict.pkl')

            #This publisher waits for handshake from simulator to say that it knows opt is starting
            # if not robot_name == "Cody":
            #     while not self.got_start:

            # self.controller_running_pub.publish(Bool(True))
            #     self.got_start = False

            cost_quadratic_matrices, cost_linear_matrices, \
            constraint_matrices, \
            constraint_vectors, lb, ub = esm.convert_to_qp(J_h, Jc_l,
                                                    K_j, Kc_l, Rc_l,
                                                    delta_f_min,
                                                    delta_f_max,
                                                    phi_curr,
                                                    delta_x_g, f_n, q,
                                                    self.robot.kinematics,
                                                    jerk_opt_weight,
                                                    max_force_mag, beta)

            delta_phi_opt, opt_error, feasible = esm.solve_qp(cost_quadratic_matrices, 
                                                              cost_linear_matrices, 
                                                              constraint_matrices, 
                                                              constraint_vectors, 
                                                              lb, ub, 
                                                              debug_qp=False)


            #This publisher waits for handshake from simulator to say that it knows opt is finished
            # if not robot_name == "Cody":
            #     while not self.got_finish:
            #         self.finished_optimization_pub.publish(Bool(True))
            #     self.got_finish = False

            # update the JEP
            jep[0:m] = (phi_curr + delta_phi_opt).A1

            #ut.get_keystroke('Hit a key to send the delta JEP')

            # warn if JEP goes beyond joint limits
            if self.robot.kinematics.within_joint_limits(jep) == False:
                rospy.logwarn('Outside joint limits. They will be clamped later...')

            if not robot_name == "Cody":
                self.controller_running_pub.publish(Bool(False))

            raise EqPtGen(stop, (jep.tolist(), time_step*1.5))

        except EqPtGen as e:
            # return string with stopping condition and tuple with new JEP and timestep
            return e.stop, e.control_args

###
###
###    END of Delta QP Controller
###
###

    def greedy_eq_gen(self, ep_gen):
        time_step = ep_gen.time_step
        max_jep_mag = ep_gen.max_jep_mag
        proportional_jep_mag_dist = ep_gen.proportional_jep_mag_dist
        stopping_dist_to_goal = ep_gen.stopping_dist_to_goal

        goal_pos = ep_gen.goal_pos
        planar = ep_gen.planar
        ignore_wrist_joints = ep_gen.ignore_wrist_joints
        m = self.robot.kinematics.n_jts
        if ignore_wrist_joints and m == 7:
            m = 4

        try:
            q = self.robot.get_joint_angles()
            # position and rotation of the hand 
            p, r = self.robot.kinematics.FK(q)
            # Jacobian for left/right arm, joint angles, and position on arm
            J_all = self.robot.kinematics.Jacobian(q, p)
            J = J_all[0:3, 0:m]

            # compute and log error distance between hand position goal and current hand position
            err = goal_pos - p
            dist_goal_2D = np.linalg.norm(err[0:2,:])
            dist_goal_3D = np.linalg.norm(err)

            vel_hand = ep_gen.goal_velocity_for_hand
            v = self.goal_motion_for_hand_advait(p, goal_pos, vel_hand * time_step)

            # delta JEP = pseudo-inverse of Jacobian * err
            #d_jep = np.linalg.pinv(J) * (goal_pos - p)
            d_jep = np.linalg.pinv(J) * v

            d_jep_mag = np.linalg.norm(d_jep)
            d_jep_mag_cutoff = min(d_jep_mag, max_jep_mag)

            # for most distances updates jep by a fixed magnitude, unless close, which does proportional
            d_jep_scaled = d_jep / d_jep_mag * d_jep_mag_cutoff
            # get the current JEP (jointspace equilibrium point)
            jep = np.array(self.robot.get_ep())
            # increment current JEP
            jep[0:m] = jep[0:m] + d_jep_scaled.A1

            ######## STOPPING CONDITIONS FOLLOW THIS LINE ########
            stop = ''

            # if close enough to goal declare success!
            if planar:
                dist_goal = dist_goal_2D
            else:
                dist_goal = dist_goal_3D

            if dist_goal < stopping_dist_to_goal:
                stop = 'Reached'
            
            # clamp JEP to be within joint limits
            jep = self.robot.kinematics.clamp_to_joint_limits(jep)

            raise EqPtGen(stop, (jep.tolist(), time_step*1.5))

        except EqPtGen as e:
            # return string with stopping condition and tuple with new JEP and timestep
            return e.stop, e.control_args

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
    def greedy_to_goal(self, goal_pos, control_param_dict, timeout,
                       eq_gen_type, logging_name, monitor_param_dict):

        controller_name = 'reach_to_location'
        self.announce_controller_start(controller_name, goal_pos,
                                       logging_name,
                                       monitor_param_dict)

        control_param_dict['goal_pos'] = goal_pos
        time_step = control_param_dict['time_step']

        if eq_gen_type == 'mpc_qs_1':
            eq_gen_func = self.delta_qp_jep_gen
        else:
            raise RuntimeError('Unknown eq_gen_type: %s'%eq_gen_type)

        if self.robot.kinematics.n_jts == 7:
            #Cody
            ep_gen = epc.EP_Generator(eq_gen_func, self.robot.set_ep,
                                      self.robot.kinematics.clamp_to_joint_limits)
        else:
            # not clamping the JEPs in software simulation.
            ep_gen = epc.EP_Generator(eq_gen_func, self.robot.set_ep)

        ep_gen.__dict__.update(control_param_dict)

        ep_gen.reaching_in = True
        if eq_gen_type == 'mpc_qs_1':
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
            for i in xrange(10):
                self.reduce_force_running_pub.publish(True)
        if controller == 'reach_to_location':
            for i in xrange(10):
                self.reach_to_location_running_pub.publish(True)
        if controller == 'pull_out_to_location':
            for i in xrange(10):
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
        print "ANNOUNCING CONTROLLER STOP IS GETTING CALLED"
        if controller == 'reduce_force':
            for i in xrange(10):
                self.reduce_force_running_pub.publish(False)
        if controller == 'reach_to_location':
            for i in xrange(10):
                self.reach_to_location_running_pub.publish(False)
        if controller == 'pull_out_to_location':
            for i in xrange(10):
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
        
    def world_to_torso(self, p):
        trans, rot = self.current_torso_pose()
        p_torso = rot.T * (p - trans)
        return p_torso

