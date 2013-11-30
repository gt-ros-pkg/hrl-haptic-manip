#!/usr/bin/python

# this node will perform logging and monitoring for stop conditions
# for each of the different controllers in my set of controllers.

# different controllers could share some logging code, but that is not
# required. For example, the pull out controller might have a higher
# stopping force than the reach in controller.

# whenever a controller starts, it will publish to a topic, which will
# let this log_and_monitor_node know that it must start logging for
# that controller.

# after this node decides that the controller should stop (question:
# can any other node issue an /epc/stop or /epc/pause command? the
# dashboard can. I'm not sure what that would do to this structure,
# but is worth keeping in mind.)
# anyways, once this node decides to stop the current controller, it
# issues a stop command, stops logging, and saves the current log as a
# pickle.

# I need to decide what all should be a service call, and what should
# be ROS messages, for syncing between the actual controller, this
# node, the dashboard etc.

import numpy as np, math
import sys

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')

import rospy
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_lib.circular_buffer as cb

from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyResponse
from hrl_srvs.srv import Bool_None, Bool_NoneResponse

from hrl_msgs.msg import FloatArray, FloatArrayBare
from std_msgs.msg import Bool, Empty
from hrl_haptic_manipulation_in_clutter_srvs.srv import LogAndMonitorInfo, LogAndMonitorInfoResponse


class LoggerAndMonitor():
    def __init__(self, skin_client, robot):
        self.sc = skin_client
        self.robot = robot
        self.pause_tm = False
        self.stop_pub = rospy.Publisher('/epc/stop', Bool)
        self.segway_stop_pub = rospy.Publisher('/hrl/segway/stop', Empty)

        rospy.Service('/epc_skin/wait_for_logger', Empty_srv,
                      self.wait_for_logger)
        rospy.Service('/epc_skin/log_and_monitor_info', LogAndMonitorInfo,
                      self.log_and_monitor_info)

        # buffers that would be common to all the controllers. Each of
        # these buffers will be logged into a pkl and cleared after
        # every run of one of the controllers.

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
        # and all the contact forces correspoinding to that arm
        # configuration
        self.num_contacts_at_time_instant_buf = cb.CircularBuffer(self.hist_size, ())

        n_jts = robot.kinematics.n_jts
        self.jep_buf = cb.CircularBuffer(self.hist_size, (n_jts,))
        self.q_buf = cb.CircularBuffer(self.hist_size, (n_jts,))
        self.qdot_buf = cb.CircularBuffer(self.hist_size, (n_jts,))

        self.time_stamp_buf = cb.CircularBuffer(self.hist_size, ())

        self.ee_gaussian_mn_buf_50 = cb.CircularBuffer(self.hist_size, (3,))
        self.ee_gaussian_cov_buf_50 = cb.CircularBuffer(self.hist_size, (3,3))
        self.mean_motion_buf = cb.CircularBuffer(self.hist_size, ())

        self.disable_stop_commands_flag = False # log data but do not stop.

        # flag to keep track of writing log to disk.
        self.done_writing_to_disk = True 

        # info that will get logged with each run of a local
        # controller.
        self.torso_position = None
        self.torso_rotation = None
        self.local_goal = None
        self.current_controller = ''
        self.logging_name = ''
        self.stopping_force = None
        self.ee_motion_threshold = None

        #--- ROS subscribers.
        # /epc/stop will result in the controller stopping, which in
        # turn will cause the controller to publish a False to the
        # /epc_skin/local_controller/<name>/running topic.

        # this is a trick. I am using the same callback for both the
        # service and the message.
        rospy.Subscriber('/epc/pause', Bool, self.pause_cb)
        rospy.Service('/task_monitor/pause', Bool_None, self.pause_cb)

        # topic for each of the local feedback controllers, to know
        # when the controller started and stopped running.
        rospy.Subscriber('/epc_skin/local_controller/reduce_force/running', Bool,
                         self.reduce_force_controller_cb)
        rospy.Subscriber('/epc_skin/local_controller/reach_to_location/running', Bool,
                         self.reach_to_location_controller_cb)
        rospy.Subscriber('/epc_skin/local_controller/pull_out_to_location/running', Bool,
                         self.pull_out_to_location_controller_cb)
        #rospy.Subscriber('/epc_skin/local_controller/move_base/running', Bool,
        #                 self.move_base_controller_cb)

    #---------------------------------------------------------------
    # synchronization with the controller (over ROS)
    #---------------------------------------------------------------

    def wait_for_logger(self, req):
        while not self.done_writing_to_disk:
            rospy.sleep(0.1)
        return EmptyResponse()

    def log_and_monitor_info(self, req):
        tp = req.torso_pose
        p = tp.position
        self.torso_position = np.matrix([p.x, p.y, p.z]).T

        q = tp.orientation
        self.torso_rotation = tr.quaternion_to_matrix([q.x, q.y, q.z, q.w])
        self.local_goal = np.matrix([req.local_goal.x, req.local_goal.y, req.local_goal.z]).T
        self.logging_name = req.logging_name
        self.stopping_force = req.stopping_force
        self.ee_motion_threshold = req.ee_motion_threshold
        return LogAndMonitorInfoResponse()

    def reduce_force_controller_cb(self, msg):
        if msg.data:
            self.current_controller = 'reduce_force'
        else:
            self.stop_current_controller()

    def reach_to_location_controller_cb(self, msg):
        if msg.data:
            self.current_controller = 'reach_to_location'
        else:
            self.stop_current_controller()

    def pull_out_to_location_controller_cb(self, msg):
        if msg.data:
            self.current_controller = 'pull_out_to_location'
        else:
            self.stop_current_controller()

    def pause_cb(self, req):
        self.pause_tm = req.data
        return Bool_NoneResponse()

    def publish_stop(self):
        self.stop_current_controller()
        self.stop_pub.publish(Bool(True))
        self.segway_stop_pub.publish(Empty())

    #---------------------------------------------------------------
    # some (parameterized) monitoring functions that might be common
    # to the different controllers
    #---------------------------------------------------------------

    def is_force_low(self, stop_force):
        low_force = True
        if len(self.max_force_buf) < 1:
            return True

        curr_max = self.max_force_buf[-1]
        if curr_max > stop_force:
            # force greater than threshold
            low_force = False
            rospy.loginfo('Force is %.1fN, above threshold of %.1fN'%(curr_max, stop_force))
        return low_force

    def is_ee_moving(self, distance_thresh):
        if len(self.mean_motion_buf) > 0 and \
           self.mean_motion_buf[-1] < distance_thresh:
            n = min(len(self.mean_motion_buf), 5)
            rospy.loginfo('Mean is not moving anymore: %s'%(str(self.mean_motion_buf.get_last(n))))
            return False
        return True

    def update_ee_gaussian_buf(self, mn_buf, cov_buf, hist_size):
        if len(self.ee_pos_buf) < hist_size:
            return
        ee_hist = self.ee_pos_buf.get_last(hist_size)
        ee_hist = np.matrix(ee_hist).T
        mn_buf.append(np.mean(ee_hist, 1).A1)
        cov_buf.append(np.cov(ee_hist))

    def update_mean_motion(self, mn_buf, step):
        if len(mn_buf) < step:
            return
        d = np.linalg.norm((mn_buf[-1] - mn_buf[-step])[0:2])
        self.mean_motion_buf.append(d)

    #---------------------------------------------------------------------
    # functions that log and monitor for each of the individual contollers
    #---------------------------------------------------------------------

    def reduce_force_logging_and_monitoring(self):
        # first log, then monitor.
        self.update_common_log()

    def reach_to_location_logging_and_monitoring(self):
        # first log, then monitor.
        self.update_common_log()

        all_ok = True
        all_ok = all_ok and self.is_force_low(self.stopping_force) # simulation
        all_ok = all_ok and self.is_ee_moving(self.ee_motion_threshold)

        if not (all_ok or self.disable_stop_commands_flag):
            self.publish_stop()

    def pull_out_to_location_logging_and_monitoring(self):
        # first log, then monitor.
        self.update_common_log()

        all_ok = True
        #all_ok = all_ok and self.is_force_low(35.) # Cody
        #all_ok = all_ok and self.is_force_low(200.) # simulation.
        all_ok = all_ok and self.is_force_low(self.stopping_force) # simulation.

        if not (all_ok or self.disable_stop_commands_flag):
            self.publish_stop()

    def move_base_logging_and_monitoring(self):
        pass

    def reduce_force_log_dict(self):
        d = {}
        # nothing more than the common logging for now.
        return d

    def reach_to_location_log_dict(self):
        d = {}
        # nothing more than the common logging for now.
        return d

    def pull_out_to_location_log_dict(self):
        d = {}
        # nothing more than the common logging for now.
        return d

    def move_base_log_dict(self):
        d = {}
        # nothing more than the common logging for now.
        return d

    def clear_reduce_force_log(self):
        #  nothing more than common logging.
        return

    def clear_reach_to_location_log(self):
        #  nothing more than common logging.
        return

    def clear_pull_out_to_location_log(self):
        #  nothing more than common logging.
        return

    def clear_move_base_log(self):
        #  nothing more than common logging.
        return


    #-------------------------------------------------------------
    # common functionality. bookkeeping etc.
    #-------------------------------------------------------------

    def update_common_log(self):
        # max force
        #f_mag_l = self.sc.force_magnitude_list()
        f_l, nrml_l, loc_l, jt_l = self.sc.force_normal_loc_joint_list(True)
        
        if f_l != []:
            mat = np.column_stack(f_l)
            f_mag_l = ut.norm(mat).A1.tolist()
            for f_mag, nrml, loc, jt in zip(f_mag_l, nrml_l, loc_l, jt_l):
                self.all_forces_buf.append(f_mag)
                self.all_forces_locs_buf.append(loc.A1)
                self.all_forces_nrmls_buf.append(nrml.A1)
                self.all_forces_jts_buf.append(jt)

            f_max = max(f_mag_l)
        else:
            f_max = 0.

        self.max_force_buf.append(f_max)

        self.num_contacts_at_time_instant_buf.append(len(f_l))

        self.jep_buf.append(self.robot.get_ep())

        q = self.robot.get_joint_angles()
        self.q_buf.append(q)

        self.qdot_buf.append(self.robot.get_joint_velocities())

        ee, _ = self.robot.kinematics.FK(q)
        self.ee_pos_buf.append(ee.A1)

        self.time_stamp_buf.append(rospy.get_time())

        # fit Gaussian to last 50 locations of the end effector.
        self.update_ee_gaussian_buf(self.ee_gaussian_mn_buf_50,
                                    self.ee_gaussian_cov_buf_50, 50)
        # find distance (in 2D) between the means of the Gaussians
        # that are 200 samples apart.
        self.update_mean_motion(self.ee_gaussian_mn_buf_50, step=200)
    
    def log_to_dict(self, controller):
        # common to all the controllers.
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

        d['torso_position'] = self.torso_position
        d['torso_rotation'] = self.torso_rotation
        d['local_goal'] = self.local_goal
        d['controller'] = controller

        if controller == 'reduce_force':
            d2 = self.reduce_force_log_dict()
        elif controller == 'reach_to_location':
            d2 = self.reach_to_location_log_dict()
        elif controller == 'pull_out_to_location':
            d2 = self.pull_out_to_location_log_dict()
        elif controller == 'move_base':
            d2 = self.move_base_log_dict()
        else:
            rospy.logerr('Unknown controller: %s'%controller)

        d.update(d2)
        return d
    
    def save_log(self, d):
        #ut.save_pickle(d, controller+'_log_'+ut.formatted_time()+'.pkl')
        ut.save_pickle(d, self.logging_name+'_log.pkl')

    def clear_log(self, controller):
        # common to all controllers.
        self.max_force_buf.clear()
        self.all_forces_buf.clear()
        self.all_forces_locs_buf.clear()
        self.all_forces_nrmls_buf.clear()
        self.all_forces_jts_buf.clear()
        self.num_contacts_at_time_instant_buf.clear()
        self.ee_pos_buf.clear()
        self.jep_buf.clear()
        self.q_buf.clear()
        self.qdot_buf.clear()
        self.ee_gaussian_mn_buf_50.clear()
        self.ee_gaussian_cov_buf_50.clear()
        self.mean_motion_buf.clear()
        self.time_stamp_buf.clear()

        if controller == 'reduce_force':
            self.clear_reduce_force_log()

        if controller == 'reach_to_location':
            self.clear_reach_to_location_log()

        if controller == 'pull_out_to_location':
            self.clear_pull_out_to_location_log()

        if controller == 'move_base':
            d2 = self.clear_move_base_log()

    def stop_current_controller(self):
        if self.current_controller == '':
            #already stopped.
            return
        # save log to disk.
        d = self.log_to_dict(self.current_controller)
        #self.save_log(self.current_controller, d)
        self.save_log(d)
        self.clear_log(self.current_controller)
        self.current_controller = ''
        self.done_writing_to_disk = True

    def disable_stop_commands(self, f):
        self.disable_stop_commands_flag = f

    #---------------------------------------------------------------
    # function that manages control between the logging, monitoring,
    # and saving functions for the individual controllers.
    #---------------------------------------------------------------

    def step(self):
        if self.current_controller == '' or self.pause_tm:
            return
        
        # here some controller is running and hence setting the flag
        # to False. After the controller stops, the function that
        # writes the data to a pkl will set this to True.
        self.done_writing_to_disk = False

        if self.current_controller == 'reduce_force':
            self.reduce_force_logging_and_monitoring()

        if self.current_controller == 'reach_to_location':
            self.reach_to_location_logging_and_monitoring()

        if self.current_controller == 'pull_out_to_location':
            self.pull_out_to_location_logging_and_monitoring()

        if self.current_controller == 'move_base':
            self.move_base_logging_and_monitoring()


def step_cb(data):
    global step_lm_flag
    step_lm_flag = True


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--cody', action='store_true', dest='cody',
                 help='task monitoring for cody')
    p.add_option('--pr2', action='store_true', dest='pr2',
                 help='task monitoring for PR2')

    p.add_option('--meka_sensor', action='store_true', dest='meka_sensor',
                 help='use Meka forearm sensor with Cody')
    p.add_option('--fabric_sensor', action='store_true', dest='fabric_sensor',
                 help='use HRL fabric sensor with Cody')
    p.add_option('--hil', action='store_true', dest='hil',
                 help='hardware-in-loop simulation with Cody')

    p.add_option('--sim3', action='store_true', dest='sim3',
                 help='three link planar (torso, upper arm, forearm)')
    p.add_option('--sim3_with_hand', action='store_true', dest='sim3_with_hand',
                 help='three link planar (upper arm, forearm, hand)')
    p.add_option('--sim6', action='store_true', dest='sim6',
                 help='task monitoring in simulation')
    p.add_option('--disable', action='store_true', dest='disable',
                 help='log the data, but disable sending a stop command.')
    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)
    opt, args = p.parse_args()

    if opt.cody:
        if opt.meka_sensor:
            skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_ft']
        elif opt.fabric_sensor:
            skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_wrist']
        elif opt.hil:
            skin_topic_list = ['/skin/contacts']
        else:
            raise RuntimeError('Missing command line argument for the testbed')
    elif opt.pr2:
        skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_upperarm']
    else:
        skin_topic_list = ['/skin/contacts']

    if opt.cody:
        from sandbox_advait_darpa_m3.cody.cody_guarded_move import Cody_SkinClient
        import hrl_cody_arms.cody_arm_client as cac

        rospy.init_node('cody_log_and_monitor_node')

        if opt.arm == None:
            rospy.logerr('Please specify an arm to use. Exiting...')
            sys.exit()

        scl = Cody_SkinClient(skin_topic_list)

        robot = cac.CodyArmClient(opt.arm)
        if opt.hil or opt.fabric_sensor:
            robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.16]).T) # tube
        if opt.meka_sensor:
            #robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.01]).T) # stub
            robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.04]).T) # stub with mini45
        
        jep_cmd_topic = '/'+opt.arm+'_arm/command/jep'
        # sync logging and task monitoring with new cmds.
        rospy.Subscriber(jep_cmd_topic, FloatArray, step_cb)

    elif opt.pr2:
        from sandbox_advait_darpa_m3.pr2.pr2_guarded_move import PR2_SkinClient
        roslib.load_manifest('hrl_pr2_arms')
        import hrl_pr2_arms.pr2_arm_darpa_m3 as padm
        from pr2_controllers_msgs.msg import JointTrajectoryActionGoal

        rospy.init_node('pr2_log_and_monitor_node')

        if opt.arm == None:
            rospy.logerr('Please specify an arm to use. Exiting...')
            sys.exit()

        scl = PR2_SkinClient(skin_topic_list)
        robot = padm.PR2Arm(opt.arm)

        jep_cmd_topic = '/'+opt.arm+'_arm_controller/joint_trajectory_action/goal'
        # sync logging and task monitoring with new cmds.
        rospy.Subscriber(jep_cmd_topic, JointTrajectoryActionGoal, step_cb)

    else:
        from hrl_software_simulation_darpa_m3.ode_sim_guarded_move import ode_SkinClient
        import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa

        if opt.sim3:
            # it doesn't matter if we have capsule or cuboid, the KDL
            # chain is identical.
            import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
        elif opt.sim3_with_hand:
            import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as d_robot
        elif opt.sim6:
            import hrl_common_code_darpa_m3.robot_config.six_link_planar as d_robot
        else:
            print 'Please specify a testbed for task monitoring.'
            print 'Exiting ...'
            sys.exit()

        rospy.init_node('ode_log_and_monitor_node')

        robot = gsa.ODESimArm(d_robot)
        scl = ode_SkinClient(skin_topic_list)

        jep_cmd_topic = '/sim_arm/command/jep'
        # sync logging and task monitoring with new cmds.
        rospy.Subscriber(jep_cmd_topic, FloatArrayBare, step_cb)
        # for some strange reason we are publishing JEPs, joint angles
        # etc as FloatArrayBare in simulation. (It looks like I did
        # that)


    lm = LoggerAndMonitor(scl, robot)
    lm.disable_stop_commands(opt.disable)

    global step_lm_flag
    step_lm_flag = False

    rt = rospy.Rate(100)
    rospy.loginfo('started log_and_monitor_node')
    while not rospy.is_shutdown():
        if step_lm_flag:
            step_lm_flag = False
            lm.step()
        rt.sleep()




