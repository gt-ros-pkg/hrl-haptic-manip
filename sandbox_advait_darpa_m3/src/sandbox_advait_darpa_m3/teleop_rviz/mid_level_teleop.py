
import sys, os
import numpy as np, math
import copy

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy
import mid_level_control.switch_among_controllers_node as sacn
import hrl_cody_arms.cody_arm_client as cac
import sandbox_advait_darpa_m3.cody.cody_guarded_move as cgm

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped

def rviz_goal_cb(msg):
    global im_pos_world, sac
    if msg.header.frame_id != '/torso_lift_link':
        rospy.logerr('Incorrect frame_id: %s'%(msg.header.frame_id))
        return
    im_pos_torso = np.matrix([msg.point.x, msg.point.y, msg.point.z]).T
    im_pos_world = sac.epc_skin.torso_to_world(im_pos_torso)

def rviz_wp_cb(msg):
    global sac, way_point_world
    if msg.header.frame_id != '/torso_lift_link':
        rospy.logerr('Incorrect frame_id: %s'%(msg.header.frame_id))
        return
    p_torso = np.matrix([msg.point.x, msg.point.y, msg.point.z]).T
    p_world = sac.epc_skin.torso_to_world(p_torso)
    way_point_world = p_world
    sac.epc_skin.publish_way_point(p_world, '/world')

def dashboard_cb(msg):
    global sac, im_pos_world, goal_world, way_point_world

    if msg.data == 'set_goal':
        if im_pos_world == None:
            rospy.logwarn('Goal not set yet.')
            return
        goal_world = copy.copy(im_pos_world)
        sac.epc_skin.publish_goal(goal_world, '/world')

    elif msg.data == 'pull_out':
        sac.pull_out('pull_out_1')

    elif msg.data == 'reduce_force':
        sac.reduce_force('reduce_force_standalone')

    elif msg.data == 'reach_to_way_point':
        t0 = rospy.get_time()
        sac.epc_skin.publish_way_point(way_point_world, '/world')
        wp = sac.epc_skin.world_to_torso(way_point_world)

        sac.reach_greedy(wp, retry_if_head_on=False,
                         eq_gen_type = eq_gen_type, stopping_dist = 0.02,
                         attempt_name = 'wp_reach_from_single_reach_in_pos')
        t1 = rospy.get_time()
        print 'Time to reach:', t1-t0

    elif msg.data == 'reach_to_goal':
        t0 = rospy.get_time()
        sac.epc_skin.publish_goal(goal_world, '/world')
        goal = sac.epc_skin.world_to_torso(goal_world)

        sac.reach_greedy(goal, retry_if_head_on=False,
                         eq_gen_type = eq_gen_type, stopping_dist = 0.02,
                         attempt_name = 'reach_from_single_reach_in_pos')
        t1 = rospy.get_time()
        print 'Time to reach:', t1-t0



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--ignore_skin', '--is', action='store_true',
                 dest='ignore_skin', help='ignore feedback from skin')

    p.add_option('--cody', action='store_true', dest='cody',
                 help='task monitoring for cody')
    p.add_option('--hil', action='store_true', dest='hil',
                 help='hardware-in-loop simulation with Cody')
    p.add_option('--use_wrist_joints', action='store_true',
                 dest='use_wrist_joints',
                 help='use wrist joints (Cody)')
    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)

    p.add_option('--sim_3link_with_hand', action='store_true', dest='sim3_with_hand',
                 help='software simulation')
    p.add_option('--sim_3link', action='store_true', dest='sim3',
                 help='software simulation')
    p.add_option('--sim_6link', action='store_true', dest='sim6',
                 help='software simulation')

    opt, args = p.parse_args()

    global sac, goal_world, way_point_world, im_pos_world

    if opt.cody and (not opt.hil):
        skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_ft']
    else:
        skin_topic_list = ['/skin/contacts']

    rospy.init_node('mid_level_teleop')

    if opt.cody:
        import hrl_cody_arms.cody_arm_client as cac
        import sandbox_advait_darpa_m3.cody.cody_guarded_move as cgm

        if opt.arm == None:
            rospy.logerr('Need to specify --arm_to_use.\nExiting...')
            sys.exit()

        robot = cac.CodyArmClient(opt.arm)

        if opt.arm == 'r':
            max_lim = np.radians([ 120.00, 122.15, 0., 144., 122.,  45.,  45.])
            min_lim = np.radians([ -47.61,  -20., -77.5,   0., -80., -45., -45.])
        elif opt.arm == 'l':
            max_lim = np.radians([ 120.00,   20.,  77.5, 144.,   80.,  45.,  45.])
            min_lim = np.radians([ -47.61, -122.15, 0.,   0., -122., -45., -45.])
        else:
            rospy.logerr('Unknown arm.\nExiting...')
            sys.exit()

        robot.kinematics.joint_lim_dict['max'] = max_lim
        robot.kinematics.joint_lim_dict['min'] = min_lim

        #robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.16]).T) # tube
        #robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.01]).T) # stub
        robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.04]).T) # stub with mini45
        scl = cgm.Cody_SkinClient(skin_topic_list)
        epcon = cgm.MobileSkinEPC_Cody(robot, scl)

        while robot.get_ep() == None:
            rospy.sleep(0.1)
        jep_start = robot.get_ep()

    else:
        import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa
        import hrl_software_simulation_darpa_m3.ode_sim_guarded_move as sgm

        if opt.sim3:
            import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
            jep_start = np.radians([-100.0, 110, 110])

        elif opt.sim3_with_hand:
            import hrl_common_code_darpa_m3.robot_config.three_link_with_hand as d_robot
            jep_start = None

        elif opt.sim6:
            import hrl_common_code_darpa_m3.robot_config.six_link_planar as d_robot
            jep_start = None

        ode_arm = gsa.ODESimArm(d_robot)
        scl = sgm.ode_SkinClient(skin_topic_list)

        epcon = sgm.MobileSkinEPC_ODE(ode_arm, scl)
        rospy.sleep(1.)

        if jep_start == None:
            jep_start = ode_arm.get_ep()

        ode_arm.set_ep(jep_start)


    eq_gen_type = 'mpc_qs_1'

    sac = sacn.SwitchAmongControllers(epcon)

    if opt.sim3 or opt.sim6 or opt.sim3_with_hand:
        im_pos_world = np.matrix(rospy.get_param('/m3/software_testbed/goal')).T
        goal_world = copy.copy(im_pos_world)
        sac.epc_skin.publish_goal(goal_world, '/world')
    else:
        im_pos_world = None

    way_point_world = None

    if opt.sim3 or opt.sim3_with_hand:
        sac.set_goal = sac.set_goal_software_sim

        #static_contact_stiffness_estimate = 10000.
        static_contact_stiffness_estimate = 100.
        estimate_contact_stiffness = True
        goal_velocity_for_hand = 0.05
        ee_motion_threshold = 0.004
        time_step = 0.01

        #---------------------------
        # greedy to goal
        #---------------------------
        greedy_to_goal_control_param_dict = {
            # for delta_qp
                'jep_start': jep_start,
                'time_step': time_step,
                'planar': True,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                'allowable_contact_force': 10.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': False,
                'ignore_skin': opt.ignore_skin,
            }

        greedy_to_goal_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 100.
            }

        #---------------------------
        # reduce force
        #---------------------------
        reduce_force_control_param_dict = {
                'planar': True,
                'ignore_wrist_joints': False,
            }
        reduce_force_monitor_param_dict = {}

        #---------------------------
        # pull out
        #---------------------------
        pull_out_control_param_dict = {
            # delta_qp
                'time_step': time_step,
                'planar': True,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                'allowable_contact_force': 10.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': False,
                'ignore_skin': opt.ignore_skin,
            }

        pull_out_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 250.
            }

        #---------------------------
        # small motions
        #---------------------------
        small_motions_control_param_dict = {
            # for delta_qp only
                'time_step': 0.01 ,
                'planar': True,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                'allowable_contact_force': 10.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': False,
                'ignore_skin': opt.ignore_skin,
            }

        small_motions_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 150.,
            }

    elif opt.sim6:
        sac.set_goal = sac.set_goal_software_sim

        #static_contact_stiffness_estimate = 10000.
        static_contact_stiffness_estimate = 1000.
        estimate_contact_stiffness = True
        goal_velocity_for_hand = 0.03
        ee_motion_threshold = 0.001
        time_step = 0.01

        #---------------------------
        # greedy to goal
        #---------------------------
        greedy_to_goal_control_param_dict = {
            # for delta_qp only
                'jep_start': jep_start,
                'time_step': time_step,
                'planar': True,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                'allowable_contact_force': 10.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': False,
                'ignore_skin': opt.ignore_skin,
            }

        greedy_to_goal_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 100.
            }

        #---------------------------
        # reduce force
        #---------------------------
        reduce_force_control_param_dict = {
                'planar': True,
                'ignore_wrist_joints': False,
            }
        reduce_force_monitor_param_dict = {}

        #---------------------------
        # pull out
        #---------------------------
        pull_out_control_param_dict = {
            # delta_qp
                'time_step': time_step,
                'planar': True,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                'allowable_contact_force': 10.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': False,
                'ignore_skin': opt.ignore_skin,
            }

        pull_out_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 250.
            }

        #---------------------------
        # small motions
        #---------------------------
        small_motions_control_param_dict = {
            # for delta_qp
                'time_step': 0.01 ,
                'planar': True,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                'allowable_contact_force': 10.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': False,
                'ignore_skin': opt.ignore_skin,
            }

        small_motions_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 150.,
            }

    elif opt.cody:
        sac.set_goal = sac.set_goal_optitrak

        static_contact_stiffness_estimate = 5000.
        estimate_contact_stiffness = False

        # start with low estimate of stiffness and turn on the
        # stiffness estimation.
        #static_contact_stiffness_estimate = 50.
        #estimate_contact_stiffness = True

        ignore_wrist_joints = not opt.use_wrist_joints

        # ok speed
        goal_velocity_for_hand = 0.2
        #ee_motion_threshold = 0.005

        # very slow.
        #goal_velocity_for_hand = 0.05
        #ee_motion_threshold = 0.002
        ee_motion_threshold = 0.000

        # fast.
        #goal_velocity_for_hand = 0.3
        #ee_motion_threshold = 0.01
        
        time_step = 0.01

        #---------------------------
        # greedy to goal
        #---------------------------
        greedy_to_goal_control_param_dict = {
            # for delta_qp
                'jep_start': jep_start,
                'time_step': time_step,
                'planar': False,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                'allowable_contact_force': 5.,
                #'allowable_contact_force': 10.,
                #'allowable_contact_force': 1.5,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': ignore_wrist_joints,
                'ignore_skin': opt.ignore_skin,
            }

        greedy_to_goal_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 15.
            }

        #---------------------------
        # reduce force
        #---------------------------
        reduce_force_control_param_dict = {
                'planar': False,
                'ignore_wrist_joints': ignore_wrist_joints,
            }
        reduce_force_monitor_param_dict = {}

        #---------------------------
        # pull out
        #---------------------------
        pull_out_control_param_dict = {
            # delta_qp
                'time_step': time_step,
                'planar': False,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                'allowable_contact_force': 5.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': ignore_wrist_joints,
                'ignore_skin': opt.ignore_skin,
            }

        pull_out_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 18.
            }

        #---------------------------
        # small motions
        #---------------------------
        small_motions_control_param_dict = {
            # for delta_qp
                'time_step': time_step,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                'planar': False,
                'allowable_contact_force': 5.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': ignore_wrist_joints,
                'ignore_skin': opt.ignore_skin,
            }

        small_motions_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 15.,
            }

    sac.greedy_to_goal_control_param_dict = greedy_to_goal_control_param_dict
    sac.greedy_to_goal_monitor_param_dict = greedy_to_goal_monitor_param_dict

    sac.reduce_force_control_param_dict = reduce_force_control_param_dict
    sac.reduce_force_monitor_param_dict = reduce_force_monitor_param_dict

    sac.pull_out_control_param_dict = pull_out_control_param_dict
    sac.pull_out_monitor_param_dict = pull_out_monitor_param_dict

    sac.small_motions_control_param_dict = small_motions_control_param_dict
    sac.small_motions_monitor_param_dict = small_motions_monitor_param_dict

    rospy.Subscriber('/epc_skin/command/behavior', String, dashboard_cb)
    rospy.Subscriber('/teleop_rviz/command/goal_position', PointStamped, rviz_goal_cb)
    rospy.Subscriber('/teleop_rviz/command/way_point_position', PointStamped, rviz_wp_cb)

    rospy.spin()



