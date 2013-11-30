
import sys, os
import numpy as np, math

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy
import mid_level_control.switch_among_controllers_node as sacn
import hrl_cody_arms.cody_arm_client as cac
import sandbox_advait_darpa_m3.cody.cody_guarded_move as cgm

from std_msgs.msg import String


def dashboard_cb(msg):
    if msg.data == 'set_goal':
        sac.set_goal()
    elif msg.data == 'set_start':
        sac.push_start()
    elif msg.data == 'pull_out':
        sac.pull_out('pull_out_1')
    elif msg.data == 'reduce_force':
        sac.reduce_force('reduce_force_standalone')
    elif msg.data == 'reach_to_goal':
        t0 = rospy.get_time()
        sac.epc_skin.publish_goal(sac.goal_location_world, '/world')
        goal = sac.epc_skin.world_to_torso(sac.goal_location_world)

        sac.reach_greedy(goal, retry_if_head_on=False,
                         eq_gen_type = eq_gen_type, stopping_dist = 0.040,
                         attempt_name = 'reach_from_single_reach_in_pos')
        t1 = rospy.get_time()
        print 'Time to reach:', t1-t0



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--use_mpc', action='store_true', dest='mpc',
                 help='run the model predictive controller')
    p.add_option('--use_baseline', action='store_true', dest='baseline',
                 help='run the baseline controller')

    opt, args = p.parse_args()

    skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_ft']

    rospy.init_node('cinderblock_demo')

    arm = 'r'

    robot = cac.CodyArmClient(arm)

    if arm == 'r':
        max_lim = np.radians([ 120.00, 122.15, 0., 144., 122.,  45.,  45.])
        min_lim = np.radians([ -47.61,  -20., -77.5,   0., -80., -45., -45.])
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

    if opt.baseline:
        eq_gen_type = 'simple'
    elif opt.mpc:
        eq_gen_type = 'optimize_vmc'
    else:
        rospy.logerr('Please specify the controller to use.\nExiting...')
        sys.exit()

    sac = sacn.SwitchAmongControllers(epcon)

    sac.set_goal = sac.set_goal_optitrak

    static_contact_stiffness_estimate = 1000.
    estimate_contact_stiffness = False

    ignore_wrist_joints =True

    # move arm slowly
    goal_velocity_for_hand = 0.05
    ee_motion_threshold = 0.002

    time_step = 0.01

    #---------------------------
    # greedy to goal
    #---------------------------
    greedy_to_goal_control_param_dict = {
        # common to all.
            'jep_start': jep_start,
            'time_step': time_step,
            'goal_velocity_for_hand': goal_velocity_for_hand,
            'planar': False,
        # for simple
            'max_jep_mag': math.radians(0.4),
            'proportional_jep_mag_dist': 0.05,
        # for delta_qp
            'allowable_contact_force': 5.,
            'k': static_contact_stiffness_estimate,
            'estimate_contact_stiffness': estimate_contact_stiffness,
            'ignore_wrist_joints': ignore_wrist_joints,
        }

    greedy_to_goal_monitor_param_dict = {
            'ee_motion_threshold': ee_motion_threshold,
            'stopping_force': 15.
        }

    #---------------------------
    # reduce force
    #---------------------------
    reduce_force_control_param_dict = {}
    reduce_force_monitor_param_dict = {}

    #---------------------------
    # pull out
    #---------------------------
    pull_out_control_param_dict = {
        # common to all.
            'time_step': time_step,
            'goal_velocity_for_hand': goal_velocity_for_hand,
            'planar': False,
        # for simple
            'max_jep_mag': math.radians(0.4),
            'proportional_jep_mag_dist': 0.05,
        # delta_qp
            'allowable_contact_force': 5.,
            'k': static_contact_stiffness_estimate,
            'estimate_contact_stiffness': estimate_contact_stiffness,
            'ignore_wrist_joints': ignore_wrist_joints,
        }

    pull_out_monitor_param_dict = {
            'ee_motion_threshold': ee_motion_threshold,
            'stopping_force': 18.
        }

    #---------------------------
    # small motions
    #---------------------------
    small_motions_control_param_dict = {
        # common to all.
            'time_step': time_step,
            'goal_velocity_for_hand': goal_velocity_for_hand,
            'planar': False,
        # for simple greedy
            'max_jep_mag': math.radians(0.4),
            'proportional_jep_mag_dist': 0.05,
        # for delta_qp
            'allowable_contact_force': 5.,
            'k': static_contact_stiffness_estimate,
            'estimate_contact_stiffness': estimate_contact_stiffness,
            'ignore_wrist_joints': ignore_wrist_joints,
        }

    small_motions_monitor_param_dict = {
            'ee_motion_threshold': ee_motion_threshold,
            'stopping_force': 15.,
        }

    sac.greedy_to_goal_control_param_dict = greedy_to_goal_control_param_dict
    sac.greedy_to_goal_monitor_param_dict = greedy_to_goal_monitor_param_dict

    sac.pull_out_control_param_dict = pull_out_control_param_dict
    sac.pull_out_monitor_param_dict = pull_out_monitor_param_dict

    sac.small_motions_control_param_dict = small_motions_control_param_dict
    sac.small_motions_monitor_param_dict = small_motions_monitor_param_dict

    rospy.Subscriber('/epc_skin/command/behavior', String,
                     dashboard_cb)

    rospy.spin()



