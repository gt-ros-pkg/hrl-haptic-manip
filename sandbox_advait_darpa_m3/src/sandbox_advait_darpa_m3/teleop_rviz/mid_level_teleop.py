
import sys, os
import numpy as np, math
import copy

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy
import mid_level_control.switch_among_controllers_node as sacn
import hrl_cody_arms.cody_arm_client as cac
import sandbox_advait_darpa_m3.cody.cody_guarded_move as cgm
import interactive_marker_util as imu

import hrl_lib.transforms as tr
import hrl_lib.util as ut

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Quaternion, PointStamped

def rviz_goal_cb(msg):
    global im_pos_world, sac, im_rot_world
    if msg.header.frame_id != '/torso_lift_link':
        rospy.logerr('Incorrect frame_id: %s'%(msg.header.frame_id))
        return
    im_pos_torso = np.matrix([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]).T
    im_pos_world = sac.epc_skin.torso_to_world(im_pos_torso)

    im_rot_world = sac.epc_skin.torso_to_world(im_pos_torso)
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    o_torso = tr.quaternion_to_matrix(q) 
    im_rot_world = sac.epc_skin.torso_to_world_rotation(o_torso)

def rviz_wp_cb(msg):
    if msg.header.frame_id != '/torso_lift_link':
        rospy.logerr('Incorrect frame_id: %s'%(msg.header.frame_id))
        return

    pose_to_published_wp(msg.pose)

    ###HACK to plot where we think key is for user
    # if False:
    #     # #right
    #     key_pos = [0.552, -0.0990, -0.167]
    #     # #left
    #     # key_pos = [0.611, -0.0218, -0.120]
    #     # #top
    #     # key_pos = [0.564, -0.0389, -0.0824]
    #     # #bottom
    #     # key_pos = [0.601, -0.0488, -0.176]

    #     key_loc = Marker()
    #     key_loc.header.frame_id = '/torso_lift_link'
    #     offset = [0.01, -0.02, 0.0]
    #     key_loc.pose.position.x = key_pos[0]
    #     key_loc.pose.position.y = key_pos[1]
    #     key_loc.pose.position.z = key_pos[2]
    #     key_loc.type = Marker.CUBE
    #     key_loc.scale.x = 0.03
    #     key_loc.scale.y = 0.03
    #     key_loc.scale.z = 0.03
    #     key_loc.color.r = 0.0
    #     key_loc.color.g = 1.0
    #     key_loc.color.b = 0.0
    #     key_loc.color.a = 0.8
    #     key_loc_pub.publish(key_loc)


def pose_to_published_wp(pose):
    global sac, way_point_world, wp_goal_tf_frame, way_point_orientation_world
    p_torso = np.matrix([pose.position.x, pose.position.y, pose.position.z]).T
    p_world = sac.epc_skin.torso_to_world(p_torso)
    way_point_world = p_world

    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    o_torso = tr.quaternion_to_matrix(q) 
    o_world = sac.epc_skin.torso_to_world_rotation(o_torso)
    way_point_orientation_world = o_world

    sac.epc_skin.publish_way_point(p_world, wp_goal_tf_frame)

def rviz_delta_goal_pos_cb(msg):
    global local_pos_goal

    q = robot.get_joint_angles()
    pos, rot = robot.kinematics.FK(q)
    quat = tr.matrix_to_quaternion(rot)
    delta_cur = np.matrix([msg.x, msg.y, msg.z]).reshape(3,1)

    #initializing goal in local frame if first callback
    if local_pos_goal == None:
        local_pos_goal = pos

    #checking for flipped sign in command
    cur_err = local_pos_goal - pos
    neg_ind = np.where(np.multiply(cur_err, delta_cur) < 0)
    local_pos_goal[neg_ind] = pos[neg_ind]

    #allowing commands to accumulate and saturate if necessary
    if np.linalg.norm(local_pos_goal-pos) < 0.20:
        local_pos_goal = local_pos_goal + delta_cur

    ps = Pose()
    ps.position.x = local_pos_goal[0,0]
    ps.position.y = local_pos_goal[1,0]
    ps.position.z = local_pos_goal[2,0]
    ps.orientation.x = quat[0]
    ps.orientation.y = quat[1]
    ps.orientation.z = quat[2]
    ps.orientation.w = quat[3]

    pose_to_published_wp(ps)

def rviz_delta_goal_rot_cb(msg):
    global local_pos_goal

    q = robot.get_joint_angles()
    pos, rot_ee = robot.kinematics.FK(q)
    quat = tr.matrix_to_quaternion(rot_ee)
    rot_delta = tr.quaternion_to_matrix([msg.x, msg.y, msg.z, msg.w])

    #initializing goal in local frame if first callback
    if local_pos_goal == None:
        local_pos_goal = pos

    # rotate with respect to gripper####################
    if False: 
        ##can get rid of these two lines with quaternion operation instead#########
        delta_rot = tr.quaternion_to_matrix([msg.x, msg.y, msg.z, msg.w])
        goal_rot = rot_ee*delta_rot
        ##can get rid of these two lines with quaternion operation instead#########
        goal_quat = tr.matrix_to_quaternion(goal_rot)
    else:
        #rotate with respect to person's frame
        rot_goal = rot_ee.T*rot_delta*rot_ee
        goal_quat = tr.matrix_to_quaternion(rot_goal)

    ps = Pose()
    ps.position.x = local_pos_goal[0,0]
    ps.position.y = local_pos_goal[1,0]
    ps.position.z = local_pos_goal[2,0]
    ps.orientation.x = goal_quat[0]
    ps.orientation.y = goal_quat[1]
    ps.orientation.z = goal_quat[2]
    ps.orientation.w = goal_quat[3]

    pose_to_published_wp(ps)

def dashboard_cb(msg):
    global sac, im_pos_world, goal_world, way_point_world, wp_goal_tf_frame, way_point_orientation_world, goal_orientation_world, im_rot_world

    if msg.data == 'set_goal':
        if im_pos_world == None:
            rospy.logwarn('Goal not set yet.')
            return
        goal_world = copy.copy(im_pos_world)
        goal_orientation_world = copy.copy(im_rot_world)
        sac.epc_skin.publish_goal(goal_world, wp_goal_tf_frame)

    elif msg.data == 'pull_out':
        sac.pull_out('pull_out_1')

    elif msg.data == 'reduce_force':
        sac.reduce_force('reduce_force_standalone')

    elif msg.data == 'go_to_way_point' or msg.data == 'orient_to_way_point':
        if msg.data == 'go_to_way_point':
            ignore_orientation=True
        if msg.data == 'orient_to_way_point':
            ignore_orientation=False

        t0 = rospy.get_time()
        sac.epc_skin.publish_way_point(way_point_world, '/world')
        wp = sac.epc_skin.world_to_torso(way_point_world)
        wp_rot = sac.epc_skin.world_to_torso_rotation(way_point_orientation_world)

        r1 = wp_rot
        r2 = copy.copy(wp_rot)
        r2[:,1:3] = r2[:,1:3] * -1 # rotate about x-axis by 180 deg.

        q = sac.epc_skin.robot.get_joint_angles()
        _, r = sac.epc_skin.robot.kinematics.FK(q)
        quat = tr.matrix_to_quaternion(r)
        quat1 = tr.matrix_to_quaternion(r1)
        quat2 = tr.matrix_to_quaternion(r2)

        ang1 = ut.quat_angle(quat, quat1)
        ang2 = ut.quat_angle(quat, quat2)

        if ang1 < ang2:
            wp_rot = r1
            wp_quat = quat1
        else:
            wp_rot = r2
            wp_quat = quat2

        #--------------- publish blue goal marker -----------
        if ignore_orientation==False:
            ps = PointStamped()
            ps.header.frame_id = '/torso_lift_link'

            r_offset = tr.quaternion_to_matrix(wp_quat)

            if robot_name == "pr2":
                offset = np.matrix([-0.17, 0., 0.]).T
                o_torso = r_offset * offset

                ps.point.x = wp[0,0] + o_torso[0,0]
                ps.point.y = wp[1,0] + o_torso[1,0]
                ps.point.z = wp[2,0] + o_torso[2,0]

                cur_gripper_cmd = MarkerArray()
                cur_gripper_cmd = imu.make_pr2_gripper_marker(ps,
                                                              [0.2, 0.2, 0.6, 1.0],
                                                              wp_quat,
                                                              marker_array=cur_gripper_cmd,
                                                              mesh_id_start = 0,
                                                              ns = "commanded_gripper")
            elif robot_name == "cody":
                offset = np.matrix([0.0, 0., 0.08]).T
                o_torso = r_offset * offset

                ps.point.x = wp[0,0] + o_torso[0,0]
                ps.point.y = wp[1,0] + o_torso[1,0]
                ps.point.z = wp[2,0] + o_torso[2,0]

                cur_gripper_cmd = MarkerArray()
                cur_gripper_cmd = imu.make_cody_ee_marker(ps,
                                                          [0.2, 0.2, 0.6, 1.0],
                                                          wp_quat,
                                                          marker_array=cur_gripper_cmd,
                                                          mesh_id_start = 0,
                                                          ns = "commanded_gripper")

            cur_gripper_cmd_pub.publish(cur_gripper_cmd)

        #stopping_dist = 0.02
        stopping_dist = 0.
     
        result = sac.reach_greedy(wp, wp_rot, retry_if_head_on=False,
                                  eq_gen_type = eq_gen_type, stopping_dist = stopping_dist,
                                  stopping_angle = math.radians(15),
                                  attempt_name = 'wp_reach_from_single_reach_in_pos',
                                  ignore_orientation = ignore_orientation)
        if result[0] == "high motor temp":
            sac.reduce_force('reduce_force_standalone')

        t1 = rospy.get_time()
        print 'Time to reach:', t1-t0

    elif msg.data == 'reach_to_goal':
        t0 = rospy.get_time()
        sac.epc_skin.publish_goal(goal_world, '/world')
        goal = sac.epc_skin.world_to_torso(goal_world)
        goal_rot = sac.epc_skin.world_to_torso_rotation(goal_orientation_world)

        sac.reach_greedy(goal, goal_rot, retry_if_head_on=False,
                         eq_gen_type = eq_gen_type, stopping_dist = 0.02,
                         stopping_angle = math.radians(10),
                         attempt_name = 'reach_from_single_reach_in_pos')
        t1 = rospy.get_time()
        print 'Time to reach:', t1-t0

    # elif msg.data == 'go_darpa_goals':
    #     # goal 1
    #     goal = [[ 0.62, -0.05, -0.07247985]]
    #     ## goal 2
    #     #goal = [[ 0.65, 0.25, -0.07247985]]
    #     start = [ 0.34548882,  0.165215,   -0.074273  ]
    #     start_jep = [-0.35932401402271485, 0.15694695732293326, -0.2590862425043608, 2.1096761875700154, -0.065588576836634466, 0.091004001130182532, -0.025897287922689666]
    #     ignore_orientation=False
    #     trial_num = 0

    #     for g in goal:
    #         stopping_dist = 0.02
    #         sac.epc_skin.go_jep(start_jep)
    #         rospy.sleep(1.0)
    #         print 'start_jep', start_jep

    #         result = sac.reach_greedy(np.matrix(g).T,  np.eye(3), retry_if_head_on=False,
    #                                   eq_gen_type = eq_gen_type, stopping_dist = stopping_dist,
    #                                   stopping_angle = math.radians(15),
    #                                   attempt_name = 'go_to_goal_trial_'+str(trial_num),
    #                                   ignore_orientation = ignore_orientation)

    #         raw_input('Done with Reach. Hit ENTER to pull out')
    #         result = sac.reach_greedy(np.matrix(start).T,  np.eye(3), retry_if_head_on=False,
    #                                   eq_gen_type = eq_gen_type, stopping_dist = stopping_dist,
    #                                   stopping_angle = math.radians(15),
    #                                   attempt_name =  'go_to_start_after_trial_'+str(trial_num),
    #                                   ignore_orientation = ignore_orientation)
    #         if result[0] != 'Reached':
    #             raw_input("I FAILED ... MOVE MY ARM PLEASE!!! ... then press Enter")
    #         trial_num = trial_num + 1


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--ignore_skin', '--is', action='store_true',
                 dest='ignore_skin', help='ignore feedback from skin')
    p.add_option('--use_orientation', '--uo', action='store_true',
                 dest='use_orientation', help='try to go to commanded orientation in addition to position')

    p.add_option('--pr2', action='store_true', dest='pr2',
                 help='mid level for the pr2')

    p.add_option('--cody', action='store_true', dest='cody',
                 help='mid level for cody')
    p.add_option('--use_wrist_joints', action='store_true',
                 dest='use_wrist_joints',
                 help='use wrist joints (Cody)')
    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)

    p.add_option('--meka_sensor', action='store_true', dest='meka_sensor',
                 help='use Meka forearm sensor with Cody')
    p.add_option('--fabric_sensor', action='store_true', dest='fabric_sensor',
                 help='use HRL fabric sensor with Cody')
    p.add_option('--hil', action='store_true', dest='hil',
                 help='hardware-in-loop simulation with Cody')

    p.add_option('--sim_3link_with_hand', action='store_true', dest='sim3_with_hand',
                 help='software simulation')
    p.add_option('--sim_3link', action='store_true', dest='sim3',
                 help='software simulation')
    p.add_option('--start_test', action='store_true', dest='start_test',
                 help='makes cody move to specified jep if true')
    p.add_option('--eq_gen_type', action='store', dest='eq_gen_type',
                 type='string', default= None, 
                 help='type of controller, options are mpc_qs_1 and qs_mpc_fast')


    opt, args = p.parse_args()

    global sac, goal_world, way_point_world, im_pos_world, wp_goal_tf_frame, way_point_orientation_world, goal_orientation_world, robot_name

    if opt.cody:
        if opt.meka_sensor:
            skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_ft']
        elif opt.fabric_sensor:
            skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_wrist']
        elif opt.hil:
            skin_topic_list = ['/skin/contacts']
        else:
            raise RuntimeError('Missing command line argument for the testbed')
        wp_goal_tf_frame = '/world'
    elif opt.pr2:
        skin_topic_list = ['/skin/contacts_forearm',
                           '/skin/contacts_upperarm',
                           '/skin/contacts_gripper_right_link',
                           '/skin/contacts_gripper_left_link',
                           '/skin/contacts_gripper_palm',
                           '/skin/contacts_fingertip_right',
                           '/skin/contacts_fingertip_left']
        wp_goal_tf_frame = '/torso_lift_link'
    else:
        skin_topic_list = ['/skin/contacts']
        wp_goal_tf_frame = '/world'

    rospy.init_node('mid_level_teleop')

    # for visualizing the current active orientation command for the gripper
    cur_gripper_cmd_pub = rospy.Publisher('/current_gripper_cmd/visualization_marker_array', MarkerArray)

    # for visualizing the estimated key location when reaching in pipe
    #key_loc_pub = rospy.Publisher('/key_location', Marker)

    if opt.cody:
        robot_name = "cody"
        import hrl_cody_arms.cody_arm_client as cac
        import sandbox_advait_darpa_m3.cody.cody_guarded_move as cgm

        if opt.arm == None:
            rospy.logerr('Need to specify --arm_to_use.\nExiting...')
            sys.exit()

        if opt.use_wrist_joints:
            robot = cac.CodyArmClient_7DOF(opt.arm)
        else:
            robot = cac.CodyArmClient_4DOF(opt.arm)

        if opt.arm == 'r':
            max_lim = np.radians([ 120.00, 122.15, 0., 144., 122.,  45.,  45.])
            min_lim = np.radians([ -47.61,  -20., -77.5,   0., -80., -45., -45.])
        elif opt.arm == 'l':
            # decreased wrist joint angles from 45deg to 30deg for
            # fabric sensor -- Advait, July 15, 2012
            max_lim = np.radians([ 120.00,   20.,  77.5, 144.,   80., 30.,  30.])
            min_lim = np.radians([ -47.61, -122.15, 0.,   0., -122., -30., -30.])
        else:
            rospy.logerr('Unknown arm.\nExiting...')
            sys.exit()

        robot.kinematics.joint_lim_dict['max'] = max_lim
        robot.kinematics.joint_lim_dict['min'] = min_lim

        if opt.hil or opt.fabric_sensor:
            robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.16]).T) # tube
        if opt.meka_sensor:
            #robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.01]).T) # stub
            robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.04]).T) # stub with mini45

        scl = cgm.Cody_SkinClient(skin_topic_list)
        epcon = cgm.MobileSkinEPC_Cody(robot, scl)

        while robot.get_ep() == None:
            rospy.sleep(0.1)
        if opt.start_test:
            raw_input('going to set jep for start of test ... hit Enter to proceed')
            robot.set_ep([-0.6860, -0.2269, 0.01985, 2.338, 0.2282, 0.1035, 0.1136])
            #robot.set_ep([-0.4343, -1.173, 0.4214, 2.393, 0.3575, -0.1585, -0.4414])
            raw_input('is cody finished? ... hit Enter to proceed')
        jep_start = robot.get_ep()

    elif opt.pr2:
        robot_name = "pr2"
        roslib.load_manifest('hrl_pr2_arms')
        import hrl_pr2_arms.pr2_arm_darpa_m3 as padm
        import sandbox_advait_darpa_m3.pr2.pr2_guarded_move as pgm
        import hrl_haptic_controllers_darpa_m3.epc_skin as es

        if opt.arm == None:
            rospy.logerr('Need to specify --arm_to_use.\nExiting...')
            sys.exit()

        robot = padm.PR2Arm(opt.arm)

        # artificially increasing the robot's estimates of the gains
        # at the wrist joints, forearm roll and shoulder roll.
        #robot.kp[2] = 50. # shoulder roll
        #robot.kp[4] = 50. # forearm roll
        #robot.kp[5] = 50. # wrist flex
        #robot.kp[6] = 50. # wrist roll

        # shoulder roll
        robot.kinematics.joint_lim_dict['max'][2] = math.radians(-5)
        robot.kinematics.joint_lim_dict['min'][2] = math.radians(-150)
        # wrist flex
        robot.kinematics.joint_lim_dict['min'][5] = math.radians(-80)
        # wrist roll
        robot.kinematics.joint_lim_dict['max'][6] = math.radians(150)
        robot.kinematics.joint_lim_dict['min'][6] = math.radians(-150)

        scl = pgm.PR2_SkinClient(skin_topic_list)
        epcon = es.Skin_EPC(robot, scl)

        while robot.get_joint_angles() == None:
            rospy.sleep(0.1)

        q = robot.get_joint_angles()
        robot.set_ep(q)
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


    if opt.eq_gen_type == None:
        eq_gen_type = 'mpc_qs_1'        
    else:
        eq_gen_type = opt.eq_gen_type
        #eq_gen_type = 

    sac = sacn.SwitchAmongControllers(epcon)

    if opt.sim3 or opt.sim3_with_hand:
        im_pos_world = np.matrix(rospy.get_param('/m3/software_testbed/goal')).T
        goal_world = copy.copy(im_pos_world)
        sac.epc_skin.publish_goal(goal_world, '/world')
    else:
        im_pos_world = None
    
    goal_orientation_world = None

    way_point_world = None
    way_point_orientation_world = None

    local_pos_goal = None

    if opt.sim3 or opt.sim3_with_hand:
        sac.set_goal = sac.set_goal_software_sim

        static_contact_stiffness_estimate = 1000.
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
                'allowable_contact_force': 5.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_skin': False,
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
                'allowable_contact_force': 5.,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_skin': opt.ignore_skin,
            }

        small_motions_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 150.,
            }

    elif opt.cody:
        sac.set_goal = sac.set_goal_optitrak

        #static_contact_stiffness_estimate = 1000.
        static_contact_stiffness_estimate = 500.
        estimate_contact_stiffness = False
        only_use_normal_force_sensing = True
        max_delta_force_mag = 10.
        jerk_opt_weight = 0.00001
        save_state = False

        if opt.use_orientation:
            orientation_weight = 4.
        else:
            orientation_weight = 0.

        # ok speed
        goal_velocity_for_hand = 0.2
        ee_motion_threshold = 0.005
        #ee_motion_threshold = 0.00   #causes it not to stop when it gets stuck

        # slow.
        #goal_velocity_for_hand = 0.1
        ##ee_motion_threshold = 0.002

        # very slow.
        #goal_velocity_for_hand = 0.05
        #ee_motion_threshold = 0.002

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
                'ignore_skin': opt.ignore_skin,
                'only_use_normal_force_sensing': only_use_normal_force_sensing,
                'max_delta_force_mag': max_delta_force_mag,
                'jerk_opt_weight': jerk_opt_weight,
                'orientation_weight': orientation_weight,
                'save_state': save_state,
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
                'ignore_skin': False,
                'only_use_normal_force_sensing': only_use_normal_force_sensing,
                'max_delta_force_mag': max_delta_force_mag,
                'jerk_opt_weight': jerk_opt_weight,
                'save_state': save_state,
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
                'ignore_skin': opt.ignore_skin,
            }

        small_motions_monitor_param_dict = {
                'ee_motion_threshold': ee_motion_threshold,
                'stopping_force': 15.,
            }

    elif opt.pr2:
        sac.set_goal = sac.set_goal_optitrak

        #static_contact_stiffness_estimate = 1000.
        #static_contact_stiffness_estimate = 5000.
        static_contact_stiffness_estimate = 200.
        estimate_contact_stiffness = False
        only_use_normal_force_sensing = True
        max_delta_force_mag = 10.
        if opt.use_orientation:
            orientation_weight = 4.
        else:
            orientation_weight = 0.
        jerk_opt_weight = 0.00001
        save_state = False

        # fast shelf speed
        goal_velocity_for_hand = 0.3
        ee_motion_threshold = 0.000

#        # ok shelf speed
#        goal_velocity_for_hand = 0.2
#        ee_motion_threshold = 0.000

#        # body speed
#        goal_velocity_for_hand = 0.1
#        ee_motion_threshold = 0.000

        time_step = 0.01

        #---------------------------
        # greedy to goal
        #---------------------------
        greedy_to_goal_control_param_dict = {
            # for fast mpc
                'goal_state_weights' : np.diag((1, 1, 1, 1./1000., 1./1000., 1./1000.)).tolist(),
                #'goal_state_weights' : np.diag((1, 1, 1, 0., 0., 0.)).tolist(),
                'task_index':[0,1,2,3,4,5],
                #'task_index':[3,4,5],
                #'task_index':[0,1,2],
                'stopping_dist_to_goal': 0.01,
                'stopping_ang_to_goal': 0.0349,
                #'goal_orientation': [ 0.,  0.70710678,  0.,  0.70710678],
                #'goal_orientation': [ 0.,  0.,  0.69613524,  0.71791067],  #rotation pi/2 around z-axis
                #'goal_orientation': [ 0., -0.69613524, 0.,  0.71791067],  #rotation -pi/2 around y-axis 
                'goal_orientation': [0, 0, 0, 1],
            # for delta_qp
                #'task_index':[0,1,2,3,4,5],
                #'stopping_dist_to_goal': 0.01,
                #'stopping_ang_to_goal': 0.0349,
                #'goal_orientation': [0, 0, 0, 1],
                'jep_start': jep_start,
                'time_step': time_step,
                'planar': False,
                'goal_velocity_for_hand': goal_velocity_for_hand,
                #'kill_controller_force': 10.,
                'allowable_contact_force': 3.0,
                #'allowable_contact_force': 8.0,
                #'allowable_contact_force': 10.,
                #'allowable_contact_force': 1.5,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_skin': opt.ignore_skin,
                'only_use_normal_force_sensing': only_use_normal_force_sensing,
                'max_delta_force_mag': max_delta_force_mag,
                'jerk_opt_weight': jerk_opt_weight,
                'orientation_weight': orientation_weight,
                'save_state': save_state,
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
                'allowable_contact_force': 3.0,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_skin': False,
                'only_use_normal_force_sensing': only_use_normal_force_sensing,
                'max_delta_force_mag': max_delta_force_mag,
                'jerk_opt_weight': jerk_opt_weight,
                'save_state': save_state,
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
                'allowable_contact_force': 3.0,
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
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
    rospy.Subscriber('/teleop_rviz/command/goal_position', PoseStamped, rviz_goal_cb)
    rospy.Subscriber('/teleop_rviz/command/way_point_pose', PoseStamped, rviz_wp_cb)
    rospy.Subscriber('/teleop_rviz/command/delta_goal_position', Vector3, rviz_delta_goal_pos_cb)
    rospy.Subscriber('/teleop_rviz/command/delta_goal_orientation', Quaternion, rviz_delta_goal_rot_cb)

    rospy.spin()


