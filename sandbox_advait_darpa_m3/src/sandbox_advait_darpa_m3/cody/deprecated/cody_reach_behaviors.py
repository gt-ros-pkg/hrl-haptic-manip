
import numpy as np, math
import copy

import roslib; roslib.load_manifest('darpa_m3')
import rospy
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_lib.viz as hv
from visualization_msgs.msg import Marker

import tf

import hrl_cody_arms.cody_arm_client as cac
import haptic_controllers.epc_skin as es
import sandbox_advait.move_segway as ms

from sandbox_advait.guarded_move_config import threshold_dict as controllers_thresh_dict
import sandbox_advait.cody.cody_guarded_move as cgm

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from hrl_srvs.srv import Bool_None, None_Bool
from hrl_msgs.msg import FloatArrayBare


#epc = None
goal_world = None
goal_world_2 = None
marker_world = None
start_torso = None
jep_start = None
q_start = None
waypoint_world = None
reach_in_pose_idx = 0

# center and radius of fragile cylinder in the torso coord frame.
fragile_cylinder_center = np.matrix([0., 0., 0.]).T
fragile_cylinder_radius = 0.01


#-------- ROS stuff -----------
def marker_cb(msg):
    global marker_world
    t = msg.transform.translation
    if use_left_arm:
        marker_world = np.matrix([t.x, t.y, t.z+0.1]).T
    else:
        marker_world = np.matrix([t.x, t.y, t.z+0.05]).T

def waypoint_cb(msg):
    global waypoint_world
    #ps = tf_lstnr.transformPoint('/world', msg) # not working :(
    wp_torso = np.matrix([msg.point.x, msg.point.y, msg.point.z]).T
    waypoint_world = epc.torso_to_world(wp_torso)
    waypoint_world[2,0] = goal_world[2,0]

def publish_epc_start():
    epc_stop_pub.publish(Bool(False))
    #tm_clear_log(False) # preserving the contact memory.
    rospy.sleep(0.1)


#-------- behaviors -------------

def change_fragile_cylinder():
    global fragile_cylinder_radius, fragile_cylinder_center
    fragile_cylinder_center = epc.world_to_torso(marker_world)
    fragile_cylinder_radius = 0.07
    r = fragile_cylinder_radius
    cyl_marker = hv.single_marker(marker_world,
                                  np.matrix([0,0,0,1]).T, 'cylinder',
                                  '/world', scale=[r*2,r*2,1.],
                                  duration=0)
    cyl_marker.header.stamp = rospy.Time.now()
    fragile_cyl_rviz_pub.publish(cyl_marker)

    c = fragile_cylinder_center.A1
    r = fragile_cylinder_radius
    fragile_cyl_params_pub.publish(FloatArrayBare([c[0], c[1], c[2], r]))

def set_goal_2():
    global goal_world_2, marker_world
    goal_world_2 = copy.copy(marker_world)
    goal = epc.world_to_torso(goal_world_2)
    epc.publish_goal(goal, '/torso_lift_link')

def set_goal():
    global goal_world, marker_world
    goal_world = copy.copy(marker_world)
    goal = epc.world_to_torso(goal_world)
    epc.publish_goal(goal, '/torso_lift_link')

def set_start():
    global start_torso, jep_start, q_start
    q_start = robot.get_joint_angles()
    jep_start = robot.get_ep()
    start_torso = robot.kinematics.FK(q_start)[0]

# retry_if_head_on - try and determine if the collision that caused
# the controller to stop was a head on collision. if it was, then pick
# a side and retry
def reach_opt_greedy(goal_world, retry_if_head_on, eq_gen_type,
                     timeout = np.inf, retry_direction='right',
                     stopping_dist = None):
    # resetting the start location. when I pull out, I pull out to
    # this location only
    set_start()
    goal = epc.world_to_torso(goal_world)
    keep_going = True
    did_i_retry = False

    gtg_dict = controllers_thresh_dict[testbed]['greedy_to_goal']

    if stopping_dist != None:
        gtg_dict['stopping_dist_to_goal'] = stopping_dist

    small_motion_eq_gen = 'no_optimize_jep'

    rospy.loginfo('goal: '+str(goal.A1))
    head_on_count = 0

    while keep_going:
        res = epc.greedy_to_goal(goal, gtg_dict, timeout = timeout,
                                 eq_gen_type = eq_gen_type)

        # check for head-on collision. pick a side in case of
        # head on and go greedy again?
        #head_on = is_contact_head_on()
        #print 'is_contact_head_on:', head_on
        head_on = False
        rospy.logwarn('Disabled check for head on collisions')

        keep_going = retry_if_head_on and head_on

        if head_on_count == 1:
            keep_going = False

        if keep_going:
            did_i_retry = True
            head_on_count += 1
            # quick and dirty. Please check all implications of this.
            publish_epc_start()
            reduce_force()

            tm_pause(True)
            q = epc.robot.get_joint_angles()
            ee = epc.robot.kinematics.FK(q)[0]
            wrist = epc.robot.kinematics.FK(q, 6)[0]
            ee_direc = ee-wrist
            ee_direc[2,0] = 0. # forcing XY plane.
            ee_direc = ee_direc / np.linalg.norm(ee_direc)

            goal_back = ee  - ee_direc * 0.05
            rospy.loginfo('Going to move back a bit.')
            #ut.get_keystroke('Hit a key to move back a bit.')
            res = epc.greedy_to_goal(goal_back, gtg_dict, timeout=3.,
                                     eq_gen_type = small_motion_eq_gen)
            tm_pause(False)

            # pick a side.
            if retry_direction == 'right':
                z = np.array([0.,0.,1.])
            else:
                z = np.array([0.,0.,-1.])

            side_vec = np.matrix(np.cross(ee_direc.A1, z)).T
            goal_side = goal_back + side_vec * 0.1
            rospy.loginfo('Going to move to the side.')
            #ut.get_keystroke('Hit a key to move to the side.')
            res = epc.greedy_to_goal(goal_side, gtg_dict, timeout=3.,
                                     eq_gen_type = small_motion_eq_gen)

            rospy.loginfo('Going to move to the goal.')
            #ut.get_keystroke('Hit a key to move to the goal.')

    res = list(res)
    res.append(did_i_retry)
    print 'res:', res[0]
    return res

def pull_out():
    global jep_start, q_start, start_torso
    # pull out using the delta QP controller (always)
    pull_out_eq_gen_type = 'optimize_vmc'

    jep_start_copy = copy.copy(jep_start)
    gtg_dict = controllers_thresh_dict[testbed]['greedy_to_goal']

    elbow_start = robot.kinematics.FK(q_start, 3)[0]
    start_world = epc.torso_to_world(start_torso)
    timeout = 60*3. # 3 minutes
    res = epc.pull_out_elbow(elbow_start, gtg_dict, timeout=timeout,
                             eq_gen_type = pull_out_eq_gen_type)

    rospy.loginfo('res: '+res[0])

    if res[0] == 'Reached':
        start_world = epc.torso_to_world(start_torso)
        #res = reach_opt_greedy(start_world, False, eq_gen_type)
        res = reach_opt_greedy(start_world, False, 'no_optimize_jep',
                               stopping_dist = 0.04)
        if res[0] == 'Reached':
            epc.go_jep(jep_start_copy, speed=math.radians(40))
            #rospy.sleep(0.1)
            set_start()

    return res

def next_reach_in_pose():
    global reach_in_pose_idx
    locs = reach_in_locs_dict['locs']
    if locs.shape[1] == reach_in_pose_idx:
        rospy.loginfo('Reached the last location')
        reach_in_pose_idx = 0
        return
    p = np.matrix(locs[:,reach_in_pose_idx]).reshape((3,1))
    a = reach_in_locs_dict['angles'][reach_in_pose_idx]
    r = tr.rotZ(a)
    epc.move_torso_to(p, r)
    reach_in_pose_idx += 1

def fixed_base_multiple_reach(delta_y_reach_in_l):
    reach_in_in_torso_frame = copy.copy(start_torso)
    retry_direction = 'right'
    keep_going = True
    reached_goal = False
    i = 0
    while keep_going:
        dy = delta_y_reach_in_l[i]
        # compute next reach in location (with base fixed)
        # hack: 1cm is the stopping distance for greedy to goal
        dy  = dy + 0.02 * np.sign(dy)
        reach_in_loc = reach_in_in_torso_frame + np.matrix([0., dy, 0.]).T

        # move arm to reach in location
        rospy.loginfo('Going to reach_in_location')
        loc_world = epc.torso_to_world(reach_in_loc)
        res = reach_opt_greedy(loc_world, retry_if_head_on=False, 
                               eq_gen_type = 'no_optimize_jep',
                               timeout = 3., stopping_dist=0.02)

        #ut.get_keystroke('Hit a key to start reaching in')
        publish_epc_start()

        # now reach in.
        rospy.loginfo('now reaching in')
        #res = reach_opt_greedy(goal_world, retry_if_head_on=False,
        res = reach_opt_greedy(goal_world, retry_if_head_on=True,
                               eq_gen_type = eq_gen_type,
                               retry_direction = retry_direction,
                               stopping_dist = 0.025)
        did_reach_opt_retry = res[-1]
        print 'res:', res[0]

        # do not retry if reached the goal.
        if res[0] == 'Reached':
            keep_going = False
            reached_goal = True

        #ut.get_keystroke('Hit a key to continue')
        rospy.loginfo('Done with reach. Now going to pull out')

        publish_epc_start()
        rospy.loginfo('reducing force')
        #reduce_force() # in case force too high, reduce_force is mostly harmless.

        #hack, disabling task monitor while pulling out.
        tm_pause(True)
        rospy.loginfo('pulling out')
        res = pull_out()
        if res[0] != 'Reached':
            publish_epc_start()
            rospy.sleep(0.5)
            res = pull_out()
        tm_pause(False)
        if res[0] != 'Reached':
            #rospy.loginfo('Pull out failed.')
            #rospy.loginfo('Please pull out manually.')
            #ut.get_keystroke('hit a key to continue')
            #publish_epc_start()
            #rospy.sleep(0.5)
            tm_pause(True)
            raise UserWarning('Pull out failed. Exiting reach_in_multiple.')

        if did_reach_opt_retry == True and retry_direction == 'right':
            # flip reaching direction from the same location.
            retry_direction = 'left'
        else:
            # did not make head on collision, so try next reach in
            # location.
            i += 1

        if i == len(delta_y_reach_in_l):
            keep_going = False

    return reached_goal

def reach_in_multiple():
    flip_retry_direction = True

    # different reach in locations with the arm fixed.
    if not flip_retry_direction:
        delta_y_reach_in_l = [0, -0.08, 0.08]
    else:
        # hacky way to change the direction of retrying if head on
        # collision.
        delta_y_reach_in_l = [0]

    # different places to move the mobile base to.
    #delta_y_mobile_base = [0, -0.1, 0.1, -0.2, 0.2]
    delta_y_mobile_base = [0, -0.2, 0.2, -0.1, 0.1, -0.3, 0.3]
    #delta_y_mobile_base = [0]
    initial_loc, initial_rot = epc.current_torso_pose()
    reach_in_in_torso_frame = copy.copy(start_torso)
    home_jep = copy.copy(jep_start)

    reach_in_base_distance = 0.2
    dist_moved_before_hit = 0.

    try:
        for dy_base in delta_y_mobile_base:
            loc_world = epc.torso_to_world(reach_in_in_torso_frame)
            res = reach_opt_greedy(loc_world, retry_if_head_on=False,
                                   eq_gen_type = 'no_optimize_jep')
            if res[0] != 'Reached':
                raise UserWarning('Unable to go to home location')

            epc.go_jep(home_jep)
            # first move the base back by the amount that
            # move_base_till_hit had moved it in. this is a bit dangerous
            epc.base.back(dist_moved_before_hit, blocking=True)

            # now go to the new torso location.
            torso_loc = initial_loc + np.matrix([0., dy_base, 0.]).T
            epc.move_torso_to(torso_loc, initial_rot)

            # in preparation for moving base until contact.
            safety_for_base_motion = 0.06
            reach_in_loc = reach_in_in_torso_frame + np.matrix([safety_for_base_motion, 0., 0.]).T
            loc_world = epc.torso_to_world(reach_in_loc)
            res = reach_opt_greedy(loc_world, retry_if_head_on=False,
                                   eq_gen_type = 'no_optimize_jep')

            publish_epc_start()
            tm_pause(True)
            res = epc.move_base_till_hit(0.2)
            tm_pause(False)
            new_torso_pos = epc.current_torso_pose()[0]
            dist_moved_before_hit = abs(new_torso_pos[0,0] - initial_loc[0,0])

            epc.go_jep(home_jep)
            if fixed_base_multiple_reach(delta_y_reach_in_l):
                # reached the goal location.
                break

    except UserWarning as e:
        rospy.logwarn(str(e.args))

def reduce_force():
    tm_pause(True)
    res = epc.reduce_force()
    
    #for i in range(5):
    #    if res[0] == 'stop_command_over_ROS':
    #        res = epc.reduce_force()
    #        #ut.get_keystroke('hit a key to do another reduce_force')

    rospy.loginfo('res: '+res[0])
    tm_pause(False)
    return res

#--------- interfacing with dashboard ---------
def skin_behavior_cb(msg):
    global goal_world

    if msg.data == 'set_goal':
        set_goal()
    if msg.data == 'set_goal_2':
        set_goal_2()
    elif msg.data == 'set_start':
        set_start()
    elif msg.data == 'next_reach_in_pose':
        next_reach_in_pose()
    elif msg.data == 'reduce_force':
        reduce_force()
    elif msg.data == 'mark_fragile':
        change_fragile_cylinder()
    else:
        tm_pause(True)
        rospy.sleep(0.02)
        #tm_clear_log(False) # preserving contact memory
        tm_pause(False)
        if msg.data == 'pull_out':
            pull_out()
        elif msg.data == 'opt_jep_goal':
            t0 = rospy.get_time()
            goal = epc.world_to_torso(goal_world)
            epc.publish_goal(goal, '/torso_lift_link')
            #reach_opt_greedy(goal_world, retry_if_head_on = False,
            reach_opt_greedy(goal_world, retry_if_head_on = True,
                             eq_gen_type = eq_gen_type, stopping_dist = 0.020)
            t1 = rospy.get_time()
            print 'Time to reach:', t1-t0
        elif msg.data == 'opt_jep_waypoint':
            reach_opt_greedy(waypoint_world, retry_if_head_on = False,
                             eq_gen_type = eq_gen_type)
        elif msg.data == 'try_multiple':
            goal = epc.world_to_torso(goal_world)
            epc.publish_goal(goal, '/torso_lift_link')

            # move the segway
            reach_in_multiple()

            # fixed base reaching in
            if False:
                home_jep = copy.copy(jep_start)
                delta_y_reach_in_l = [0, -0.1]
                goal = epc.world_to_torso(goal_world)
                epc.publish_goal(goal, '/torso_lift_link')
                fixed_base_multiple_reach(delta_y_reach_in_l)
                if goal_world_2 != None:
                    epc.go_jep(home_jep) # this is dangerous.
                    goal_world = copy.copy(goal_world_2)
                    goal = epc.world_to_torso(goal_world)
                    epc.publish_goal(goal, '/torso_lift_link')
                    fixed_base_multiple_reach(delta_y_reach_in_l)

#--------- interfacing with the task_monitor ---------

#def is_contact_head_on():
#    return is_contact_head_on_proxy().data


if __name__ == '__main__':
    rospy.init_node('cody_guarded_move')
    #tf_lstnr = tf.TransformListener()

    #use_left_arm = False
    use_left_arm = True

    rospy.Subscriber('/epc_skin/command/behavior', String,
                      skin_behavior_cb)
    rospy.Subscriber('/fixed_obstacle_trackable/pose',
                     TransformStamped, marker_cb)
    rospy.Subscriber('/epc_skin/command/waypoint', PointStamped,
                     waypoint_cb)

    skin_topic = '/skin/contacts'

    if use_left_arm:
        robot = cac.CodyArmClient('l')
    else:
        robot = cac.CodyArmClient('r')

    robot.kinematics.set_tooltip(np.matrix([0.,0.,-0.16]).T) # tube

    alpha = [0.1939, 0.35, 0.748, 0.7272, 0.75]
    robot.set_impedance_scale(alpha)

    scl = cgm.Cody_SkinClient(skin_topic)
    epc = cgm.Mobile_Skin_EPC(robot, scl)
    reach_in_locs_dict = ut.load_pickle('reach_in_points.pkl')

    while robot.get_ep() == None:
        rospy.sleep(0.1)

    epc_stop_pub = rospy.Publisher('/epc/stop', Bool)
    fragile_cyl_rviz_pub = rospy.Publisher('/epc_skin/viz/fragile_cylinder', Marker)
    fragile_cyl_params_pub = rospy.Publisher('/epc_skin/fragile_cylinder_params', FloatArrayBare)

    srv_nm = '/task_monitor/pause'
    rospy.loginfo('Waiting for '+ srv_nm + ' service')
    rospy.wait_for_service(srv_nm)
    rospy.loginfo('Done waiting.')
    tm_pause = rospy.ServiceProxy(srv_nm, Bool_None)


    #srv_nm = '/task_monitor/clear_log'
    #rospy.loginfo('Waiting for '+ srv_nm + ' service')
    #rospy.wait_for_service(srv_nm)
    #rospy.loginfo('Done waiting.')
    #tm_clear_log = rospy.ServiceProxy(srv_nm, Bool_None)

    #srv_nm = '/task_monitor/is_contact_head_on'
    #rospy.loginfo('Waiting for '+ srv_nm + ' service')
    #rospy.wait_for_service(srv_nm)
    #rospy.loginfo('Done waiting.')
    #is_contact_head_on_proxy = rospy.ServiceProxy(srv_nm, None_Bool)

    testbed = 'hardware_in_loop_cody'

    #eq_gen_type = 'no_optimize_jep'
    eq_gen_type = 'optimize_vmc'
    #eq_gen_type = 'simple'

    rospy.spin()


#    import openravepy as orpy
#
#    env = orpy.Environment()
#    #env.SetViewer('qtcoin')
#    #env.Load('../hardware_in_loop_simulation/cody_skin_env.xml')
#    env.Load('../../hardware_in_loop_simulation/openrave_ik.xml')
#    cody_openrave = env.GetRobots()[0]
#    cody_openrave.SetTransform(np.diag([0.,0.,0.,1.]))
#
#    ik_type = orpy.IkParameterization.Type.Translation3D
#    ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(cody_openrave,
#                                                                      iktype=ik_type)
#    ikmodel.load()
#    filter_options = orpy.IkFilterOptions.CheckEnvCollisions | orpy.IkFilterOptions.IgnoreSelfCollisions
#    #filter_options = orpy.IkFilterOptions.IgnoreSelfCollisions
#
#    or_dict = {}
#    or_dict['env'] = env
#    or_dict['robot'] = cody_openrave
#    or_dict['ik_type'] = ik_type
#    or_dict['ik_model'] = ikmodel
#    or_dict['ik_param_func'] = orpy.IkParameterization
#    or_dict['filter_options'] = filter_options
#    or_dict['bodies'] = env.GetBodies()[1:]



#def reach_in_with_switching():
#    i = 10
#    while i>0:
#        res = reach_opt_greedy(goal_world)
#        print 'res:', res[0]
#        if 'Reached' not in res[0]: # i.e. stuck
#            #compute_wp_pub.publish(Empty())
#            publish_epc_start()
#            #reduce_force()
#
#            # temporary hack, disabling task monitor while pulling out.
#            tm_pause(True)
#            pull_out()
#            tm_pause(False)
#
#            #publish_epc_start()
#            #reduce_force()
#            #reach_opt_greedy(waypoint_world)
#
#            #publish_epc_start()
#            #reduce_force()
#        else:
#            break
#        #i = i-1
#        i = 0














