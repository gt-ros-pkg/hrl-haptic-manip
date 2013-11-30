
import numpy as np, math
import copy

import roslib; roslib.load_manifest('darpa_m3')
import rospy
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_lib.geometry as hg

import tf

import haptic_controllers.epc_skin as es
import software_simulation.ode_sim_arms as osa

from sandbox_advait.guarded_move_config import threshold_dict as controllers_thresh_dict
import sandbox_advait.software_simulation.ode_sim_guarded_move as sgm
import sandbox_advait.epc_skin_advait as esa

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Bool, Empty
from hrl_srvs.srv import Bool_None

#epc = None
goal_world = None
waypoint_world = None
start_torso = None
jep_start = None

def set_goal():
    global goal_world
    goal_world = np.matrix(rospy.get_param('m3/software_testbed/goal')).T
    epc.publish_goal(goal_world, '/world')

# there is a subtle hack here. currently (July 1, 2011) simulation
# does not have a /torso_lift_link TF frame.
# However the PointStamped ROS topic has frameid = /torso_lift_link
def goal_cb(msg):
    global goal_world
    #ps = tf_lstnr.transformPoint('/world', msg)
    ps = msg
    goal_world = np.matrix([ps.point.x, ps.point.y, ps.point.z]).T
    epc.publish_goal_marker(goal_world, '/world')

# there is a subtle hack here. currently (July 1, 2011) simulation
# does not have a /torso_lift_link TF frame.
# However the PointStamped ROS topic has frameid = /torso_lift_link
def waypoint_cb(msg):
    global waypoint_world
    #ps = tf_lstnr.transformPoint('/world', msg)
    ps = msg
    waypoint_world = np.matrix([ps.point.x, ps.point.y, ps.point.z]).T

def set_start():
    global start_torso, jep_start
    q_start = ode_arm.get_joint_angles()
    ode_arm.set_ep(q_start)
    jep_start = ode_arm.get_ep()
    start_torso, _ = ode_arm.kinematics.FK(q_start)

def reach_opt_greedy(goal_world):
    goal = goal_world
    gtg_dict = controllers_thresh_dict[testbed]['greedy_to_goal']
    jt_index_dict = {0:0, 1:1, 2:2, -1: ()}
    active_joints = [0,1,2]

    res = epc.greedy_to_goal(goal, active_joints, gtg_dict,
                             jt_index_dict, timeout = np.inf,
                             eq_gen_type = eq_gen_type)
    rospy.loginfo('res: '+res[0])
    return res

# pull out to reach back in towards the waypoint
def pull_out_to_reach_for_waypoint():
    global waypoint_world
    q = ode_arm.get_joint_angles()
    p_l = ode_arm.kinematics.arm_config_to_points_list(q)
    pt_arm = hg.project_point_on_curve(waypoint_world, p_l)
    d = hg.distance_along_curve(pt_arm, p_l)
    pt_go = hg.get_point_along_curve(p_l, d-0.1)
    pull_out(pt_go)

def pull_out(pt):
#    res = reach_opt_greedy(start_torso)
    gtg_dict = controllers_thresh_dict[testbed]['greedy_to_goal']
    jt_index_dict = {0:0, 1:1, 2:2, -1: ()}
    active_joints = [0,1,2]

    res = epc.pull_out_along_arm_shapely(pt, active_joints,
                                 gtg_dict, jt_index_dict,
                                 timeout = np.inf,
                                 eq_gen_type = eq_gen_type)
    rospy.loginfo('res: '+res[0])
    return res

def reduce_force():
    #pause_pub.publish(True)
    tm_pause(True)
    jt_index_dict = {0:0, 1:1, 2:2, -1: ()}
    active_joints = [0,1,2]
    res = epc.reduce_force(jt_index_dict, active_joints)
    rospy.loginfo('res: '+res[0])
    tm_pause(False)
    #pause_pub.publish(False)
    return res

def reach_in_with_switching():
    i = 10
    while i>0:
        res = reach_opt_greedy(goal_world)
        print 'res:', res[0]
        if 'Reached' not in res[0]: # i.e. stuck
            #raw_input('ENTER to compute new waypoint')
            compute_wp_pub.publish(Empty())
            publish_epc_start()

            #raw_input('ENTER to reduce force')
            reduce_force()
            #raw_input('ENTER to pull_out_to_reach_for_waypoint')
            pull_out_to_reach_for_waypoint()

            publish_epc_start()
            #raw_input('ENTER to reduce force')
            reduce_force()
            #raw_input('ENTER to reach for waypoint')
            reach_opt_greedy(waypoint_world)

            publish_epc_start()
            #raw_input('ENTER to reduce force')
            reduce_force()
        else:
            break
        i = i-1

def publish_epc_start():
    epc_stop_pub.publish(Bool(False))
    tm_clear_log(False) # preserving the contact memory.
    rospy.sleep(0.1)


# callback for the command that decides which behavior to execute.
# this will come from the dashboard (in case of human teleoperation),
# or the task monitor (supervisory controller?)
def skin_behavior_cb(msg):
    if msg.data == 'set_goal':
        set_goal()
    elif msg.data == 'set_start':
        set_start()
    elif msg.data == 'next_reach_in_pose':
        rospy.logwarn('Unimplemented %s'%(msg.data))
    else:
        reduce_force()
        #pause_pub.publish(True)
        tm_pause(True)
        rospy.sleep(0.02)
        tm_clear_log(False) # preserving the contact memory.
        #pause_pub.publish(False)
        tm_pause(False)
        if msg.data == 'pull_out':
            global start_torso
            res = pull_out(start_torso)
            if res[0] == 'Reached':
                epc.go_jep(jep_start)

        #elif msg.data == 'pull_out_for_waypoint':
        #    pull_out_to_reach_for_waypoint()
        elif msg.data == 'pull_out_and_retry':
            pull_out_and_retry()
        elif msg.data == 'opt_jep_goal':
            reach_opt_greedy(goal_world)
        elif msg.data == 'opt_jep_waypoint':
            reach_opt_greedy(waypoint_world)
        elif msg.data == 'try_multiple':
            reach_in_with_switching()

if __name__ == '__main__':
    rospy.init_node('sim_arm_guarded_move')
    #tf_lstnr = tf.TransformListener()

    rospy.Subscriber('/epc_skin/command/behavior', String,
                      skin_behavior_cb)
    rospy.Subscriber('/epc_skin/command/waypoint', PointStamped,
                     waypoint_cb)
    rospy.Subscriber('/epc_skin/goal', PointStamped, goal_cb)

    skin_topic = '/skin/contacts'
    ode_arm = osa.ODESimArm()
    scl = sgm.ode_SkinClient(skin_topic)
    epc = esa.Skin_EPC_advait(ode_arm, scl)

    #eq_gen_type = 'optimize_vmc'
    eq_gen_type = 'optimize_jep'

    #jep_start = np.radians([-80.0, 57, 157])
    jep_start = np.radians([-80.0, 57, 120])
    ode_arm.set_ep(jep_start)
    #q = ode_arm.get_joint_angles()
    #ode_arm.set_ep(q)
    #reduce_force()

    compute_wp_pub = rospy.Publisher('/task_monitor/command/compute_waypoint', Empty)
    epc_stop_pub = rospy.Publisher('/epc/stop', Bool)
    #pause_pub = rospy.Publisher('/task_monitor/pause', Bool)

    srv_nm = '/task_monitor/pause'
    rospy.loginfo('Waiting for '+ srv_nm + ' service')
    rospy.wait_for_service(srv_nm)
    rospy.loginfo('Done waiting.')
    tm_pause = rospy.ServiceProxy(srv_nm, Bool_None)

    srv_nm = '/task_monitor/clear_log'
    rospy.loginfo('Waiting for '+ srv_nm + ' service')
    rospy.wait_for_service(srv_nm)
    rospy.loginfo('Done waiting.')
    tm_clear_log = rospy.ServiceProxy(srv_nm, Bool_None)

    testbed = 'software_simulation'
    rospy.spin()








#---------- old code ----------------

#def pull_out_and_retry():
#    pull_out_to_reach_for_waypoint()
#    reach_opt_greedy(waypoint_world)
#    wait_for_epc_start()
#    reach_opt_greedy(goal_world)















