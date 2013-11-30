
import numpy as np, math
import copy

import roslib; roslib.load_manifest('hrl_haptic_manipulation_in_clutter')
import rospy
import hrl_lib.util as ut

import software_simulation.ode_sim_arms as osa
import software_simulation.draw_scene as ds
from hrl_haptic_manipulation_in_clutter_srvs.srv import StartLog
from hrl_haptic_manipulation_in_clutter_msgs.msg import MechanicalImpedanceParams
import threading

import hrl_haptic_controllers_darpa_m3.epc_skin as es

#from sandbox_marc.guarded_move_config import threshold_dict as controllers_thresh_dict
print "NEED TO UPDATE WHERE THE THRESHOLD DICT COMES FROM and the sim guarded move"
assert(False)

class New_Skin_EPC(es.Skin_EPC):
    def __init__(self, robot, skin_client):
        es.Skin_EPC.__init__(self, robot, skin_client)

k_p = [0, 0, 0]
k_d = [0, 0, 0]
lock = threading.RLock()


def get_impedance(msg):
    lock.acquire()
    global k_p, k_d
    k_p = list(msg.k_p.data)
    k_d = list(msg.k_d.data)
    lock.release()

if __name__ == '__main__':
    
    import optparse
    p = optparse.OptionParser()

    p.add_option('--opt_vmc', action='store_true', dest='opt_vmc',
                 help='delta jep via optimization of VMC-like elements')
    p.add_option('--opt_force', action='store_true', dest='opt_force',
                 help='compute delta_jep by optimizing contact forces and dist to goal')
    p.add_option('--compliant', action='store_true', dest='compliant',
                 help='really compliant robot arm')
    p.add_option('--stiff', action='store_true', dest='stiff',
                 help='really stiff robot arm')
    p.add_option('--torque_vmc', action='store_true', dest='torque_vmc',
                 help='torque controlled through setting jep with vmc-like stuff')
    p.add_option('--batch', action='store_true', dest='batch',
                 help='part of a batch run, with logging etc.')
    p.add_option('--fname', action='store', dest='fname', type='string',
                 help='name of pkl file with controller result.')

    opt, args = p.parse_args()

    rospy.init_node('multi_contact_mover')

    skin_topic = '/skin/contacts'
    ode_arm = osa.ODESimArm()

    scl = osgm.ode_SkinClient(skin_topic)
    epc = New_Skin_EPC(ode_arm, scl)

    r_arm, l_arm = 0, 1
    arm = r_arm
    rospy.sleep(1.)

    jep = ode_arm.get_joint_angles(arm)
    ode_arm.set_jep(arm, jep)

    rospy.sleep(1.)

    goal_pos = np.matrix(rospy.get_param('m3/software_testbed/goal')).T

    print 'goal_pos:', goal_pos.A1

    testbed = 'software_simulation'
    joint_index_dict = {0:0, 1:1, 2:2, -1: ()}
    gtg_dict = controllers_thresh_dict[testbed]['greedy_to_goal']

    k_p_comp = [1, 0.6, 0.3]
    k_d_comp = [1.5, 1, 0.9]

    k_p_stiff = [30, 20, 15]
    k_d_stiff = [15, 10, 8]

    if opt.compliant == True:
        imped_sub = rospy.Subscriber('/sim_arm/joint_impedance', MechanicalImpedanceParams, get_impedance)
        imped_pub = rospy.Publisher('/sim_arm/command/joint_impedance', MechanicalImpedanceParams)
        impedance = MechanicalImpedanceParams()
        impedance.k_p.data = k_p_comp
        impedance.k_d.data = k_d_comp
        while (k_p != k_p_comp):
            imped_pub.publish(impedance)
            rospy.sleep(0.01)
    
    if opt.stiff == True:
        imped_sub = rospy.Subscriber('/sim_arm/joint_impedance', MechanicalImpedanceParams, get_impedance)
        imped_pub = rospy.Publisher('/sim_arm/command/joint_impedance', MechanicalImpedanceParams)
        impedance = MechanicalImpedanceParams()
        impedance.k_p.data = k_p_stiff
        impedance.k_d.data = k_d_stiff
        while (k_p != k_p_stiff):
            imped_pub.publish(impedance)
            rospy.sleep(0.01)


    if opt.opt_force:
        eq_gen_type = 'optimize_jep'
    elif opt.torque_vmc:
        eq_gen_type = 'torque_vmc'
    elif opt.opt_vmc:
        eq_gen_type = 'optimize_vmc'
    else:
        eq_gen_type = 'simple'

    # run controller on a number of different test cases
    if opt.batch:
        rospy.wait_for_service("start_contact_logger")
        try:
            start_call = rospy.ServiceProxy("start_contact_logger", StartLog)
            end_call = rospy.ServiceProxy("stop_contact_logger", StartLog)
            start_call()
            rospy.sleep(0.5)
        except rospy.ServiceException, e:
            print "service call failed: %s"%e

    active_joints = [0,1,2]
    res = epc.greedy_to_goal(arm, goal_pos, active_joints, gtg_dict,
                             joint_index_dict, timeout = 240.,
                             eq_gen_type = eq_gen_type)
    print 'res:', res[0]
    
    if opt.batch:
        try:
            result_dict = {}
            result_dict['final_q'] = epc.robot.get_joint_angles(arm)
            result_dict['result'] = res[0]
            result_dict['min_dist'] = epc.min_dist
            result_dict['end_dist'] = epc.cur_dist
            result_dict['min_config'] = epc.min_config

            ut.save_pickle(result_dict, opt.fname)
            end_call()
        except rospy.ServiceException, e:
            print "service call failed: %s"%e




