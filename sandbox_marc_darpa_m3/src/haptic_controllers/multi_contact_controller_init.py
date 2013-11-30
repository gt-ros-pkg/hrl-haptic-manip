#!/usr/bin/env python  

import sys, os
import numpy as np, math
import copy

import roslib
roslib.load_manifest('sandbox_marc_darpa_m3')
import rospy
import hrl_lib.util as ut

import hrl_common_code_darpa_m3.visualization.draw_scene as ds
from hrl_haptic_manipulation_in_clutter_srvs.srv import StartLog


#unclear if need this ####from advaits switch among controllers
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TransformStamped
from hrl_srvs.srv import Bool_None

#import haptic_controllers.epc_skin as es
import hrl_haptic_controllers_darpa_m3.epc_skin as es
from haptic_controllers.epc_skin_marc import *

class ode_SkinClient(es.SkinClient):
    def __init__(self, skin_topic, num_jts):
        es.SkinClient.__init__(self, skin_topic)
        self.num_jts = num_jts
        self.max_force = 0

    def force_normal_loc_joint_list(self, normal_component_only,
                                    return_time=False):
        f_l, n_l, nm_l, loc_l, stamp = self.get_snapshot()

        jt_l = []
        for i in range(len(f_l)):
            f = f_l[i]
            n = n_l[i]
            if np.linalg.norm(f) > self.max_force:
                self.max_force = np.linalg.norm(f)
                print "Max force is :", self.max_force
            if normal_component_only:
                f_l[i] = n * np.linalg.norm(f)

            nm = nm_l[i]
            jt_num = None
            for j in xrange(self.num_jts):
                if 'link'+str(j+1) in nm:
                    jt_num = j
            # if 'link1' in nm:
            #     jt_num = 0
            # elif 'link2' in nm:
            #     jt_num = 1
            # elif 'link3' in nm:
            #     jt_num = 2
            jt_l.append(jt_num)

        if return_time:
            return f_l, n_l, loc_l, jt_l, stamp
        else:
            return f_l, n_l, loc_l, jt_l


if __name__ == '__main__':
    
    import optparse
    p = optparse.OptionParser()

    p.add_option('--qs_mpc_slow', action='store_true', dest='opt_qp_jep',
                 help='quasi-static model predictive controller')
    p.add_option('--qs_mpc_fast', action='store_true', dest='fast_mpc',
                 help='this is the same quasi-static mpc but fast')
    p.add_option('--torque_vmc', action='store_true', dest='torque_vmc',
                 help='virtual model control using torques and subtractings torques caused by contact forces')
    p.add_option('--batch', action='store_true', dest='batch',
                 help='part of a batch run, with logging etc.')
    p.add_option('--fname', action='store', dest='fname', type='string',
                 help='name of pkl file with controller result.')
    p.add_option('--teleop', action='store_true', dest='teleop', default='False',
                 help='this will make the controller run continuously until it is killed or paused')
    p.add_option('--reach_greedy', action='store_true', dest='reach_greedy', default='False',
                 help='controller only makes a single attempt to reach goal')
    p.add_option('--reach_list', action='store_true', dest='reach_list', default='False',
                 help='controller makes a single attempt to reach each point in a list of hard coded goal locations')
    p.add_option('--reach_mult', action='store_true', dest='reach_mult', default='False',
                 help='controller only makes multiple attempts to reach goal')
    p.add_option('--cody_4dof', action='store_true', dest='cody_4dof',
                 help='using cody with 4 dof ', default=False)
    p.add_option('--cody_7dof', action='store_true', dest='cody_7dof',
                 help='using cody with 7 dof ', default=False)
    p.add_option('--hil', action='store_true', dest='hil',
                 help='hardware-in-loop simulation with Cody')
    p.add_option('--sim', action='store_true', dest='sim',
                 help='software simulation')
    p.add_option('--planar_three_link_capsule', action='store_true', 
                 dest='three_link_planar', default=False,
                 help='this is to the use the three_link_planar robot config file')
    p.add_option('--six_link_planar', action='store_true', 
                 dest='six_link_planar', default=False,
                 help='this is to the use the six_link_planar robot config file')
    p.add_option('--multi_link_three_planar', action='store_true', 
                 dest='multi_link_three', default=False,
                 help='this is to the use the multi_link_three_planar robot config file')
    p.add_option('--multi_link_four_planar', action='store_true', 
                 dest='multi_link_four', default=False,
                 help='this is to the use the multi_link_four_planar robot config file')
    p.add_option('--multi_link_five_planar', action='store_true', 
                 dest='multi_link_five', default=False,
                 help='this is to the use the multi_link_five_planar robot config file')
    p.add_option('--multi_link_six_planar', action='store_true', 
                 dest='multi_link_six', default=False,
                 help='this is to the use the multi_link_six_planar robot config file')
    p.add_option('--multi_link_seven_planar', action='store_true', 
                 dest='multi_link_seven', default=False,
                 help='this is to the use the multi_link_seven_planar robot config file')
    p.add_option('--multi_link_eight_planar', action='store_true', 
                 dest='multi_link_eight', default=False,
                 help='this is to the use the multi_link_eight_planar robot config file')
    p.add_option('--use_wrist_joints', action='store_true',
                 dest='use_wrist_joints', default=False,
                 help='use wrist joints (Cody)')
    p.add_option('--ignore_mobile_base', action='store_true',
                 dest='ignore_mobile_base',
                 help='ignore mobile base (software simulation)')
    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)
    p.add_option('--ignore_skin', '--is', action='store_true', default=False,
                 dest='ignore_skin', help='ignore feedback from skin')


    opt, args = p.parse_args()

    rospy.init_node('multi_contact_marc_controller')

    if opt.sim or opt.hil:
        skin_topic_list = ['/skin/contacts']
    else:
        skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_ft']

    if opt.cody_4dof or opt.cody_7dof :
        import hrl_cody_arms.cody_arm_client as cac
        #import sandbox_advait_darpa_m3.cody.cody_guarded_move as cgm
        from sandbox_advait_darpa_m3.cody.cody_guarded_move import Cody_SkinClient
        #from sandbox_advait_darpa_m3.cody.cody_guarded_move import MobileSkinEPC_Cody


        if opt.arm == None:
            rospy.logerr('Need to specify --arm_to_use.\nExiting...')
            sys.exit()

        if opt.cody_4dof == True:
            robot = cac.CodyArmClient_4DOF(opt.arm)
        elif opt.cody_7dof == True:
            robot = cac.CodyArmClient_7DOF(opt.arm)
        else:
            rospy.logerr('Need to specify --cody_Xdof.\nExiting...')
            sys.exit()       

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

        #scl = cgm.Cody_SkinClient(skin_topic_list)
        scl = Cody_SkinClient(skin_topic_list)

#        epcon = cgm.MobileSkinEPC_Cody(robot, scl)
#        epcon = MobileSkinEPC_Cody(robot, scl)
        epc = Skin_EPC_marc(robot, scl)
        while robot.get_ep() == None:
            rospy.sleep(0.1)
        jep_start = robot.get_ep()

        q_home = np.matrix(jep_start)

        testbed = 'hardware_in_loop_cody'

        jep = robot.get_joint_angles()
        cur_pos, _ = robot.kinematics.FK(jep)
        goal_pos = np.matrix(cur_pos)

        # cur_pos, _ = robot.kinematics.FK(jep)
        # goal_pos = np.matrix(cur_pos)


    if opt.sim:

        if opt.three_link_planar == True:
            import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
            #import hrl_common_code_darpa_m3.robot_config.multi_link_three_planar as d_robot
        elif opt.six_link_planar == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_six_planar as d_robot
        elif opt.multi_link_three == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_three_planar as d_robot
        elif opt.multi_link_four == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_four_planar as d_robot
        elif opt.multi_link_five == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_five_planar as d_robot
        elif opt.multi_link_six == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_six_planar as d_robot
        elif opt.multi_link_seven == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_seven_planar as d_robot
        elif opt.multi_link_eight == True:
            import hrl_common_code_darpa_m3.robot_config.multi_link_eight_planar as d_robot
        else:
            rospy.logerr("You didn't give a robot config option for simulation, see --help \n Exiting ...")
            sys.exit()
        import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa
        q_home = np.matrix(rospy.get_param('/m3/software_testbed/joints/init_angle')).T
        robot = gsa.ODESimArm(d_robot)
        scl = ode_SkinClient(skin_topic_list, robot.kinematics.n_jts)

        testbed = 'software_simulation'

        epc = Skin_EPC_marc(robot, scl)

        goal_pos = np.matrix(rospy.get_param('m3/software_testbed/goal')).T
        # if opt.ignore_mobile_base:
        #     epcon = esa.Skin_EPC_advait(ode_arm, scl)
        #     rospy.sleep(1.)
        #     jep_start = np.radians([-140.0, 100, 120])
        # else:
        #     epcon = sgm.MobileSkinEPC_ODE(ode_arm, scl)
        #     rospy.sleep(1.)
        #     jep_start = np.radians([-100.0, 110, 110])
        #     # move base so that starting reach in location is in the
        #     # middle of the cluttered volume.
        #     v = -np.matrix([0.11439146, -0.15089079,  0.]).T
        #     epcon.base.go(v, 0, blocking=True)

        # ode_arm.set_ep(jep_start)

        jep_start = np.radians([-140.0, 100, 120])

#########################################################################################
#########################################################################################
#########################################################################################
#########################################################################################
#########################################################################################



    jep = robot.get_joint_angles()
    #robot.set_ep(jep)

    rospy.sleep(1.)

    # goal_pos[0,0] = goal_pos[0,0] -0.05
    # goal_pos[1,0] = goal_pos[1,0] -0.3

    epc.publish_goal(goal_pos, '/torso_lift_link')

    rospy.loginfo("Goal Position:"+str(goal_pos.A1))


    #gtg_dict = controllers_thresh_dict[testbed]['greedy_to_goal']

    if opt.opt_qp_jep:
        eq_gen_type = 'optimize_qp_jep'
    elif opt.fast_mpc:
        eq_gen_type = 'fast_delta_mpc_qs'
    elif opt.torque_vmc:
        eq_gen_type = 'torque_vmc'
    else:
        eq_gen_type = 'simple'


    # run controller on a number of different test cases
    if opt.batch:
        rospy.wait_for_service("start_contact_logger")
        try:
            start_call = rospy.ServiceProxy("start_contact_logger", StartLog)
            end_call = rospy.ServiceProxy("stop_contact_logger", StartLog)
            # start_call("None")
            start_call()
            rospy.sleep(0.5)
        except rospy.ServiceException, e:
            rospy.logerr("service call failed: %s"%e)
            # print "service call failed: %s"%e

    if opt.sim:

        #static_contact_stiffness_estimate = 100000.
        #static_contact_stiffness_estimate = 10000.
        static_contact_stiffness_estimate = 1000.
        estimate_contact_stiffness = False
        goal_velocity_for_hand = 0.04

        ############### Controller Specific Parameters #################
        #---------------------------
        # greedy to goal
        #---------------------------
        greedy_to_goal_control_param_dict = {
                'stopping_dist_to_goal': 0.01,  
                'stopping_ang_to_goal':0.05,                
                # for fast mpc
                'goal_state_weights' : np.diag((1., 1., 1., 1./100., 1./100., 1./100.)).tolist(),
                'task_index':[0,1,2,3,4,5],
                'orientation_weight': 4, 
                'position_weight': 5,
            # common to all.
                'jep_start': jep_start,
                'time_step': 0.01,
                'only_use_normal_force_sensing' : True,
            # for simple greedy, no_optimize_jep.
                'max_jep_mag': math.radians(0.3),
                'proportional_jep_mag_dist': 0.05,
            # for simple greedy only.
                'planar': True,
            # for no_optimize_jep, delta_qp
                'allowable_contact_force': 5.,
                'kill_controller_force': 50.,
            # for simple greedy, delta_qp.
                'goal_velocity_for_hand': goal_velocity_for_hand,
            # for delta_qp only
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': False,
                'jerk_opt_weight' : 0.00001, # 0.0 means ignore
                'save_state': False,
                'ignore_skin': opt.ignore_skin,
                #### not clear the last few are very useful 
                #### but leaving for now -- marc
                'max_delta_force_mag' : 10., # Newtons/second
                #"use_var_impedance" : True,  this is currently now being used
            }

        greedy_to_goal_monitor_param_dict = {
                'ee_motion_threshold': 0.001,
                'stopping_force': 40.
            }

    elif opt.cody_4dof or opt.cody_7dof:
        static_contact_stiffness_estimate = 1000.
        estimate_contact_stiffness = False

        # start with low estimate of stiffness and turn on the
        # stiffness estimation.
        #static_contact_stiffness_estimate = 50.
        #estimate_contact_stiffness = True
        ignore_wrist_joints = not opt.use_wrist_joints
        goal_velocity_for_hand = 0.05
        
        time_step = 0.01
        #time_step = 1.

        #---------------------------
        # greedy to goal
        #---------------------------
        greedy_to_goal_control_param_dict = {
            # for fast mpc
                'goal_state_weights' : np.diag((1, 1, 1, 1./100., 1./100., 1./100.)).tolist(),
                'task_index':[0,1,2,3,4,5],
            # common to all.
                'stopping_dist_to_goal': 0.01,  
                'jep_start': jep_start,
                'time_step': time_step,
                'only_use_normal_force_sensing' : True,
            # for simple greedy, no_optimize_jep.
                'max_jep_mag': math.radians(0.4),
                'proportional_jep_mag_dist': 0.05,
            # for simple greedy only.
                'planar': False,
            # for no_optimize_jep, delta_qp
                'allowable_contact_force': 5.,
                'kill_controller_force': 20.,
            # for simple greedy, delta_qp.
                'goal_velocity_for_hand': goal_velocity_for_hand,
            # for delta_qp only
                'k': static_contact_stiffness_estimate,
                'estimate_contact_stiffness': estimate_contact_stiffness,
                'ignore_wrist_joints': ignore_wrist_joints,
                'jerk_opt_weight' : 0.00001, # 0.0 means ignore
                'save_state': False,
                'ignore_skin': opt.ignore_skin,
                #### not clear the last few are very useful 
                #### but leaving for now -- marc
                'max_delta_force_mag' : 10., # Newtons/second
                #"use_var_impedance" : True,  this is currently now being used
            }

        greedy_to_goal_monitor_param_dict = {
                'ee_motion_threshold': 0.001,
                'stopping_force': 25.
            }


    if opt.teleop == True:
        greedy_to_goal_control_param_dict['stopping_dist_to_goal']= -1
        # epc.greedy_to_goal(goal_pos, 
        #                    greedy_to_goal_control_param_dict,
        #                    np.inf,
        #                    eq_gen_type,
        #                    'teleop_logging', 
        #                    greedy_to_goal_monitor_param_dict)

        res = epc.teleop_greedy_to_goal(goal_pos, None, 
                                        greedy_to_goal_control_param_dict, 
                                        eq_gen_type,
                                        'teleop_logging')

    elif opt.reach_greedy == True:
        res = epc.greedy_to_goal_marc(goal_pos, None, greedy_to_goal_control_param_dict,
                                      180., eq_gen_type)
    elif opt.reach_mult == True:
        res = epc.greedy_pull_in_and_out(goal_pos, None, eq_gen_type, q_home, greedy_to_goal_control_param_dict)
    elif opt.reach_list == True:
        goals = [[0.10, -0.3,   0],
                 [0.55, -0.32,  0],
                 [0.10, -0.3,   0],
                 [0.10, -0.075, 0],
                 [0.55, -0.095, 0],
                 [0.10, -0.075, 0],
                 [0.10, 0.15,   0],
                 [0.60, 0.35,   0]]

        for point in goals:
            goal_pos = np.matrix(point).T
            res = epc.greedy_to_goal_marc(goal_pos, goal_orientation, greedy_to_goal_control_param_dict,
                                          100., eq_gen_type)
            

    else:
        rospy.logerr("Should have at least designated either teleop, single autonomous reach or multiple reaches in options")
        assert(False)

    rospy.loginfo("Result is: "+str(res[0]))
    
    if opt.batch:
        try:
            result_dict = {}
            result_dict['final_q'] = robot.get_joint_angles()
            result_dict['result'] = res[0]
            result_dict['parameters'] = greedy_to_goal_control_param_dict

            rospy.loginfo("Robot joint angles: "+str(robot.get_joint_angles()))
            rospy.loginfo("Results: "+str(res[0]))

            # end_call(opt.fname+"_logger.pkl")
            end_call()
            ut.save_pickle(result_dict, opt.fname)          

        except rospy.ServiceException, e:
            rospy.logerr("service call failed: %s"%e)
            ut.save_pickle(result_dict, opt.fname)



