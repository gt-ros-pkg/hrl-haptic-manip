

import numpy as np, math
import sys
import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('force_torque')

import rospy
import hrl_lib.util as ut
import force_torque.FTClient as ftc

import hrl_lib.matplotlib_util as mpu
import sandbox_advait_darpa_m3.plots.visualize_logs as vl

from hrl_msgs.msg import FloatArray, FloatArrayBare

def step_cb(data):
    global step_flag
    step_flag = True


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--log', action='store_true', dest='log',
                 help='log force-torque values')
    p.add_option('--viz', action='store_true', dest='viz',
                 help='visualize the pkl (ati_ft_mag_list.pkl)')

    opt, args = p.parse_args()


    if opt.log:
        ft7 = ftc.FTClient('force_torque_ft7', netft=True)
        ft8 = ftc.FTClient('force_torque_ft8', netft=True)
        ft9 = ftc.FTClient('force_torque_ft9', netft=True)
        #ft10 = ftc.FTClient('force_torque_ft10', netft=True)
        ft11 = ftc.FTClient('force_torque_ft11', netft=True)

        #ft_sen_list = [ft7, ft8, ft9, ft10, ft11]
        ft_sen_list = [ft7, ft8, ft9, ft11]

        ut.get_keystroke('Hit ENTER to compute FT sensor bias')
        rospy.loginfo('======= before computing bias =========')
        for ft_sen in ft_sen_list:
            ft_sen.bias()
        rospy.loginfo('====== after computing bias ========')

        global step_flag
        step_flag = False
        jep_cmd_topic = '/l_arm/command/jep'
        # sync logging and task monitoring with new cmds.
        rospy.Subscriber(jep_cmd_topic, FloatArray, step_cb)

        ft_mag_list = []

        rt = rospy.Rate(100)
        rospy.loginfo('started logging ATI sensors')
        while not rospy.is_shutdown():
            if step_flag:
                step_flag = False
                for ft_sen in ft_sen_list:
                    f = ft_sen.read()[0:3]
                    f_mag = np.linalg.norm(f)
                    if f_mag < 0.5:
                        continue
                    ft_mag_list.append(f_mag)
            rt.sleep()

        ut.save_pickle(ft_mag_list, 'ati_ft_mag_list.pkl')

    if opt.viz:
        ft_mag_list = ut.load_pickle('ati_ft_mag_list.pkl')
        vl.plot_force_histogram(ft_mag_list, 'testing')
        pp.show()


