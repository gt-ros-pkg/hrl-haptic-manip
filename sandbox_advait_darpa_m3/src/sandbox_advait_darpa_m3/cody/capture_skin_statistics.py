
import sys
import numpy as np, math
import copy
from threading import RLock

import itertools
import matplotlib.pyplot as pp

from scipy.spatial import KDTree

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('hrl_meka_skin_sensor_darpa_m3')

import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu

import hrl_meka_skin_sensor_darpa_m3.skin_patch_calibration as spc


# collect the training data
def create_training_set(robot, rdc, duration):
    q_list = []
    taxel_force_list = []
    i = 0
    
    data_dict = {}
    t_end = rospy.get_time() + duration
    while True:
        p = rdc.get_raw_data(fresh=True)
        q = robot.get_joint_angles()
        taxel_force_list.append(p)
        q_list.append(q[5:7])
        k = tuple(q[5:7])
        if k not in data_dict:
            data_dict[k] = [p]
        else:
            data_dict[k].append(p)

        rospy.sleep(0.1)
        if rospy.get_time() > t_end:
            break

    ut.save_pickle(data_dict, 'capture_statistics_training_data_'+ut.formatted_time()+'.pkl')

def zigzag_cody_wrist_motion(epcon):
    max_jep1 = math.radians(45)
    max_jep2 = math.radians(45)
    step = math.radians(2)
    speed = math.radians(20)

    jep = list(robot.get_ep())
    jep[5] = max_jep1
    jep[6] = max_jep2
    epcon.go_jep(jep, speed)

    raw_input('Hit ENTER to being the zigzag pattern')

    while not rospy.is_shutdown():
        #jep[5] = max_jep1
        #jep[6] = max_jep2
        for jep1 in np.arange(max_jep1, -max_jep1-step, -step):
            jep[5] = jep1
            jep[6] = -max_jep2
            epcon.go_jep(jep, speed)
            jep[6] = max_jep2
            epcon.go_jep(jep, speed)
        #jep[5] ~ -max_jep1
        #jep[6] = max_jep2
        for jep2 in np.arange(max_jep2, -max_jep2-step, -step):
            jep[6] = jep2
            jep[5] = max_jep1
            epcon.go_jep(jep, speed)
            jep[5] = -max_jep1
            epcon.go_jep(jep, speed)
        #jep[5] = -max_jep1
        #jep[6] ~ -max_jep2
        for jep1 in np.arange(-max_jep1, max_jep1+step, step):
            jep[5] = jep1
            jep[6] = max_jep2
            epcon.go_jep(jep, speed)
            jep[6] = -max_jep2
            epcon.go_jep(jep, speed)
        #jep[5] ~ max_jep1
        #jep[6] = -max_jep2
        for jep2 in np.arange(-max_jep2, max_jep2+step, step):
            jep[6] = jep2
            jep[5] = -max_jep1
            epcon.go_jep(jep, speed)
            jep[5] = max_jep1
            epcon.go_jep(jep, speed)
        #jep[5] = max_jep1
        #jep[6] ~ max_jep2

def create_kdtree(data_dict):
    kd = KDTree(data_dict.keys())
    return kd

def compute_bias_mn_std(q1, q2, kd, data_dict, k):
    dists, idxs = kd.query((q1,q2), k)
    qs = kd.data[idxs]

    wt_list = []
    force_l_list = []
    for q, d in zip(qs, dists):
        f = data_dict[tuple(q)]
        force_l_list.extend(f)
        wt_list.extend([d]*len(f))

    mn, std = ut.weighted_avg_and_std(np.matrix(force_l_list),
                                      np.array(wt_list),
                                      unbiased=False)
    return mn, std
        
#---------- visualization -------------

def scatter_plot_wrist_angles(data_dict):
    q_arr = np.array(data_dict.keys())
    q1_arr = q_arr[:,0]
    q2_arr = q_arr[:,1]

    mpu.figure()
    pp.scatter(np.degrees(q1_arr), np.degrees(q2_arr), alpha=0.5,
               linewidth=0)
    pp.axis('equal')

def scatter_plot_std(data_dict):
    kd = create_kdtree(data_dict)

    max_q1 = math.radians(45)
    max_q2 = math.radians(45)
    step = math.radians(2)

    std_list = [[] for i in range(15)]
    mn_list = [[] for i in range(15)]
    max_q_l = []
    dist_q_l = []

    for q1 in np.arange(max_q1, -max_q1-step, -step):
        for q2 in np.arange(max_q2, -max_q2-step, -step):
            max_q_l.append(math.degrees(max(q1,q2)))
            dist_q_l.append(math.degrees(math.sqrt(q1*q1+q2*q2)))
            mn, std = compute_bias_mn_std(q1, q2, kd, data_dict, k=10)
            for i,(m,s) in enumerate(zip(mn.tolist(), std.tolist())):
                mn_list[i].append(m)
                std_list[i].append(s)

    nrows = 3
    ncols = 5

    fig = mpu.figure()
    fig.subplots_adjust(hspace=0.0001, wspace=0.0001, bottom=0.07,
                        top=0.96, left=0.04, right=0.96)
    for i in range(15):
        r = (i % nrows)
        c = (i / nrows)
        
        sp = pp.subplot(nrows, ncols, r*ncols + c + 1)

#        pp.scatter(dist_q_l, mn_list[i], alpha=0.5, linewidth=0, color='g')
#        pp.ylim((-10, 250))
        pp.xlim((-5, 70))
        pp.scatter(dist_q_l, std_list[i], alpha=0.5, linewidth=0)
        pp.ylim((-10, 250))
        pp.xlim((-5, 70))

        if c != 0:
            pp.setp(sp.get_yticklabels(), visible=False)
        if r != nrows-1:
            pp.setp(sp.get_xticklabels(), visible=False)

    pp.suptitle('std of ADC values (y-axis) and distance from (0,0) in degrees')


def compare_constant_bias_and_config_dependant_bias(data_dict):
    kd = create_kdtree(data_dict)
    nrows = 3
    ncols = 5

    const_err_list = [[] for i in range(15)]
    config_err_list = [[] for i in range(15)]
    dist_q_l = []

    const_bias_mn, _ = compute_bias_mn_std(0., 0., kd, data_dict, k=10)

    for k in data_dict.keys()[::1]:
        if abs(k[1]) > math.radians(45.):
            continue

        for p in data_dict[k]:
            dist_q_l.append(math.degrees(math.sqrt(k[0]*k[0]+k[1]*k[1])))

            err_const = np.array(p) - const_bias_mn
            config_bias_mn, _ = compute_bias_mn_std(k[0], k[1], kd,
                                                    data_dict, k=10)
            err_config = np.array(p) - config_bias_mn

            for i, (a,b) in enumerate(zip(err_const.tolist(), err_config.tolist())):
                const_err_list[i].append(a)
                config_err_list[i].append(b)

    fig = mpu.figure()
    fig.subplots_adjust(hspace=0.0001, wspace=0.0001, bottom=0.07,
                        top=0.96, left=0.04, right=0.96)
    for i in range(15):
        r = (i % nrows)
        c = (i / nrows)
        
        sp = pp.subplot(nrows, ncols, r*ncols + c + 1)

        pp.scatter(dist_q_l, config_err_list[i], alpha=0.5,
                   linewidth=0, color='b', s=3)
        pp.scatter(dist_q_l, const_err_list[i], alpha=1.0,
                   linewidth=0, color='y', s=3)
        pp.ylim((-300, 50))
        pp.xlim((-5, 70))

        if c != 0:
            pp.setp(sp.get_yticklabels(), visible=False)
        if r != nrows-1:
            pp.setp(sp.get_xticklabels(), visible=False)

    pp.suptitle('difference of measured ADC value and predicted value (y-axis) using a constant (yellow) or a config dependant value computed from mean of 10 neighbors (blue) as a function of distance of the wrist configuration from (0,0) in degrees')




if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--collect_training_data', '--ctd',
                 action='store_true', dest='ctd')
    p.add_option('--zig_zag_cody', '--zzc', action='store_true',
                 dest='zzc', help='move wrist in zig zag pattern to collect data')
    p.add_option('--viz', action='store_true', dest='viz',
                 help='visualize the collected data')
    p.add_option('--training_data_pkl', '--tdp', action='store',
                 dest='tdp', type='string',
                 help='pkl with the training data')

    opt, args = p.parse_args()

    if opt.viz:
        if opt.tdp == None:
            raise RuntimeError('Please specify a training data pkl (--tdp)')

        data_dict = ut.load_pickle(opt.tdp)
        #scatter_plot_wrist_angles(data_dict)
        scatter_plot_std(data_dict)
        compare_constant_bias_and_config_dependant_bias(data_dict)

        pp.show()
        sys.exit()


    #--- below this need ROS -----

    import hrl_cody_arms.cody_arm_client as cac

    rospy.init_node('capture_skin_statistics_node')
    rospy.loginfo('waiting for things to come up')

    arm_to_use = 'r'
    robot = cac.CodyArmClient(arm_to_use)
    rdc = spc.RawDataClient('/fabric_skin/taxels/raw_data')
    
    while robot.get_ep() == None:
        rospy.sleep(0.1)
    rospy.loginfo('Ready')

    if opt.ctd:
        rospy.loginfo('Going to start creating the training dataset')
        rospy.sleep(0.5)
        create_training_set(robot, rdc, duration=1200)
        rospy.loginfo('Done creating the training dataset')

    if opt.zzc:
        import equilibrium_point_control.epc as epc
        epcon = epc.EPC(robot)
        rospy.loginfo('Going to move wrist in zigzag pattern.')
        zigzag_cody_wrist_motion(epcon)







