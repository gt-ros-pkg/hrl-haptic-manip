
import sys
import numpy as np, math
import copy
from threading import RLock

import itertools
import matplotlib.pyplot as pp

from enthought.mayavi import mlab

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('hrl_meka_skin_sensor_darpa_m3')

import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu

import hrl_meka_skin_sensor_darpa_m3.skin_patch_calibration as spc

step = math.radians(5)
min_angle = math.radians(-45)
max_angle = math.radians(45)
angles_1 = np.arange(min_angle, max_angle + step, step)
#angles_2 = np.arange(min_angle, max_angle + step, step)
angles_2 = np.array([0.])



# collect the training data
def create_training_set(robot, rdc, iterations):
    q_list = []
    taxel_force_list = []
    i = 0
    
    for i in range(iterations):
        jep = list(robot.get_ep())
        jep[5] = angles_1[0]
        jep[6] = angles_2[0]
        robot.set_ep(jep)

        raw_input('Hit a key when ready')

        for a1 in angles_1:
            for a2 in angles_2:
                jep = list(robot.get_ep())
                jep[5] = a1
                jep[6] = a2

                robot.set_ep(jep)
                rospy.sleep(1.)
                p = rdc.get_raw_data(fresh=True)
                q = robot.get_joint_angles()
                taxel_force_list.append(p)
                q_list.append(q[5:7])
                rospy.loginfo('added point to training set')

    d = {}
    d['q_list'] = q_list
    d['taxel_force_list'] = taxel_force_list

    #print 'd:', d
    ut.save_pickle(d, 'taxel_force_training_data_'+ut.formatted_time()+'.pkl')

def find_closest_key(q, key_arr):
    idx = np.argmin(ut.norm((key_arr - q).T))
    return tuple(key_arr[idx])

def create_lookup_table(q_list, taxel_force_list):
    lut = {}
    keys = itertools.product(angles_1, angles_2)
    key_list = []

    for k in keys:
        if not k.__class__ == tuple:
            raise RuntimeError('k is not a tuple. it is %s'%k.__class__)
        lut[k] = {'q_list': [], 'f_list': [], 'f_mean': None,
                  'f_std': None, 'n_pts': None}
        key_list.append(k)

    key_arr = np.array(key_list)
    for q, f in itertools.izip(q_list, taxel_force_list):
        k = find_closest_key(np.array(q), key_arr)
        lut[k]['q_list'].append(q)
        lut[k]['f_list'].append(f)

    for k in key_list:
        fl = lut[k]['f_list']
        lut[k]['n_pts'] = len(fl)
        fa = np.array(fl)
        lut[k]['f_mean'] = fa.mean(0)
        lut[k]['f_std'] = fa.std(0)
    return lut


def visualize_lookup_table(lut, nrows, ncols):
    keys = lut.keys()
    keys.sort()
    x, y = np.meshgrid(range(ncols), range(nrows))
    x = x.flatten() * 1.
    y = y.flatten() * 1.

    mn_list = []
    std_list = []
    k_list = []
    for k in keys:
        mn = lut[k]['f_mean'] * 1.
        if mn.shape == ():
            continue
        k_list.append(math.degrees(k[0]))
        mn_list.append(mn)
        std_list.append(lut[k]['f_std'])

    # each item of list is the means for that taxel as the joint angle
    # varied according to k_list
    per_taxel_mns_l = np.row_stack(mn_list).T.tolist()
    per_taxel_stds_l = np.row_stack(std_list).T.tolist()


    fig = mpu.figure()
    fig.subplots_adjust(hspace=0.0001, wspace=0.0001, bottom=0.07,
                        top=0.96, left=0.04, right=0.96)

    for i, mns in enumerate(per_taxel_mns_l):
        r = (i % nrows)
        c = (i / nrows)
        
        sp = pp.subplot(nrows, ncols, r*ncols + c + 1)
        pp.errorbar(k_list, mns, yerr=per_taxel_stds_l[i])

        pp.ylim((450, 1010))
        pp.xlim((-45, 50))
        if c != 0:
            pp.setp(sp.get_yticklabels(), visible=False)
        if r != nrows-1:
            pp.setp(sp.get_xticklabels(), visible=False)

    pp.suptitle('mean(std) of forces for each taxel for different configurations')
    pp.show()

# 3D bar plot
#    for k in keys:
#        mlab.figure()
#        s = lut[k]['f_mean'] / 500.
#        if s.shape == ():
#            continue
#        mlab.barchart(x, y, s)
#        mlab.title('k: %.0f, %.0f'%(math.degrees(k[0]), math.degrees(k[1])))
#    mlab.show()



class Fabric_Skin_Cody_Calibration(spc.SkinCalibration):
    def __init__(self, robot, lut):
        spc.SkinCalibration.__init__(self, '/fabric_skin')
        self.robot = robot
        self.lut = lut
        self.key_arr = np.array(lut.keys())

    def raw_data_to_force(self, raw_data):
        q = self.robot.get_joint_angles()[5:7]
        k = find_closest_key(q, self.key_arr)
        bias = self.lut[k]['f_mean']
        force = (bias - raw_data) / 100. # calibration!
        return force



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--publish_calib_data', '--pcd', action='store_true',
                 dest='pcd')
    p.add_option('--viz_lut', '--vl', action='store_true',
                 dest='vl', help='visualize the lookup table')

    p.add_option('--lut_pkl', '--lp', type='string', action='store',
                 dest='lp', help='pkl with the lookup table')

    p.add_option('--collect_training_data', '--ctd',
                 action='store_true', dest='ctd')
    p.add_option('--create_lookup_table', '--clt',
                 action='store_true', dest='clt')
    p.add_option('--training_data_pkl', '--tdp', type='string',
                 action='store', dest='tdp',
                 help='pkl with the training data')
    opt, args = p.parse_args()


    if opt.clt:
        if opt.tdp == None:
            raise RuntimeError('Need to specify a training data pkl')
        d = ut.load_pickle(opt.tdp)
        lut = create_lookup_table(d['q_list'], d['taxel_force_list'])
        ut.save_pickle(lut, 'lookup_table_'+ut.formatted_time()+'.pkl')
        sys.exit()

    if opt.vl:
        if opt.lp == None:
            raise RuntimeError('need to specify an lut_pkl')
        lut = ut.load_pickle(opt.lp)
        visualize_lookup_table(lut, 3, 5)
        sys.exit()


    #--- below this need ROS -----

    import hrl_cody_arms.cody_arm_client as cac

    rospy.init_node('fabric_skin_on_cody_calibration_node')
    rospy.loginfo('waiting for things to come up')

    arm_to_use = 'r'
    robot = cac.CodyArmClient(arm_to_use)
    rdc = spc.RawDataClient('/fabric_skin/taxels/raw_data')
    
    while robot.get_ep() == None:
        rospy.sleep(0.1)
    rospy.loginfo('Ready')

    if opt.ctd:
        create_training_set(robot, rdc, iterations=5)

    if opt.pcd:
        if opt.lp == None:
            raise RuntimeError('need to specify an lut_pkl')

        lut = ut.load_pickle(opt.lp)
        fscc = Fabric_Skin_Cody_Calibration(robot, lut)
        fscc.precompute_taxel_location_and_normal()

        while not rospy.is_shutdown():
            d = rdc.get_raw_data(True)
            fscc.publish_taxel_array(d)








