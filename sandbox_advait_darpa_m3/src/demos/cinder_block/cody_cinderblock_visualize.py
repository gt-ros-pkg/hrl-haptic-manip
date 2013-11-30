
import sys, os
import matplotlib.pyplot as pp
import numpy as np, math

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')

import sandbox_advait_darpa_m3.plots.visualize_logs as vl
import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu


def force_histogram(pkl_list):
    for pkl in pkl_list:
        pkl_nm_split = '.'.join((pkl.split('/')[-1]).split('.')[0:-1])
        print 'pkl:', pkl

        if 'to_goal' in pkl:
            lab = 'reaching to goal'
        elif 'pull_out' in pkl:
            lab = 'pulling out'
        else:
            continue

        d = ut.load_pickle(pkl)
        if 'controller' in d:
            f_mag_l = d['all_forces_list']
            if f_mag_l != []:
                mpu.figure()
                vl.plot_force_histogram(f_mag_l, lab)



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--dir', action='store', dest='direc',
                 type='string', default = None,
                 help='directory with all the logs from one switched control trial.')

    opt, args = p.parse_args()

    import hrl_cody_arms.cody_arm_kinematics as cak

    arm = 'r'
    kinematics = cak.CodyArmKinematics(arm)
    #kinematics.set_tooltip(np.matrix([0.,0.,-0.16]).T) # tube
    #kinematics.set_tooltip(np.matrix([0.,0.,-0.01]).T) # stub
    kinematics.set_tooltip(np.matrix([0.,0.,-0.04]).T) # stub with mini45

    if opt.direc == None:
        print 'Please provide a directory'
        print 'Exiting ...'
        sys.exit()

    # want pkl_list in the order in which they were saved to disk.
    pkl_list = ut.get_bash_command_output('ls -t '+opt.direc+'/*.pkl')
    pkl_list.reverse()

    force_histogram(pkl_list)
    pp.show()



