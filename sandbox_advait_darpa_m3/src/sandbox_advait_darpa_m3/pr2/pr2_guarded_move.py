
import numpy as np, math
import copy
from threading import RLock

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('segway_vo')

import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.transforms as tr

import equilibrium_point_control.epc as epc
import segway_vo.segway_command as sc

import hrl_cody_arms.cody_arm_client as cac
import hrl_haptic_controllers_darpa_m3.epc_skin as es
import hrl_haptic_controllers_darpa_m3.mobile_skin_epc as mse

from geometry_msgs.msg import TransformStamped

class PR2_SkinClient(es.SkinClient):
    def __init__(self, skin_topic_list):
        es.SkinClient.__init__(self, skin_topic_list)

    # list of force vectors, list of normal vectors, list of joint number after which the
    # joint torque will have no effect on the contact force.
    def force_normal_loc_joint_list(self, normal_component_only,
                                    return_time=False):
        f_l, n_l, nm_l, loc_l, stamp = self.get_snapshot()

        jt_l = []
        for i in range(len(f_l)):
            f = f_l[i]
            n = n_l[i]
            if normal_component_only:
                f_l[i] = n * np.linalg.norm(f)

            nm = nm_l[i]
            if 'shoulder_pan' in nm:
                jt_num = 0
            elif 'shoulder_lift' in nm:
                jt_num = 1
            elif 'upper_arm' in nm:
                jt_num = 2
            elif 'elbow' in nm:
                jt_num = 3
            elif 'forearm' in nm:
                jt_num = 4
            elif 'wrist_flex' in nm:
                jt_num = 5
            else:
                jt_num = 6

            jt_l.append(jt_num)

        if return_time:
            return f_l, n_l, loc_l, jt_l, stamp
        else:
            return f_l, n_l, loc_l, jt_l



