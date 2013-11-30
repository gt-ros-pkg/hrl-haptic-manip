
import numpy as np, math
import copy
from threading import RLock

import roslib; roslib.load_manifest('hrl_haptic_controllers_darpa_m3')
import rospy

import hrl_lib.util as ut
import hrl_lib.transforms as tr

import epc_skin as es

class MobileBase():
    def __init__(self):
        pass

    # vec - 3x1 np vector in local coord frame.
    # blocks until it reaches.
    def go(self, vec, ang):
        raise RuntimeError('Unimplemented function')

    def fwd(self, dist, blocking):
        raise RuntimeError('Unimplemented function')

    def back(self, dist, blocking):
        raise RuntimeError('Unimplemented function')

    def left(self, dist, blocking):
        raise RuntimeError('Unimplemented function')

    def right(self, dist, blocking):
        raise RuntimeError('Unimplemented function')


class MobileSkinEPC(es.Skin_EPC):
    def __init__(self, robot, skin_client):
        es.Skin_EPC.__init__(self, robot, skin_client)
        self.base = None

    #---------------------------------
    # transforms etc.
    #---------------------------------

    ## given a position and orientation for the torso, find the
    # position of base in the /world frame.
    def compute_base_position(self, p_torso, rot_torso):
        raise RuntimeError('Unimplemented function')

    def current_torso_pose(self):
        raise RuntimeError('Unimplemented function')

    def current_base_pose(self):
        t, r = self.current_torso_pose()
        p_base = self.compute_base_position(t, r)
        return p_base, r

    #--------- motion --------------

    ## move torso to a pose in /world
    def move_torso_to(self, p, rot, safety_margin=True):
        p_now, r_now = self.current_base_pose()
        if safety_margin:
            p_orig = copy.copy(p)
            safety_offset = r_now * np.matrix([-0.05, 0., 0.]).T
            p = p + safety_offset

        p_goal = self.compute_base_position(p, rot)
        v = r_now.T * (p_goal - p_now)

        a_now, direc = tr.matrix_to_axis_angle(r_now)
        a_goal, direc = tr.matrix_to_axis_angle(rot)
        a = a_goal - a_now
        # blocking while moving the segway is unsafe.
        self.base.go(v, a, blocking=True)

        if safety_margin:
            self.move_torso_to(p_orig, rot, safety_margin=False)

    
    def move_base_till_hit(self, fwd_distance):
        raise RuntimeError('Unimplemented function')







