#   Copyright 2013 Georgia Tech Research Corporation
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
#  http://healthcare-robotics.com/

#
# Any robot that wants to use equilibrium point control should implement the functions
# sketched out in the HRLArm and HRLArmKinematics
#

# @package hrl_haptic_mpc
# @author Advait Jain

import numpy as np
import copy
from threading import RLock

try:
    import hrl_lib.geometry as hg
except ImportError, e:
    print '<hrl_arm.py> WARNING:', e


class HRLArm():

    def __init__(self, kinematics):
        # object of class derived from HRLArmKinematics
        self.kinematics = kinematics
        self.ep = None  # equilibrium point
        self.kp = None  # joint stiffness
        self.kd = None  # joint damping
        self.q = None  # angles
        self.qdot = None  # angular velocity
        self.joint_names_list = None  # joint names
        self.lock = RLock()

    def get_joint_velocities(self):
        with self.lock:
            return copy.copy(self.qdot)

    def get_joint_angles(self):
        with self.lock:
            return copy.copy(self.q)

    def set_ep(self, *args):
        raise RuntimeError('Unimplemented Function')

    # publish different viz markers.
    def publish_rviz_markers(self):
        raise RuntimeError('Unimplemented Function')

    def get_ep(self):
        with self.lock:
            return copy.copy(self.ep)

    # returns kp, kd
    # np arrays of stiffness and damping of the virtual springs.
    def get_joint_impedance(self):
        with self.lock:
            return copy.copy(self.kp), copy.copy(self.kd)

    def get_joint_names(self):
        with self.lock:
            return copy.copy(self.joint_names_list)

    # do we really need this function?
    def freeze(self):
        self.set_ep(self.ep)

    def get_end_effector_pose(self):
        return self.kinematics.FK(self.get_joint_angles())


class HRLArmKinematics():

    def __init__(self, n_jts):
        self.tooltip_pos = np.matrix([0., 0., 0.]).T
        self.tooltip_rot = np.matrix(np.eye(3))
        self.n_jts = n_jts

    # FK without the tooltip
    def FK_vanilla(self, q, link_number=None):
        raise RuntimeError('Unimplemented Function')

    # @param q - array-like (RADIANs)
    # @param link_number - perform FK up to this link. (0-n_jts)
    # @return pos (3X1) np matrix, rot (3X3) np matrix
    def FK(self, q, link_number=None):
        if link_number is None:
            link_number = self.n_jts
        if link_number > self.n_jts:
            raise RuntimeError(
                'Link Number is greater than n_jts: %d' % link_number)
        pos, rot = self.FK_vanilla(q, link_number)

        if link_number == self.n_jts:
            tooltip_baseframe = rot * self.tooltip_pos
            pos += tooltip_baseframe
            rot = rot * self.tooltip_rot
        return pos, rot

    ##
    # Computes IK for the tooltip.  The desired location is first transformed
    # back into the last link's frame and IK is performed on that location.
    # @param pos Desired link position (3x1 np matrix)
    # @param rot Desired link rotation (3x3 np matrix)
    # @param q_guess Estimate of the desired joint angles which seeds the IK solver
    def IK(self, pos, rot, q_guess=None):
        last_link_pos = pos - rot * self.tooltip_rot.T * self.tooltip_pos
        last_link_rot = rot * self.tooltip_rot.T
        return self.IK_vanilla(last_link_pos, last_link_rot, q_guess)

    # IK without the  tooltip.
    def IK_vanilla(self, p, rot, q_guess=None):
        raise RuntimeError('Unimplemented Function')

    # @param p - 3x1 np matrix
    # @param rot - orientation of end effector frame wrt base of the arm.
    def IK(self, p, rot, q_guess=None):
        # this code should be common to everyone.
        pass

    # compute Jacobian at point pos.
    def jacobian(self, q, pos=None):
        raise RuntimeError('Unimplemented Function')

    # return min_array, max_array
    def get_joint_limits(self):
        raise RuntimeError('Unimplemented Function')

    # define tooltip as a 3x1 np matrix in the wrist coord frame.
    def set_tooltip(self, p, rot=np.matrix(np.eye(3))):
        self.tooltip_pos = copy.copy(p)
        self.tooltip_rot = copy.copy(rot)

    # ----- 2D functions ----------

    # return list of 2D points corresponding to the locations of the
    # joint axes for a planar arm. Something funky for a spatial arm
    # that Advait does not want to put into words.
    def arm_config_to_points_list(self, q):
        return [self.FK(q, i)[0].A1[0:2] for i in range(len(q) + 1)]

    # project point onto the arm skeleton in 2D and compute distance
    # along it to the end effector.
    def distance_from_ee_along_arm(self, q, pt):
        p_l = self.arm_config_to_points_list(q)
        ee = self.FK(q)[0]
        d_ee = hg.distance_along_curve(ee, p_l)
        d_pt = hg.distance_along_curve(pt, p_l)
        assert(d_ee >= d_pt)
        return d_ee - d_pt

    # distance of a point from the arm
    def distance_from_arm(self, q, pt):
        p_l = self.arm_config_to_points_list(q)
        return hg.distance_from_curve(pt, p_l)

    # is pt at the joint?
    # pt - 2x1 or 3x1 np matrix
    # return True if distance between a joint and the point projected
    # onto the skeleton is <= dist_threshold.
    #
    # tested only for planar arms. (see test_contact_at_joints.py in
    # sandbox_advait_darpa_m3/src/sandbox_advait_darpa_m3/software_simulation)
    def is_contact_at_joint(self, pt, q, dist_threshold):
        pts_list = [self.FK(q, i)[0].A1 for i in range(len(q) + 1)]
        proj_pt = hg.project_point_on_curve(pt, pts_list)

        # ignore end effector (it is not a joint)
        for jt in pts_list[:-1]:
            dist = np.linalg.norm(np.matrix(jt).T - proj_pt)
            if dist <= dist_threshold:
                return True
        return False
