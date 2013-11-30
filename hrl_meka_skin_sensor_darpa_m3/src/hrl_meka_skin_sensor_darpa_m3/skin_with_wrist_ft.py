#!/usr/bin/python

import numpy as np, math
from threading import RLock
import copy

import roslib; roslib.load_manifest('hrl_meka_skin_sensor_darpa_m3')
import rospy
import tf

import hrl_lib.transforms as tr

from hrl_msgs.msg import FloatArray
from hrl_msgs.msg import FloatArrayBare
from hrl_haptic_manipulation_in_clutter_msgs.msg import SkinContact
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

class ForceClient():
    def __init__(self):
        self.raw_data = None # 1D np array
        self.fresh_data = False
        self.bias = None
        self.lock = RLock()
        rospy.Subscriber('/sensor/force', FloatArray, self.force_cb)

    def force_cb(self, msg):
        with self.lock:
            self.raw_data = np.array(msg.data)
            self.fresh_data = True

    def get_raw_data(self, fresh):
        if fresh:
            while not self.fresh_data:
                rospy.sleep(0.002)
        with self.lock:
            self.fresh_data = False
            d = copy.copy(self.raw_data)
        return d

    def bias_data(self, n):
        rospy.logwarn('started biasing...')
        d_list = []
        for i in range(n):
            d_list.append(self.get_raw_data(fresh = True))
        d_arr = np.row_stack(d_list)
        mn = np.mean(d_arr, 0)
        std = np.std(d_arr, 0)
        self.bias = mn
        self.std = std
        rospy.logwarn('...done')

    # n_std = 0 is the same as no thresholding.
    # return a 1D np array of length 6.
    def get_biased_data(self, fresh, n_std=0):
        d = self.get_raw_data(fresh)
        d_biased = d - self.bias
        idxs = np.where(np.abs(d_biased) < n_std * self.std)[0]
        d_biased[idxs] = 0
        return d_biased


class FT_to_SkinContact():
    # arm - which arm to use ('l' or 'r')
    def __init__(self, arm):
        self.use_right_arm = (arm == 'r')
        self.sc_pub = rospy.Publisher('/ft/skin_contact', SkinContact)
        self.tf_lstnr = tf.TransformListener()

    def publish_skin_contact(self, ft):
        # -ve sign because we want the force that the robot is applying
        # on the world.
        ft = -np.matrix(ft).T
        force = ft[0:3,:]

        f_mag = np.linalg.norm(force)
        torque = ft[3:,:]

        msg = SkinContact()
        if self.use_right_arm:
            frm_id = '/handmount_RIGHT'
        else:
            frm_id = '/handmount_LEFT'

        msg.header.frame_id = '/torso_lift_link'
        msg.header.stamp = rospy.Time.now()

        if f_mag > 2.0:
            t, q = self.tf_lstnr.lookupTransform('/torso_lift_link',
                                                 frm_id, rospy.Time(0))
            t = np.matrix(t).reshape(3,1)
            rot = tr.quaternion_to_matrix(q)

            # approx location of the FT sensor in the handmount_LEFT
            # or handmount_RIGHT frame.
            pt = np.matrix([0., 0., -0.06]).T

            # using a fixed location for the contact, for now.
            if False:
                # here I can try and compute the line of action of
                # the force and then attempt to find the contact
                # location by taking the intersection of the line of
                # action with the surface of the cylinder.
                # Too complicated for now (Sept 29, 2011)
                p_oc = np.cross(force.A1, torque.A1) / (force.T * force)[0,0]
                pt = np.matrix(p_oc).T + pt

            # now transform vectors into the torso_lift_link
            force = rot * force
            n = force / f_mag
            pt = rot * pt + t

            msg.locations.append(Point(pt[0,0], pt[1,0], pt[2,0]))
            msg.forces.append(Vector3(force[0,0], force[1,0], force[2,0]))
            msg.normals.append(Vector3(n[0,0], n[1,0], n[2,0]))

            msg.pts_x.append(FloatArrayBare([pt[0,0]]))
            msg.pts_y.append(FloatArrayBare([pt[1,0]]))
            msg.pts_z.append(FloatArrayBare([pt[2,0]]))

            if self.use_right_arm:
                msg.link_names.append('end_effector_RIGHT')
            else:
                msg.link_names.append('end_effector_LEFT')

        self.sc_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('FT_sensor_to_skin_contact')
    ft_to_sc = FT_to_SkinContact('r')
    fc = ForceClient()
    fc.bias_data(50)
    
    while not rospy.is_shutdown():
        ft = fc.get_biased_data(True, 0)
        ft_to_sc.publish_skin_contact(ft)



