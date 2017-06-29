#!/usr/bin/python

#
# Sept 6, 2011
# Going to also use this node to interface with the TaxelArray from
# the Meka skin patch. For that I will need to accumulate the received
# TaxelArray messages over time, since a single message will not be
# sufficient to create a complete SkinContact message.
#
# Ignoring this for now and publishing a new SkinContact msg from
# within the TaxelArray callback.
#
#
#
#

import numpy as np, math

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.viz as hv
import hrl_lib.util as ut
import hrl_lib.transforms as tr

import rospy
import tf

from visualization_msgs.msg import Marker
from hrl_haptic_manipulation_in_clutter_msgs.msg import SkinContact

from hrl_haptic_manipulation_in_clutter_msgs.msg import TaxelArray
#from m3skin_ros.msg import TaxelArray as TaxelArray_Meka
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3


def taxel_array_cb(ta, callback_args):
    sc_pu, tf_lstnr = callback_args
    sc = SkinContact()
    sc.header.frame_id = '/torso_lift_link' # has to be this and no other coord frame.
    sc.header.stamp = ta.header.stamp

    pts = np.column_stack((ta.centers_x, ta.centers_y, ta.centers_z))
    nrmls = np.column_stack((ta.normals_x, ta.normals_y, ta.normals_z))
    fs = np.column_stack((ta.values_x, ta.values_y, ta.values_z))

    t1, q1 = tf_lstnr.lookupTransform(sc.header.frame_id,
                                      ta.header.frame_id,
                                      rospy.Time(0))
                                      #ta.header.stamp)

    t1 = np.matrix(t1).reshape(3,1)
    r1 = tr.quaternion_to_matrix(q1)

    if ta.link_names == []:
        real_skin_patch = True
    else:
        real_skin_patch = False

    # transform to the torso_lift_link frame.
    pts = r1 * np.matrix(pts).T + t1
    nrmls = r1 * np.matrix(nrmls).T
    fs = r1 * np.matrix(fs).T
    fmags = ut.norm(fs).A1

    # for fabric sensor and meka sensor, Advait moved the thresholding
    # etc within the calibration node. (June 13, 2012)
    if not real_skin_patch:
        idxs = np.where(fmags > 0.5)[0]
    else:
        idxs = np.where(fmags > 0.001)[0]

    for i in idxs:
        p = pts[:,i]
        n1 = nrmls[:,i]
        n2 = fs[:,i]

        if real_skin_patch:
            link_name = ta.header.frame_id
        else:
            link_name = ta.link_names[i]
        
        sc.locations.append(Point(p[0,0], p[1,0], p[2,0]))
        sc.forces.append(Vector3(n2[0,0], n2[1,0], n2[2,0]))
        sc.normals.append(Vector3(n1[0,0], n1[1,0], n1[2,0]))
        sc.link_names.append(link_name)
        sc.pts_x.append(FloatArrayBare(p[0,:].A1))
        sc.pts_y.append(FloatArrayBare(p[1,:].A1))
        sc.pts_z.append(FloatArrayBare(p[2,:].A1))

    sc_pub.publish(sc)



if __name__ == '__main__':
    import optparse
#    p = optparse.OptionParser()
#    p.add_option('--meka_skin_patch', action='store_true',
#                 dest='meka_skin_patch',
#                 help='data coming from Meka skin patch')
#    opt, args = p.parse_args()

    node_nm = 'taxel_array_to_skin_contact'
#    if opt.meka_skin_patch:
#        node_nm = 'taxel_array_to_skin_contact_for_meka_skin_patch'

    rospy.init_node(node_nm)

    tf_lstnr = tf.TransformListener()

    skin_topic = '/skin/contacts'
    sc_pub = rospy.Publisher(skin_topic, SkinContact)
    rospy.Subscriber('/skin/taxel_array', TaxelArray,
                     taxel_array_cb,
                     callback_args = (sc_pub, tf_lstnr))
    rospy.Subscriber('/skin/taxel_array_meka', TaxelArray,
                     taxel_array_cb,
                     callback_args = (sc_pub, tf_lstnr))
    rospy.loginfo('Started taxel_array to skin_contact!')

    rospy.spin()


