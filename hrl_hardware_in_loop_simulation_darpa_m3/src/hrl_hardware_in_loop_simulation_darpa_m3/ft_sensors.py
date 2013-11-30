
import sys
import numpy as np, math
import copy

import roslib; roslib.load_manifest('hrl_hardware_in_loop_simulation_darpa_m3')
import rospy
from visualization_msgs.msg import Marker

import hrl_lib.transforms as tr
import hrl_lib.viz as hv

import force_torque.FTClient as ftc
import tf

import hrl_common_code_darpa_m3.visualization.viz as viz

class Object_FT_Sensors():

    ##
    # @param obj_id_list - unique names for the different objects.
    # should be the same name used for the object in the collision
    # map. used to correlate force and collision detection.
    def __init__(self, obj_id_list, netft_flag_list):
        self.ftc_list = []
        for oid, netft in zip(obj_id_list, netft_flag_list):
            self.ftc_list.append(ftc.FTClient(oid, netft))
        self.oid_list = copy.copy(obj_id_list)
        self.tf_lstnr = tf.TransformListener()

    # returns a dict of <object id: 3x1 np matrix of force>
    def get_forces(self, bias = True):
        f_list = []
        for i, ft_client in enumerate(self.ftc_list):
            f = ft_client.read(without_bias = not bias)
            f = f[0:3, :]

            trans, quat = self.tf_lstnr.lookupTransform('/torso_lift_link',
                                                        self.oid_list[i],
                                                        rospy.Time(0))
            rot = tr.quaternion_to_matrix(quat)
            f = rot * f
            f_list.append(-f)

        return dict(zip(self.oid_list, f_list))

#        return -f # the negative is intentional (Advait, Nov 24. 2010.)

    def bias_fts(self):
        for ftcl in self.ftc_list:
            ftcl.bias()


if __name__ == '__main__':
    rospy.init_node('force_visualize_test')
    marker_pub = rospy.Publisher('/skin/viz_marker', Marker)

    fts = Object_FT_Sensors()
    fts.bias_fts()

    # for testing, this loops shows the force vector at the tip of the
    # PR2 end effector.
    import hrl_pr2_lib.pr2_arms as pa
    pr2_arms = pa.PR2Arms()
    r_arm, l_arm = 0, 1
    arm = r_arm

    while not rospy.is_shutdown():
        f = fts.get_forces()[0]
        p, r = pr2_arms.end_effector_pos(arm)
        m1, m2 = viz.get_arrow_text_markers(p, f, 'torso_lift_link', 0, 1.)
        marker_pub.publish(m1)
        marker_pub.publish(m2)
        rospy.sleep(0.1)









