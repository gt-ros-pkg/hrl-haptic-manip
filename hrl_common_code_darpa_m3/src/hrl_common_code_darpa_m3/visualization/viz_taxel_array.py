#!/usr/bin/python

import numpy as np, math

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.viz as hv
import hrl_lib.util as ut
import rospy

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from hrl_haptic_manipulation_in_clutter_msgs.msg import TaxelArray
from m3skin_ros.msg import TaxelArray as TaxelArray_Meka


def visualize_taxel_array(ta, marker_pub):
    markers = MarkerArray()
    stamp = ta.header.stamp
    frame = ta.header.frame_id

    pts = np.column_stack((ta.centers_x, ta.centers_y, ta.centers_z))
    colors = np.zeros((4, pts.shape[0]))
    colors[0,:] = 243/255.0
    colors[1,:] = 132/255.0
    colors[2,:] = 0.
    colors[3,:] = 1.0
    scale = (0.005, 0.005, 0.005)

    duration = 0.
    m = hv.list_marker(pts.T, colors, scale, 'points',
                      frame, duration=duration, m_id=0)
    m.header.stamp = stamp
    markers.markers.append(m)

    # now draw non-zero forces as arrows.
    nrmls = np.column_stack((ta.normals_x, ta.normals_y, ta.normals_z))
    fs = np.column_stack((ta.forces_x, ta.forces_y, ta.forces_z))

    fmags = ut.norm(fs.T).flatten()

    if hasattr(ta, 'link_name'):
        idxs = np.where(fmags > 0.01)[0]
    else:
        # HACK. need to calibrate the skin patch so that something
        # reasonable gets outputted.
        idxs = np.where(fmags > 0.2)[0]

    force_marker_scale = 0.04
    duration = 0.02
    for i in idxs:
        p = np.matrix(pts[i]).T
        n1 = np.matrix(nrmls[i]).T
        n2 = np.matrix(fs[i]).T

        q1 = hv.arrow_direction_to_quat(n1)
        l1 = (n2.T * n1)[0,0] * force_marker_scale


        if 'electric' not in roslib.__path__[0]:
            scale = (l1, 0.2, 0.2)
        else:
            scale = (0.2, 0.2, l1)

        m = hv.single_marker(p, q1, 'arrow', frame, duration=duration,
                             scale=scale, m_id=3*i+1)
        m.header.stamp = stamp
        markers.markers.append(m)

        q2 = hv.arrow_direction_to_quat(n2)
        l2 = np.linalg.norm(n2) * force_marker_scale

        if 'electric' not in roslib.__path__[0]:
            scale = (l2, 0.2, 0.2)
        else:
            scale = (0.2, 0.2, l2)

        m = hv.single_marker(p, q2, 'arrow', frame, duration=duration,
                             scale=scale, color=(0.,0.5,0.,1.0),
                             m_id=3*i+2)
        m.header.stamp = stamp
        markers.markers.append(m)

        m = hv.single_marker(p + n2/np.linalg.norm(n2) * l2 * 1.6, q2, 'text_view_facing', frame,
                             (0.07, 0.07, 0.07), m_id = 3*i+3,
                             duration = duration, color=(0.,0.5,0.,1.))
        m.text = '%.1fN'%(np.linalg.norm(n2))
        m.header.stamp = stamp
        markers.markers.append(m)

    marker_pub.publish(markers)

if __name__ == '__main__':
    rospy.init_node('taxel_array_viz_publisher')
    _ = rospy.Publisher('/skin/viz/taxel_array', Marker)
    marker_pub = rospy.Publisher('/skin/viz/taxel_array_array', MarkerArray)

    rospy.Subscriber('/skin/taxel_array', TaxelArray,
                     visualize_taxel_array,
                     callback_args = marker_pub)
    rospy.Subscriber('/skin/taxel_array_meka', TaxelArray_Meka,
                     visualize_taxel_array,
                     callback_args = marker_pub)
    rospy.loginfo('Started visulizing taxel array!')

    rospy.spin()


