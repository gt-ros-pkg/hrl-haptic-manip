#!/usr/bin/python

import numpy as np, math

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.viz as hv
import hrl_lib.util as ut
import rospy

from geometry_msgs.msg import Point
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
    fs = np.column_stack((ta.values_x, ta.values_y, ta.values_z))

    fmags = ut.norm(fs.T).flatten()

    if hasattr(ta, 'link_name'):
        idxs = np.where(fmags > 0.0)[0]  #was .01
    else:
        # HACK. need to calibrate the skin patch so that something
        # reasonable gets outputted.
        idxs = np.where(fmags > 0.)[0] #was 0.2

    #force_marker_scale = 0.04
    force_marker_scale = 1.0
    duration = 0.4
    for i in idxs:
        p = np.matrix(pts[i]).T
        n1 = np.matrix(nrmls[i]).T
        n2 = np.matrix(fs[i]).T

        if False:
            q1 = hv.arrow_direction_to_quat(n1)
            l1 = (n2.T * n1)[0,0] * force_marker_scale


            if 'electric' not in roslib.__path__[0]:
                scale = (l1, 0.2, 0.2)
            else:
                scale = (0.2, 0.2, 0.2)  #something weird here, was (0.2, 0.2, l1)

            scale = (0.4, 0.4, l1)

            m = hv.single_marker(p, q1, 'arrow', frame, duration=duration,
                                 scale=scale, m_id=3*i+1)
            m.header.stamp = stamp
            #markers.markers.append(m)

        if True:
            if np.linalg.norm(n2) < 0.30:
                q2 = hv.arrow_direction_to_quat(n2)
                l2 = np.linalg.norm(n2) * force_marker_scale

                m = Marker()
                m.points.append(Point(p[0,0], p[1,0], p[2,0]))
                m.points.append(Point(p[0,0]+n2[0,0], p[1,0]+n2[1,0], p[2,0]+n2[2,0]))
                m.scale = Point(0.02, 0.04, 0.0)
                m.header.frame_id = frame
                m.id = 3*i+2
                m.type = Marker.ARROW
                m.action = Marker.ADD
                m.color.r = 0.
                m.color.g = 0.8
                m.color.b = 0.
                m.color.a = 1.
                m.lifetime = rospy.Duration(duration)
                m.header.stamp = stamp
                markers.markers.append(m)

                m = hv.single_marker(p + n2/np.linalg.norm(n2) * l2 * 1.2, q2, 'text_view_facing', frame,
                                     (0.07, 0.07, 0.07), m_id = 3*i+3,
                                     duration = duration, color=(0.5,0.5,0.5,1.))
                m.text = '%.2fm'%(np.linalg.norm(n2))
                m.header.stamp = stamp
                markers.markers.append(m)

    marker_pub.publish(markers)

if __name__ == '__main__':
    rospy.init_node('taxel_array_viz_publisher')
    _ = rospy.Publisher('/skin/viz/taxel_array', Marker)
    marker_forearm_pub = rospy.Publisher('/skin/viz/bosch/forearm_taxel_array_array', MarkerArray)
    marker_upperarm_pub = rospy.Publisher('/skin/viz/bosch/upperarm_taxel_array_array', MarkerArray)
    marker_pub = rospy.Publisher('/skin/viz/taxel_array_array', MarkerArray)
    simulation_pub = rospy.Publisher('/simulation/viz/taxel_array_array', MarkerArray)

    rospy.Subscriber('/skin/bosch/forearm_taxel_array', TaxelArray,
                     visualize_taxel_array,
                     callback_args = marker_forearm_pub)

    rospy.Subscriber('/skin/bosch/upperarm_taxel_array', TaxelArray,
                     visualize_taxel_array,
                     callback_args = marker_upperarm_pub)

    rospy.Subscriber('/haptic_mpc/simulation/proximity/taxel_array', TaxelArray,
                     visualize_taxel_array,
                     callback_args = simulation_pub)

    rospy.Subscriber('/skin/taxel_array', TaxelArray,
                     visualize_taxel_array,
                     callback_args = marker_pub)
    rospy.Subscriber('/skin/taxel_array_meka', TaxelArray_Meka,
                     visualize_taxel_array,
                     callback_args = marker_pub)
    rospy.loginfo('Started visulizing taxel array!')

    rospy.spin()


