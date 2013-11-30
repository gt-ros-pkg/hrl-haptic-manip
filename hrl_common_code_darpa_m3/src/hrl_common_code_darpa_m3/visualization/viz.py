#!/usr/bin/python

import numpy as np, math

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.viz as hv
import rospy

from visualization_msgs.msg import Marker
from hrl_haptic_manipulation_in_clutter_msgs.msg import SkinContact


## arrow and text markers for a force at a particular location.
# use to visulaize simulated skin in rviz.
def get_arrow_text_markers(p, f, frame, m_id, duration, c=(1.,0.,0.,1.0)):
    t_now = rospy.Time.now()
    q = hv.arrow_direction_to_quat(f)
    arrow_len = np.linalg.norm(f) * 0.04
    #arrow_len = np.linalg.norm(f) * 0.007
    f_unit = f/np.linalg.norm(f)

    #if 'electric' not in roslib.__path__[0]:
    #    # diamondback
    #    scale = (arrow_len, 0.1, 0.1)
    #else:
    # electric
    scale = (0.20, 0.20, arrow_len)

    m1 = hv.single_marker(p, q, 'arrow', frame, scale, c, m_id = m_id,
                          duration = duration)
    m1.header.stamp = t_now

    m2 = hv.single_marker(p + f_unit * arrow_len * 1.6, q, 'text_view_facing', frame,
    #m2 = hv.single_marker(p, q, 'text_view_facing', frame,
                          (0.07, 0.07, 0.07), m_id = m_id+1,
                          duration = duration, color=c)
    m2.text = '%.1fN'%(np.linalg.norm(f))
    m2.header.stamp = t_now
    return m1, m2

def visualize_skin(sc, callback_args):
    marker_pub, display_normal_component = callback_args
    if sc.link_names == []:
        return

    n = len(sc.link_names)
    for i in range(len(sc.link_names)):
        l = sc.locations[i]
        mn = np.matrix([l.x, l.y, l.z]).T
        ff = sc.forces[i]
        f = np.matrix([ff.x, ff.y, ff.z]).T
        nn = sc.normals[i]
        nrml = np.matrix([nn.x, nn.y, nn.z]).T

        frame = sc.header.frame_id
        if opt.ac == 'green':
            c = (0.,0.5,0.,1.0)
        elif opt.ac == 'blue':
            c = (0.,0.,1.0,1.0)
        else:
            raise 'Unrecognized color'
        m1, m2 = get_arrow_text_markers(mn, f, frame,
                                        m_id = 4*i+1, duration=0.2,
                                        c=c)
        m1.header.stamp = sc.header.stamp
        m2.header.stamp = sc.header.stamp
        marker_pub.publish(m1)
        marker_pub.publish(m2)

        if display_normal_component:
            m3, m4 = get_arrow_text_markers(mn, nrml*(nrml.T*f)[0,0],
                                            frame, m_id = 4*i+3,
                                            duration=0.2,
                                            c=(1.0,0.,0,0.5))
            m3.header.stamp = sc.header.stamp
            m4.header.stamp = sc.header.stamp
            marker_pub.publish(m3)
            #marker_pub.publish(m4)



if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--arrow_color', action='store', dest='ac',
                 type='string', help='green or blue',
                 default='green')
    p.add_option('--display_normal_component', action='store_true',
                 dest='dnc', help='display an rviz marker for the normal conponent of the contact forces.')
    opt, args = p.parse_args()


    rospy.init_node('skin_visualize_publisher')
    marker_pub = rospy.Publisher('/skin/viz/contacts', Marker)
    rospy.Subscriber('/skin/contacts', SkinContact, visualize_skin,
                     callback_args = (marker_pub, opt.dnc))
    rospy.loginfo('Started visulizing skin!')
    rospy.spin()






