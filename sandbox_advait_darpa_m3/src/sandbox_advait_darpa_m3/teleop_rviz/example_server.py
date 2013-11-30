
#
# to run:
#   python example_server.py
#
# In rviz: ensure that fixed frame is /world
#
#

import math, numpy as np

import interactive_marker_util as imu

import roslib; roslib.load_manifest('interactive_markers')
import rospy

import interactive_markers.interactive_marker_server as ims
import interactive_markers.menu_handler as mh

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerFeedback, InteractiveMarkerControl
from geometry_msgs.msg import PointStamped


def feedback_rviz_cb(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        pp = feedback.pose.position
        print 'Position of the marker: (%.2f, %.2f, %.2f)'%(pp.x, pp.y, pp.z)
    server.applyChanges()

def feedback_menu_handler(feedback):
    rospy.loginfo('You clicked: ' + str(feedback.menu_entry_id))


if __name__ == '__main__':
    rospy.init_node('basic_controls')

    server = ims.InteractiveMarkerServer('basic_controls')

    pos = np.matrix([0.,0.,0.]).T
    ps = PointStamped()
    ps.header.frame_id = '/world'
    ps.point.x = 0
    ps.point.y = 0
    ps.point.z = 0

    #im = imu.make_3dof_marker_position(ps, 1.0, (0.6, 0.6, 0., 1.), 'cube')

    im = imu.make_6dof_marker(False, ps, 1.0, (0.6, 0.6, 0., 1.), 'cube')
    #im = imu.make_6dof_marker(False, ps, 1.0, (0.6, 0.6, 0., 1.), 'sphere')
    im.name = 'test_6dof'
    im.description = 'Test Marker Description'
    server.insert(im, feedback_rviz_cb)
    server.applyChanges()

    menu_handler = mh.MenuHandler()
    menu_handler.insert('First Entry', callback = feedback_menu_handler)
    menu_handler.insert('Second Entry', callback = feedback_menu_handler)
    imu.add_menu_handler(im, menu_handler, server)


    rospy.loginfo('Example interactive marker server started')
    rospy.spin()



