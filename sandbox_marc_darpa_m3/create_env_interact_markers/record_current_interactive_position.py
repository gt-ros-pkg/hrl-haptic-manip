
import roslib
roslib.load_manifest('interactive_markers')
from interactive_markers.interactive_marker_server import *
rospy.init_node('getting_msg')
import cPickle


msg = rospy.wait_for_message('/basic_controls/update_full', InteractiveMarkerInit)

pos_list = []

for i in xrange(len(msg.markers)):
    if msg.markers[i].pose.position.x < 0.2:
        x = msg.markers[i].pose.position.x
        y = msg.markers[i].pose.position.y
        z = msg.markers[i].pose.position.z
        pos_list.append([x, y, z])

file_h = open('fixed_custom_dict.pkl', 'w')
cPickle.dump(pos_list, file_h)

file_h.close()
