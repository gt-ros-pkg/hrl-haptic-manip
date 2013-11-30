
import math

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')
from geometry_msgs.msg import Vector3

import rospy
import hrl_lib.curses_menu as hcm

from std_msgs.msg import String, Bool, Empty

pos_step = 0.03
rot_step = math.radians(45.0)

d = {}
d['i'] = [0., 0., pos_step]
d['k'] = [0., 0., -pos_step]
d['j'] = [0., pos_step, 0.]
d['l'] = [0., -pos_step, 0.]
d['u'] = [pos_step, 0., 0.]
d['o'] = [-pos_step, 0., 0.]
d['a'] = [rot_step, 0., 0.]
d['z'] = [-rot_step, 0., 0.]
d['s'] = [0., rot_step, 0.]
d['x'] = [0., -rot_step, 0.]
d['d'] = [0., 0., rot_step]
d['c'] = [0., 0., -rot_step]
d['t'] = False

def ros_publish(option):
    if option == 't':
        d[option] = not(d[option])
        cntrl_orient_pub.publish(d[option])
    else:
        delta = d[option]
        msg = Vector3(delta[0], delta[1], delta[2])

        if option == 'i' or option == 'k' or option == 'j' or option == 'l' or option == 'u' or option == 'o':
            pos_pub.publish(msg)
        else:
            rot_pub.publish(msg)
        
if __name__ == '__main__':
    rospy.init_node('keyboard_skin_teleop')

    pos_pub = rospy.Publisher('/cartesian_skin_control/position_command', Vector3)
    rot_pub = rospy.Publisher('/cartesian_skin_control/rotation_command', Vector3)
    cntrl_orient_pub = rospy.Publisher('/cartesian_skin_control/set_orient_cntrl', Bool)

    cm = hcm.CursesMenu()
    cm.begin_menu('Here is what I can do for you today:')
    cm.add_item('t', 'toggle orientation control', ros_publish)
    cm.add_item('i', 'go up', ros_publish)
    cm.add_item('k', 'go down', ros_publish)
    cm.add_item('j', 'go left', ros_publish)
    cm.add_item('l', 'go right', ros_publish)
    cm.add_item('u', 'go forward', ros_publish)
    cm.add_item('o', 'go back', ros_publish)
    cm.add_item('a', 'positive roll', ros_publish)
    cm.add_item('z', 'negative roll', ros_publish)
    cm.add_item('s', 'positive pitch', ros_publish)
    cm.add_item('x', 'negative pitch', ros_publish)
    cm.add_item('d', 'positive yaw', ros_publish)
    cm.add_item('c', 'negative yaw', ros_publish)
    cm.add_item('q', 'Quit', cm.exit)
    
    cm.finish_menu('Select your option')
    cm.run()


