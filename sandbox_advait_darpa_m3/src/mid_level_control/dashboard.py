
import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')

import rospy
import hrl_lib.curses_menu as hcm

from std_msgs.msg import String, Bool, Empty


d = {}
d['2'] = 'set_way_point'
d['3'] = 'set_goal'

d['h'] = 'halt'
d['p'] = 'pause'
d['r'] = 'resume'
d['s'] = 'start'

d['f'] = 'reduce_force'
d['m'] = 'try_multiple'
d['o'] = 'pull_out'
d['w'] = 'reach_to_way_point'
d['g'] = 'reach_to_goal'


def ros_publish(option):
    cmd = d[option]
    if cmd == 'signal_stuck':
        stuck_pub.publish(Empty())
    elif cmd == 'halt':
        stop_pub.publish(Bool(True))
    elif cmd == 'start':
        stop_pub.publish(Bool(False))
    elif cmd == 'pause':
        pause_pub.publish(Bool(True))
    elif cmd == 'resume':
        pause_pub.publish(Bool(False))
    else:
        ros_pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('dashboard_cody')

    msg_nm = '/epc_skin/command/behavior'
    ros_pub = rospy.Publisher(msg_nm, String)
    stop_pub = rospy.Publisher('/epc/stop', Bool)
    pause_pub = rospy.Publisher('/epc/pause', Bool)

    cm = hcm.CursesMenu()
    cm.begin_menu('Here is what I can do for you today:')
    cm.add_item('1', 'Fly')
    cm.add_item('2', 'Set way point', ros_publish)
    cm.add_item('3', 'Set goal location', ros_publish)
    cm.add_item('h', 'Halt', ros_publish)
    cm.add_item('s', 'Start', ros_publish)
    cm.add_item('p', 'Pause', ros_publish)
    cm.add_item('r', 'Resume', ros_publish)
    cm.add_item('q', 'Quit', cm.exit)

    cm.add_item('f', 'reduce force', ros_publish)
    cm.add_item('m', 'try multiple times', ros_publish)
    cm.add_item('o', 'pull out', ros_publish)
    cm.add_item('w', 'reach to way point', ros_publish)
    cm.add_item('g', 'reach to goal location', ros_publish)

    cm.finish_menu('Select your option')
    cm.run()


