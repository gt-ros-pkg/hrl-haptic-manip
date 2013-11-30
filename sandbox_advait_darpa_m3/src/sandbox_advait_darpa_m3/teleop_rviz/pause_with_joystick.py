#!/usr/bin/python

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

def joy_cb(msg):
    global paused
    if msg.buttons[12] == 1 and paused:
        pause_pub.publish(Bool(False))
        paused = False
        rospy.sleep(0.02)

    if msg.buttons[14] == 1 and not paused:
        pause_pub.publish(Bool(True))
        paused = True
        rospy.sleep(0.02)



if __name__ == '__main__':
    rospy.init_node('dashboard_cody')

    msg_nm = '/epc_skin/command/behavior'
    ros_pub = rospy.Publisher(msg_nm, String)
    pause_pub = rospy.Publisher('/epc/pause', Bool)

    global paused
    paused = False
    rospy.Subscriber('/joy', Joy, joy_cb)
    rospy.loginfo('Running pause with joystick')
    rospy.spin()


