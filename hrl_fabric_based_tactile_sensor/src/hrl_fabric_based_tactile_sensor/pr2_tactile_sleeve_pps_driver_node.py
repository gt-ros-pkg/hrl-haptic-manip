#!/usr/bin/env python

import copy

import rospy

from m3skin_ros.msg import RawTaxelArray
from m3skin_ros.srv import None_TransformArray
from m3skin_ros.srv import None_String

from pr2_tactile_sleeve_driver_node import Tactile_Sleeve

from pr2_msgs.msg import PressureState


def pps_cb(msg):
    global l_fingertip, r_fingertip
    l_fingertip = copy.copy(msg.l_finger_tip)
    r_fingertip = copy.copy(msg.r_finger_tip)


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--arm', action='store',
                 dest='arm', type='string',
                 help='l or r')

    opt, args = p.parse_args()

    if opt.arm != 'r' and opt.arm != 'l':
        rospy.logerr('Unsupported arm')
        raise RuntimeError('Unsupported arm')

    ts = Tactile_Sleeve(opt.arm)

    raw_data_left_pps_pub = rospy.Publisher('pr2_pps_left_sensor/taxels/raw_data', RawTaxelArray,queue_size=1)
    raw_data_right_pps_pub = rospy.Publisher('pr2_pps_right_sensor/taxels/raw_data', RawTaxelArray,queue_size=1)

    rospy.Service('pr2_pps_left_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_pps_left_cb)
    rospy.Service('pr2_pps_left_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_pps_left_cb)

    rospy.Service('pr2_pps_right_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_pps_right_cb)
    rospy.Service('pr2_pps_right_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_pps_right_cb)

    global l_fingertip, r_fingertip
    l_fingertip = None
    r_fingertip = None

    if opt.arm == 'l':
        input_topic = '/pressure/l_gripper_motor'
    if opt.arm == 'r':
        input_topic = '/pressure/r_gripper_motor'

    rospy.Subscriber(input_topic, PressureState, pps_cb)

    rospy.init_node('pps_driver_node')

    rospy.loginfo('waiting for fingertip data')
    while r_fingertip is None:
        rospy.sleep(0.1)

    rospy.loginfo('Started publishing data')

    rta_left = RawTaxelArray()
    rta_right = RawTaxelArray()

    import time
    start = time.time()
    while not rospy.is_shutdown():
        l = l_fingertip
        r = r_fingertip

        # front, bottom, top is order of taxels
        data_left = [l[3]+l[4], l[5]+l[6], l[1]+l[2]]
        rta_left.val_z = data_left

        # front, bottom, top is order of taxels
        data_right = [r[3]+r[4], r[1]+r[2], r[5]+r[6]]
        rta_right.val_z = data_right

        raw_data_left_pps_pub.publish(rta_left)
        raw_data_right_pps_pub.publish(rta_right)

        rospy.sleep(0.02)
