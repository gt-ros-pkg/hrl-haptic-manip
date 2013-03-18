#!/usr/bin/python

import sys
import math, numpy as np

import roslib; roslib.load_manifest('hrl_fabric_based_tactile_sensor')
import rospy
from hrl_msgs.msg import FloatArray

import hrl_lib.util as ut
import hrl_lib.transforms as tr

import hrl_fabric_based_tactile_sensor.adc_publisher_node as apn

from m3skin_ros.msg import RawTaxelArray
from geometry_msgs.msg import Transform

from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse
from m3skin_ros.srv import None_String

from pr2_tactile_sleeve_driver_node import Tactile_Sleeve


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--arm_to_use', action='store',
                 dest='arm', type='string',
                 help='l or r')

    opt, args = p.parse_args()

    if opt.arm != 'r' and opt.arm != 'l':
        rospy.logerr('Invalid arm_to_use')
        raise RuntimeError('Invalid arm_to_use')

    ts = Tactile_Sleeve(opt.arm)

#    raw_data_gripper_pub = rospy.Publisher('pr2_fabric_gripper_sensor/taxels/raw_data',
#                                           RawTaxelArray)
#    rospy.Service('pr2_fabric_gripper_sensor/taxels/srv/local_coord_frames',
#                  None_TransformArray, ts.local_coord_frames_gripper_cb)
#    rospy.Service('pr2_fabric_gripper_sensor/taxels/srv/link_name', None_String,
#                  ts.link_name_gripper_cb)

    raw_data_gripper_right_link_pub = rospy.Publisher('pr2_fabric_gripper_right_link_sensor/taxels/raw_data',
                                                      RawTaxelArray)
    rospy.Service('pr2_fabric_gripper_right_link_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_gripper_right_link_cb)
    rospy.Service('pr2_fabric_gripper_right_link_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_gripper_right_link_cb)

    raw_data_gripper_left_link_pub = rospy.Publisher('pr2_fabric_gripper_left_link_sensor/taxels/raw_data',
                                                      RawTaxelArray)
    rospy.Service('pr2_fabric_gripper_left_link_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_gripper_left_link_cb)
    rospy.Service('pr2_fabric_gripper_left_link_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_gripper_left_link_cb)

    raw_data_gripper_palm_pub = rospy.Publisher('pr2_fabric_gripper_palm_sensor/taxels/raw_data',
                                                      RawTaxelArray)
    rospy.Service('pr2_fabric_gripper_palm_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_gripper_palm_cb)
    rospy.Service('pr2_fabric_gripper_palm_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_gripper_palm_cb)

    raw_data_forearm_pub = rospy.Publisher('pr2_fabric_forearm_sensor/taxels/raw_data',
                                           RawTaxelArray)
    rospy.Service('pr2_fabric_forearm_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_forearm_cb)
    rospy.Service('pr2_fabric_forearm_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_forearm_cb)

    rospy.init_node('fabric_tactile_sleeve_driver_node')

    baudrate = 115200
    dev2_nm = '/dev/robot/arduino3'
    dev2 = apn.setup_serial(dev2_nm, baudrate)

    for i in range(10):
        dev2.readline()
    
    rospy.loginfo('Started publishing gripper and forearm taxels')

    rta1 = RawTaxelArray()
    rta2 = RawTaxelArray()
    rta3 = RawTaxelArray()
    rta4 = RawTaxelArray()
    while not rospy.is_shutdown():
        adc_data = apn.get_adc_data(dev2, 32)

        #for t in range(32):
        #    if adc_data[t] < 1000:
        #        rospy.loginfo(t)

        forearm_vals = adc_data[0:5] + adc_data[6:16] + adc_data[17:24]
        rta1.val_z = forearm_vals
        raw_data_forearm_pub.publish(rta1)

        rta2.val_z = adc_data[26:27] + adc_data[29:30] + adc_data[31:32] + adc_data[24:25]
        raw_data_gripper_left_link_pub.publish(rta2)

        rta3.val_z = adc_data[28:29] + adc_data[30:31] + adc_data[27:28] + adc_data[25:26]
        raw_data_gripper_right_link_pub.publish(rta3)

        rta4.val_z = adc_data[5:6] + adc_data[16:17];
        raw_data_gripper_palm_pub.publish(rta4)

    dev2.close()



