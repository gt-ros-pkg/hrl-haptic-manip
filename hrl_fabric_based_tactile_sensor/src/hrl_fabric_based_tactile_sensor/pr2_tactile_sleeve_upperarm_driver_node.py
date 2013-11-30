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

    raw_data_upperarm_pub = rospy.Publisher('pr2_fabric_upperarm_sensor/taxels/raw_data',
                                           RawTaxelArray)
    rospy.Service('pr2_fabric_upperarm_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_upperarm_cb)
    rospy.Service('pr2_fabric_upperarm_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_upperarm_cb)

    rospy.init_node('fabric_tactile_sleeve_driver_node')

    baudrate = 115200
    dev2_nm = '/dev/robot/arduino1'
    dev2 = apn.setup_serial(dev2_nm, baudrate)

    for i in range(10):
        dev2.readline()
    
    rospy.loginfo('Started publishing data and upper arm')

    rta1 = RawTaxelArray()
    rta2 = RawTaxelArray()
    rta3 = RawTaxelArray()
    rta4 = RawTaxelArray()
    while not rospy.is_shutdown():
        adc_data = apn.get_adc_data(dev2, 32)

        rta1.val_z = adc_data[4:8]
        raw_data_upperarm_pub.publish(rta1)

        #rta3.val_z = adc_data[6:]
        #raw_data_gripper_pub.publish(rta3)

        #rta2.val_z = adc_data[24:26] + adc_data[30:32]
        #raw_data_gripper_left_link_pub.publish(rta2)

        #rta3.val_z = adc_data[26:30]
        #raw_data_gripper_right_link_pub.publish(rta3)

        #rta4.val_z = adc_data[15:16] + adc_data[13:14] 
        #raw_data_gripper_palm_pub.publish(rta4)

    dev2.close()



