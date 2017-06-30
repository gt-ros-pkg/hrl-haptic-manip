#!/usr/bin/env python

import numpy as np

import rospy

from std_msgs.msg import Bool

import hrl_fabric_based_tactile_sensor.adc_publisher_node as apn

from m3skin_ros.msg import RawTaxelArray

from m3skin_ros.srv import None_TransformArray
from m3skin_ros.srv import None_String

from pr2_tactile_sleeve_driver_node import Tactile_Sleeve


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--arm', action='store',
                 dest='arm', type='string',
                 help='<l> or <r> or <base>')

    opt, args = p.parse_args()

    if opt.arm != 'r' and opt.arm != 'l' and opt.arm != 'base':
        rospy.logerr('Invalid arm/base to use')
        raise RuntimeError('Invalid arm/base to use')

    rospy.init_node('fabric_tactile_sleeve_driver_node')

    ts = Tactile_Sleeve(opt.arm)

    raw_data_base_pub = rospy.Publisher('pr2_fabric_base_sensor/taxels/raw_data', RawTaxelArray, queue_size=1)
    rospy.Service('pr2_fabric_base_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_base_cb)
    rospy.Service('pr2_fabric_base_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_base_cb)
    data_valid_pub = rospy.Publisher('pr2_fabric_base_sensor/taxels/data_valid', Bool, latch=True, queue_size=1)

    baudrate = 115200
    dev_nm = '/dev/robot/fabric_skin_' + opt.arm
    dev = apn.setup_serial(dev_nm, baudrate)

    for i in range(10):
        dev.readline()

    rospy.loginfo('[%s] Started: publishing data' % rospy.get_name())

    rta = RawTaxelArray()
    data_valid = True
    data_valid_pub.publish(data_valid)

    while not rospy.is_shutdown():
        adc_data = apn.get_adc_data(dev, 32)
        if adc_data == [-1]: # Hack! [-1] is code for 'reset me'
            dev_temp = apn.setup_serial(dev_nm, baudrate)
            if dev_temp != []:
                dev = dev_temp
        elif adc_data == []:
            rospy.logwarn('[%s] Data Invalid: No taxel data from base', rospy.get_name())
            if data_valid:
                data_valid = False
                data_valid_pub.publish(data_valid)
        elif len(adc_data) != 32:
            rospy.logwarn('[%s] Recieved suspect data from base', rospy.get_name())
            if data_valid:
                data_valid = False
                data_valid_pub.publish(data_valid)
        else:
            base_idx = [13, 8, 6, 5, 4, 3, 1, 2, 9, 10, 11, 12]
            base_vals = [adc_data[i] for i in base_idx]
            if any(np.array(base_vals) > 4095):
                if data_valid:
                    rospy.logwarn('[%s] Data Invalid: Base ADC board disconnected.', rospy.get_name())
                    data_valid = False
                    data_valid_pub.publish(data_valid)
            elif not data_valid:
                    data_valid = True
                    data_valid_pub.publish(data_valid)

            rta.val_z = base_vals
            raw_data_base_pub.publish(rta)

    dev.close()
