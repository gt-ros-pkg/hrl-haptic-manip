#!/usr/bin/env python

import rospy

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
                 help='l or r')

    opt, args = p.parse_args()

    if opt.arm != 'r' and opt.arm != 'l':
        rospy.logerr('Invalid arm')
        raise RuntimeError('Invalid arm')

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

    raw_data_gripper_left_link_pub = rospy.Publisher('pr2_fabric_gripper_left_link_sensor/taxels/raw_data', RawTaxelArray)
    rospy.Service('pr2_fabric_gripper_left_link_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_gripper_left_link_cb)
    rospy.Service('pr2_fabric_gripper_left_link_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_gripper_left_link_cb)

    raw_data_gripper_palm_pub = rospy.Publisher('pr2_fabric_gripper_palm_sensor/taxels/raw_data', RawTaxelArray)
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
    dev2_nm = '/dev/robot/skin_arduino_forearm'
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

        # for t in range(32):
        #    if adc_data[t] < 1000:
        #        rospy.loginfo(t)
        if adc_data == []:
            rospy.logwarn('Data from forearm and gripper taxels was empty ...')
        else:
            forearm_vals = adc_data[0:3] + adc_data[4:13] + adc_data[14:24]
            rta1.val_z = forearm_vals
            raw_data_forearm_pub.publish(rta1)

            rta2.val_z = adc_data[28:32]  # + adc_data[1:2] + adc_data[1:2] + adc_data[1:2]
            raw_data_gripper_left_link_pub.publish(rta2)

            rta3.val_z = adc_data[24:28]  # + adc_data[1:2] + adc_data[1:2] + adc_data[1:2]
            raw_data_gripper_right_link_pub.publish(rta3)

            rta4.val_z = adc_data[3:4] + adc_data[13:14]
            raw_data_gripper_palm_pub.publish(rta4)

    dev2.close()
