#!/usr/bin/env python
import math
import rospy
import numpy as np

import hrl_fabric_based_tactile_sensor.adc_publisher_node as apn

from m3skin_ros.msg import RawTaxelArray

from std_msgs.msg import Bool

from m3skin_ros.srv import None_TransformArray
from m3skin_ros.srv import None_String

from pr2_tactile_sleeve_driver_node import Tactile_Sleeve
from sensor_msgs.msg import JointState


def joint_states_cb(data):
    global ignore_forearm_taxel13
    global ignore_forearm_taxel12
    joint_index = data.name.index(opt.arm + '_wrist_flex_joint')
    ang = data.position[joint_index]
    if abs(ang) > math.radians(70.):
        ignore_forearm_taxel13 = True
    else:
        ignore_forearm_taxel13 = False

    if abs(ang) > math.radians(90.):
        ignore_forearm_taxel12 = True
    else:
        ignore_forearm_taxel12 = False


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--arm', action='store', dest='arm', type='string', help='l or r')
    p.add_option('--skin', action='store', dest='skin', type='string', help='gray or blue')
    opt, args = p.parse_args()

    if opt.arm != 'r' and opt.arm != 'l':
        rospy.logerr('Invalid arm')
        raise RuntimeError('Invalid arm')

    if opt.skin != 'gray' and opt.skin != 'blue':
        rospy.logerr('Invalid skin')
        raise RuntimeError('Invalid skin')

    ts = Tactile_Sleeve(opt.arm)

    rospy.init_node('fabric_tactile_sleeve_driver_node')

    if opt.arm == 'l':
        raw_data_l_forearm_pub = rospy.Publisher('pr2_fabric_l_forearm_sensor/taxels/raw_data', RawTaxelArray, queue_size=1)
        rospy.Service('pr2_fabric_l_forearm_sensor/taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_forearm_cb)
        rospy.Service('pr2_fabric_l_forearm_sensor/taxels/srv/link_name', None_String,
                      ts.link_name_forearm_cb)
        l_forearm_data_valid_pub = rospy.Publisher('pr2_fabric_l_forearm_sensor/taxels/data_valid', Bool, latch=True, queue_size=1)

        raw_data_l_upperarm_pub = rospy.Publisher('pr2_fabric_l_upperarm_sensor/taxels/raw_data', RawTaxelArray, queue_size=1)
        rospy.Service('pr2_fabric_l_upperarm_sensor/taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_upperarm_cb)
        rospy.Service('pr2_fabric_l_upperarm_sensor/taxels/srv/link_name', None_String,
                      ts.link_name_upperarm_cb)
        l_upperarm_data_valid_pub = rospy.Publisher('pr2_fabric_l_upperarm_sensor/taxels/data_valid', Bool, latch=True, queue_size=1)

        l_forearm_data_valid = True
        l_upperarm_data_valid = True

        l_forearm_data_valid_pub.publish(l_forearm_data_valid)
        l_upperarm_data_valid_pub.publish(l_upperarm_data_valid)

    elif opt.arm == 'r':
        raw_data_r_forearm_pub = rospy.Publisher('pr2_fabric_r_forearm_sensor/taxels/raw_data', RawTaxelArray, queue_size=1)
        rospy.Service('pr2_fabric_r_forearm_sensor/taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_forearm_cb)
        rospy.Service('pr2_fabric_r_forearm_sensor/taxels/srv/link_name', None_String,
                      ts.link_name_forearm_cb)
        r_forearm_data_valid_pub = rospy.Publisher('pr2_fabric_r_forearm_sensor/taxels/data_valid', Bool, latch=True, queue_size=1)

        raw_data_r_upperarm_pub = rospy.Publisher('pr2_fabric_r_upperarm_sensor/taxels/raw_data', RawTaxelArray, queue_size=1)
        rospy.Service('pr2_fabric_r_upperarm_sensor/taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_upperarm_cb)
        rospy.Service('pr2_fabric_r_upperarm_sensor/taxels/srv/link_name', None_String,
                      ts.link_name_upperarm_cb)
        r_upperarm_data_valid_pub = rospy.Publisher('pr2_fabric_r_upperarm_sensor/taxels/data_valid', Bool, latch=True, queue_size=1)

        r_forearm_data_valid = False
        r_upperarm_data_valid = False
        r_forearm_data_valid_pub.publish(r_forearm_data_valid)
        r_upperarm_data_valid_pub.publish(r_upperarm_data_valid)

    global ignore_forearm_taxel13
    ignore_forearm_taxel13 = False
    global ignore_forearm_taxel12
    ignore_forearm_taxel12 = False
    rospy.Subscriber('/joint_states', JointState, joint_states_cb)

    baudrate = 115200
    dev_nm = '/dev/robot/fabric_skin_' + opt.skin
    dev = apn.setup_serial(dev_nm, baudrate)

    for i in range(10):
        dev.readline()

    rospy.loginfo('Started publishing upperarm and forearm taxels (' + opt.arm + ':' + opt.skin + ')')

    rta1 = RawTaxelArray()
    rta2 = RawTaxelArray()

    while not rospy.is_shutdown():
        adc_data = apn.get_adc_data(dev, 32)
        if adc_data == [-1]: # Hack! [-1] means 'reset me'
            dev_temp = apn.setup_serial(dev_nm, baudrate)
            if dev_temp != []:
                dev = dev_temp
        # for t in range(32):
        #    if adc_data[t] < 1000:
        #        rospy.loginfo(t)
        elif adc_data == []:
            rospy.logwarn('Data from upperarm and forearm taxels was empty ...')
            if opt.arm == 'l':
                if l_forearm_data_valid:
                    l_forearm_data_valid = False
                    l_forearm_data_valid_pub.publish(l_forearm_data_valid)
                if l_upperarm_data_valid:
                    l_upperarm_data_valid = False
                    l_upperarm_data_valid_pub.publish(l_upperarm_data_valid)
            if opt.arm == 'r':
                if r_forearm_data_valid:
                    r_forearm_data_valid = False
                    r_forearm_data_valid_pub.publish(r_forearm_data_valid)
                if r_upperarm_data_valid:
                    r_upperarm_data_valid = False
                    r_upperarm_data_valid_pub.publish(r_upperarm_data_valid)
        elif len(adc_data) != 32:
            rospy.logwarn('Data from upperarm and forearm taxels was truncated ...')
            if opt.arm == 'l':
                if l_forearm_data_valid:
                    l_forearm_data_valid = False
                    l_forearm_data_valid_pub.publish(l_forearm_data_valid)
                if l_upperarm_data_valid:
                    l_upperarm_data_valid = False
                    l_upperarm_data_valid_pub.publish(l_upperarm_data_valid)
            if opt.arm == 'r':
                if r_forearm_data_valid:
                    r_forearm_data_valid = False
                    r_forearm_data_valid_pub.publish(r_forearm_data_valid)
                if r_upperarm_data_valid:
                    r_upperarm_data_valid = False
                    r_upperarm_data_valid_pub.publish(r_upperarm_data_valid)
        else:
            if opt.skin == 'blue':
                forearm_idx = [9, 10, 11, 8, 12, 16, 18, 21, 23, 17, 22, 19, 20, 5, 7, 1, 0, 6, 4, 2, 15, 14]
                upperarm_idx = [30, 29, 31]

            elif opt.skin == 'gray':
                forearm_idx = [0, 1, 4, 2, 3, 9, 10, 13, 12, 8, 14, 15, 11, 18, 22, 19, 21, 23, 17, 20, 7, 6]
                upperarm_idx = [29, 30, 31]

            forearm_vals = np.array([adc_data[i] for i in forearm_idx])
            upperarm_vals = np.array([adc_data[i] for i in upperarm_idx] + [4095]) # + [4095]  # Hack! Hack!

            if any(forearm_vals > 4095):
                rospy.logwarn("[%s] %s (%s) forearm skin disconnected", rospy.get_name(), opt.arm, opt.skin)
                if any(upperarm_vals > 4095):
                    rospy.logwarn("[%s] %s (%s) upperarm skin disconnected", rospy.get_name(), opt.arm, opt.skin)
                    if opt.arm == 'l' and l_upperarm_data_valid:
                        l_upperarm_data_valid = False
                        l_upperarm_data_valid_pub.publish(l_upperarm_data_valid)
                    if opt.arm == 'r' and r_upperarm_data_valid:
                        r_upperarm_data_valid = False
                        r_upperarm_data_valid_pub.publish(r_upperarm_data_valid)

                if opt.arm == 'l' and l_forearm_data_valid:
                    l_forearm_data_valid = False
                    l_forearm_data_valid_pub.publish(l_forearm_data_valid)
                if opt.arm == 'r' and r_forearm_data_valid:
                    r_forearm_data_valid = False
                    r_forearm_data_valid_pub.publish(r_forearm_data_valid)


            if ignore_forearm_taxel13:
                forearm_vals[13] = 4000  # hack!
            if ignore_forearm_taxel12:
                forearm_vals[12] = 4000  # hack!

            rta1.val_z = forearm_vals
            rta2.val_z = upperarm_vals 

            
            if opt.arm == 'l':
                raw_data_l_forearm_pub.publish(rta1)
                raw_data_l_upperarm_pub.publish(rta2)
                if all(forearm_vals <= 4095) and not(l_forearm_data_valid and l_upperarm_data_valid):
                    l_forearm_data_valid = True
                    l_upperarm_data_valid = True
                    l_forearm_data_valid_pub.publish(l_forearm_data_valid)
                    l_upperarm_data_valid_pub.publish(l_upperarm_data_valid)

            elif opt.arm == 'r':
                raw_data_r_forearm_pub.publish(rta1)
                raw_data_r_upperarm_pub.publish(rta2)
                if all(forearm_vals <= 4095) and not(r_forearm_data_valid and r_upperarm_data_valid):
                    r_forearm_data_valid = True
                    r_upperarm_data_valid = True
                    r_forearm_data_valid_pub.publish(r_forearm_data_valid)
                    r_upperarm_data_valid_pub.publish(r_upperarm_data_valid)

    dev.close()
