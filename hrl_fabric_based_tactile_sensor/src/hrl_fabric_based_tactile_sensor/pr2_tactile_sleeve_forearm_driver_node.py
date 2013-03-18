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
from sensor_msgs.msg import JointState

from pr2_tactile_sleeve_driver_node import Tactile_Sleeve

def joint_states_cb(data):
    global ignore_forearm_taxel0

    idx = data.name.index('r_wrist_flex_joint')
    ang = data.position[idx]

    if abs(ang) >  math.radians(70.):
        ignore_forearm_taxel0 = True
    else:
        ignore_forearm_taxel0 = False



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
    raw_data_forearm_pub = rospy.Publisher('pr2_fabric_forearm_sensor/taxels/raw_data',
                                           RawTaxelArray)
    rospy.Service('pr2_fabric_forearm_sensor/taxels/srv/local_coord_frames',
                  None_TransformArray, ts.local_coord_frames_forearm_cb)
    rospy.Service('pr2_fabric_forearm_sensor/taxels/srv/link_name', None_String,
                  ts.link_name_forearm_cb)


    global ignore_forearm_taxel0
    ignore_forearm_taxel0 = False
    rospy.Subscriber('/joint_states', JointState, joint_states_cb)

    rospy.init_node('fabric_tactile_sleeve_driver_node')

    baudrate = 115200
    dev1_nm = '/dev/robot/arduino1'
    dev1 = apn.setup_serial(dev1_nm, baudrate)

    for i in range(10):
        dev1.readline()
    
    rospy.loginfo('Started publishing data')


    # hack, hack, hack!
    adc_data = apn.get_adc_data(dev1, 32)
a   forearm_taxel0_dummy_val = adc_data[0] - 30

    rta2 = RawTaxelArray()
    while not rospy.is_shutdown():
        adc_data = apn.get_adc_data(dev1, 32)
        
        #forearm_vals = adc_data[0:12]
        forearm_vals = adc_data[0:13] + adc_data[14:15]
        if ignore_forearm_taxel0:
            forearm_vals[0] = forearm_taxel0_dummy_val # hack!
        rta2.val_z = forearm_vals
        raw_data_forearm_pub.publish(rta2)

    dev1.close()



