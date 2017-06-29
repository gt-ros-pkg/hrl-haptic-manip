#!/usr/bin/env python

import sys
import math

import rospy

import hrl_lib.transforms as tr

import hrl_fabric_based_tactile_sensor.adc_publisher_node as apn

from m3skin_ros.msg import RawTaxelArray
from geometry_msgs.msg import Transform

from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse
from m3skin_ros.srv import None_String


class Tactile_Sleeve():
    def __init__(self):
        self.tar_forearm = None_TransformArrayResponse()
        self.setup_forearm_taxels_transforms()

        self.tar_wrist = None_TransformArrayResponse()
        self.setup_wrist_taxels_transforms()

    def setup_forearm_taxels_transforms(self):
        n_circum = 4
        n_axis = 3
        self.link_name_forearm = '/wrist_LEFT'

        rad = 0.04
        dist_along_axis = 0.065
        angle_along_circum = 2*math.pi / n_circum

        offset_along_axis = 0.02
        offset_along_circum = math.radians(-45)

        n_taxels = n_circum * n_axis

        self.tar_forearm.data = [None for i in range(n_taxels)]
        # mapping the taxels to the raw ADC list.
        idx_list = [6, 9, 0, 3, 7, 10, 1, 4, 8, 11, 2, 5]
        for i in range(n_axis):
            for j in range(n_circum):
                t = Transform()
                ang = j*angle_along_circum + offset_along_circum
                t.translation.x = rad * math.cos(ang)
                t.translation.y = rad * math.sin(ang)
                t.translation.z = offset_along_axis + i * dist_along_axis

                rot_mat = tr.Rz(-ang)*tr.Ry(math.radians(-90))
                quat = tr.matrix_to_quaternion(rot_mat)

                t.rotation.x = quat[0]
                t.rotation.y = quat[1]
                t.rotation.z = quat[2]
                t.rotation.w = quat[3]
                self.tar_forearm.data[idx_list[i*n_circum+j]] = t

    def setup_wrist_taxels_transforms(self):
        self.link_name_wrist = '/handmount_LEFT'
        n_circum = 4
        dist_along_axis = 0.065
        angle_along_circum = 2*math.pi / n_circum

        offset_along_circum = math.radians(-45)

        self.tar_wrist.data = [None for i in range(13)]

        # mapping the taxels to the raw ADC list.
        idx_list = [6, 9, 2, 5]
        n_axis = 1
        rad = 0.03
        offset_along_axis = -0.04
        for i in range(n_axis):
            for j in range(n_circum):
                t = Transform()
                ang = j*angle_along_circum + offset_along_circum
                t.translation.x = rad * math.cos(ang)
                t.translation.y = rad * math.sin(ang)
                t.translation.z = offset_along_axis + i * dist_along_axis

                rot_mat = tr.Rz(-ang)*tr.Ry(math.radians(-90))
                quat = tr.matrix_to_quaternion(rot_mat)

                t.rotation.x = quat[0]
                t.rotation.y = quat[1]
                t.rotation.z = quat[2]
                t.rotation.w = quat[3]
                self.tar_wrist.data[idx_list[i*n_circum+j]] = t

        # mapping the taxels to the raw ADC list.
        idx_list = [8, 11, 0, 3, 7, 10, 1, 4]
        n_axis = 2
        rad = 0.02
        offset_along_axis = -0.17
        for i in range(n_axis):
            for j in range(n_circum):
                t = Transform()
                ang = j*angle_along_circum + offset_along_circum
                t.translation.x = rad * math.cos(ang)
                t.translation.y = rad * math.sin(ang)
                t.translation.z = offset_along_axis + i * dist_along_axis

                rot_mat = tr.Rz(-ang)*tr.Ry(math.radians(-90))
                quat = tr.matrix_to_quaternion(rot_mat)

                t.rotation.x = quat[0]
                t.rotation.y = quat[1]
                t.rotation.z = quat[2]
                t.rotation.w = quat[3]
                self.tar_wrist.data[idx_list[i*n_circum+j]] = t

        t = Transform()
        t.translation.x = 0.
        t.translation.y = 0
        t.translation.z = -0.2
        rot_mat = tr.Rx(math.radians(180))
        quat = tr.matrix_to_quaternion(rot_mat)

        t.rotation.x = quat[0]
        t.rotation.y = quat[1]
        t.rotation.z = quat[2]
        t.rotation.w = quat[3]
        self.tar_wrist.data[12] = t

    def local_coord_frames_forearm_cb(self, req):
        return self.tar_forearm

    def local_coord_frames_wrist_cb(self, req):
        return self.tar_wrist

    def link_name_forearm_cb(self, req):
        return self.link_name_forearm

    def link_name_wrist_cb(self, req):
        return self.link_name_wrist


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--wrist', action='store_true',
                 dest='wrist',
                 help='node for the wrist taxels of the sleeve')
    p.add_option('--forearm', action='store_true',
                 dest='forearm',
                 help='node for the forearm taxels of the sleeve')
    p.add_option('--serial_dev', action='store',
                 dest='serial_dev_name', type='string',
                 help='path to the arduino serial device')

    opt, args = p.parse_args()

    raw_data_forearm_pub = rospy.Publisher('taxels/raw_data',
                                           RawTaxelArray)

    ts = Tactile_Sleeve()

    if opt.forearm:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_forearm_cb)
        rospy.Service('taxels/srv/link_name', None_String,
                      ts.link_name_forearm_cb)
        n_taxels = 12
    elif opt.wrist:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_wrist_cb)
        rospy.Service('taxels/srv/link_name', None_String,
                      ts.link_name_wrist_cb)
        n_taxels = 13
    else:
        rospy.logerr('Specify either --forearm or --wrist')
        sys.exit()

    rospy.init_node('fabric_tactile_sleeve_driver_node')

    baudrate = 115200
    dev = apn.setup_serial(opt.serial_dev_name, baudrate)

    for i in range(10):
        dev.readline()

    rospy.loginfo('Started publishing data')

    rta = RawTaxelArray()
    while not rospy.is_shutdown():
        rta.val_z = apn.get_adc_data(dev, 16)[0:n_taxels]
        raw_data_forearm_pub.publish(rta)

    dev.close()
