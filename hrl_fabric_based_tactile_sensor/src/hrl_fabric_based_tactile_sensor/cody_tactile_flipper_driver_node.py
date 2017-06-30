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


class Tactile_Flipper():
    def __init__(self):
        self.tar_mid = None_TransformArrayResponse()
        self.setup_mid_taxels_transforms()
        self.tar_connector = None_TransformArrayResponse()
        self.setup_connector_taxels_transforms()
        self.tar_tip = None_TransformArrayResponse()
        self.setup_tip_taxels_transforms()

    def setup_tip_taxels_transforms(self):
        self.link_name_tip = '/handmount_RIGHT'
        n_taxels = 29

        self.tar_tip.data = [Transform()] * n_taxels  # [Transform() for i in range(n_taxels)] #[None for i in range(n_taxels)]
        idx_list = range(0, 29)

        x_disp = [0.0214, 0.0155, 0.0096, 0.0214, 0.0155, 0.0, 0.0, 0.0, -0.0214, -0.0155, -0.0096, -0.0214, -0.0155, 0.0, 0.0, 0.0, -0.03724, -0.03217, -0.03724, -0.03217, -0.01867, -0.01615, 0.01867, 0.01615, 0.0, -0.0120, 0.0120, -0.0120, 0.0120]
        y_disp = [-0.0098, -0.0076, 0.0, 0.0098, 0.0076, 0.0282, 0.0246, 0.0211, 0.0098, 0.0076, 0.0, -0.0098, -0.0076, -0.0282, -0.0246, -0.0211, -0.01793, -0.01413, 0.01793, 0.01413, -0.0360, -0.02846, -0.02846, -0.036, 0.0, -0.0195, -0.0195, 0.0195, 0.0195]
        z_disp = [-0.2208, -0.2385, -0.2562, -0.2208, -0.2385, -0.2174, -0.2315, -0.2457, -0.2208, -0.2385, -0.2562, -0.2208, -0.2385, -0.2174, -0.2315, -0.2457, -0.13103, -0.1513, -0.13103, -0.1513, -0.13067, -0.15077, -0.13067, -0.15077, -0.27, -0.2384, -0.2384, -0.2384, -0.2384]

        x_ang = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0]
        y_ang = [-1.8900, -1.8900, -1.8900, -1.8900, -1.8900, -1.8100, -1.8100, -1.8100, 1.8900, 1.8900, 1.8900, 1.8900, 1.8900, 1.8100, 1.8100, 1.8100, math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.0, 0.0, 0.0, 0.0, math.pi, -1.8900, -1.8900, -1.8900, -1.890]
        z_ang = [0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5*math.pi/4, math.pi/4, 5*math.pi/4, -math.pi/4]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = sum((tr.Rz(-120*math.pi/180)*[[x_disp[i]], [y_disp[i]], [z_disp[i]]])[0].tolist(), [])[0]
            t.translation.y = sum((tr.Rz(-120*math.pi/180)*[[x_disp[i]], [y_disp[i]], [z_disp[i]]])[1].tolist(), [])[0]
            t.translation.z = sum((tr.Rz(-120*math.pi/180)*[[x_disp[i]], [y_disp[i]], [z_disp[i]]])[2].tolist(), [])[0]

            rot_mat = tr.Rz(-120*math.pi/180)*tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_tip.data[idx_list[i]] = t

    def setup_mid_taxels_transforms(self):
        self.link_name_mid = '/handmount_RIGHT'
        n_taxels = 28

        self.tar_mid.data = [Transform()] * n_taxels  # [Transform() for i in range(n_taxels)] #[None for i in range(n_taxels)]
        idx_list = range(0, 28)

        # x_disp = [0.02500, 0.02500, 0.02500, 0.02500, 0.00854, 0.00854, -0.00854, -0.00854, -0.02500, -0.02500, -0.02500, -0.02500, -0.00850, -0.00854, 0.00854, 0.00854, -0.02140, -0.02140, 0.02140, 0.02140, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # y_disp = [-0.01102, -0.01103, 0.01100, 0.01103, 0.03000, 0.03000, 0.03000, 0.03000, 0.01102, 0.01103, -0.01102, -0.01103, -0.03000, -0.03000, -0.03000, -0.03000, 0.02640, -0.02640, -0.02640, 0.02640, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # z_disp = [-0.06000, -0.08000, -0.06000, -0.08000, -0.06000, -0.08000, -0.06000, -0.08000, -0.06000, -0.08000, -0.06000, -0.08000, -0.06000, -0.08000, -0.06000, -0.08000, -0.07000, -0.07000, -0.07000, -0.07000, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # x_ang = [0.00, 0.00, 0.00, 0.00, math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.00, 0.00, 0.00, 0.00, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # y_ang = [-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0.00, 0.00, 0.00, 0.00, math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # z_ang = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, -5*math.pi/4, math.pi/4, -5*math.pi/4, math.pi/4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        x_disp = [0.02500, 0.02500, 0.02500, 0.02500, 0.00854, 0.00854, -0.00854, -0.00854, -0.02500, -0.02500, -0.02500, -0.02500, -0.00850, -0.00854, 0.00854, 0.00854, -0.02140, -0.02140, 0.02140, 0.02140, 0.03724, 0.03217, 0.03724, 0.03217, 0.01867, 0.01615, -0.01867, -0.01615]
        y_disp = [-0.01102, -0.01103, 0.01100, 0.01103, 0.03000, 0.03000, 0.03000, 0.03000, 0.01102, 0.01103, -0.01102, -0.01103, -0.03000, -0.03000, -0.03000, -0.03000, 0.02640, -0.02640, -0.02640, 0.02640, -0.01793, -0.01413, 0.01793, 0.01413, 0.036, 0.02846, 0.036, 0.02846]
        z_disp = [-0.18, -0.2, -0.18, -0.2, -0.18, -0.2, -0.18, -0.2, -0.18, -0.2, -0.18, -0.2, -0.18, -0.2, -0.18, -0.2, -0.19, -0.19, -0.19, -0.19, -0.14067, -0.16077, -0.14067, -0.16077, -0.14103, -0.1613, -0.14103, -0.1613]

        x_ang = [0.00, 0.00, 0.00, 0.00, math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.00, 0.00, 0.00, 0.00, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, math.pi/2, 0.0, 0.0, 0.0, 0.0, math.pi/2, math.pi/2, math.pi/2, math.pi/2]
        y_ang = [-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0.00, 0.00, 0.00, 0.00, math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0]
        z_ang = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, -5*math.pi/4, math.pi/4, -5*math.pi/4, math.pi/4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = sum((tr.Rz(-120*math.pi/180)*[[x_disp[i]], [y_disp[i]], [z_disp[i]]])[0].tolist(), [])[0]
            t.translation.y = sum((tr.Rz(-120*math.pi/180)*[[x_disp[i]], [y_disp[i]], [z_disp[i]]])[1].tolist(), [])[0]
            t.translation.z = sum((tr.Rz(-120*math.pi/180)*[[x_disp[i]], [y_disp[i]], [z_disp[i]]])[2].tolist(), [])[0]

            rot_mat = tr.Rz(-120*math.pi/180)*tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_mid.data[idx_list[i]] = t

    def setup_connector_taxels_transforms(self):
        self.link_name_connector = '/handmount_RIGHT'
        n_taxels = 12

        self.tar_connector.data = [Transform()] * n_taxels  # [Transform() for i in range(n_taxels)] #[None for i in range(n_taxels)]
        idx_list = range(0, 12)

        x_disp = [0.0355, 0.0205, -0.0355, -0.0205, 0.0396, -0.0106, -0.0396, 0.0106, 0.0396, -0.0106, -0.0396, 0.0106]
        y_disp = [0.0205, -0.0355, -0.0205, 0.0355, -0.0106, -0.0396, 0.0106, 0.0396, -0.0106, -0.0396, 0.0106, 0.0396]
        z_disp = [-0.105, -0.105, -0.105, -0.105, -0.081, -0.081, -0.081, -0.081, -0.053, -0.053, -0.053, -0.053]

        x_ang = [0, -math.pi/2, 0, math.pi/2, 0, -math.pi/2, 0, math.pi/2, 0, -math.pi/2, 0, math.pi/2]
        y_ang = [-math.pi/2, 0, math.pi/2, 0, math.pi/2, 0, -math.pi/2, 0, -math.pi/2, 0, math.pi/2, 0]
        z_ang = [0, 0, 0, 0, 5*math.pi/4, math.pi/4, 5*math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = x_disp[i]
            t.translation.y = y_disp[i]
            t.translation.z = z_disp[i]

            rot_mat = tr.Rz(-30*math.pi/180)*tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_connector.data[idx_list[i]] = t

    def local_coord_frames_connector_cb(self, req):
        return self.tar_connector

    def link_name_connector_cb(self, req):
        return self.link_name_connector

    def local_coord_frames_mid_cb(self, req):
        return self.tar_mid

    def link_name_mid_cb(self, req):
        return self.link_name_mid

    def local_coord_frames_tip_cb(self, req):
        return self.tar_tip

    def link_name_tip_cb(self, req):
        return self.link_name_tip


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--mid', action='store_true',
                 dest='mid',
                 help='node for the mid taxels of the flipper')
    p.add_option('--tip', action='store_true',
                 dest='tip',
                 help='node for the tip taxels of the flipper')
    p.add_option('--connector', action='store_true',
                 dest='connector',
                 help='node for the connector taxels of the flipper')
    p.add_option('--serial_dev', action='store',
                 dest='serial_dev_name', type='string',
                 help='path to the arduino serial device')

    opt, args = p.parse_args()

    raw_data_mid_pub = rospy.Publisher('taxels/raw_data', RawTaxelArray)

    raw_data_connector_pub = rospy.Publisher('taxels/raw_data', RawTaxelArray)

    raw_data_tip_pub = rospy.Publisher('taxels/raw_data', RawTaxelArray)

    tFlip = Tactile_Flipper()

    if opt.mid:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, tFlip.local_coord_frames_mid_cb)
        rospy.Service('taxels/srv/link_name', None_String,
                      tFlip.link_name_mid_cb)
        n_taxels = 28
    elif opt.tip:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, tFlip.local_coord_frames_tip_cb)
        rospy.Service('taxels/srv/link_name', None_String,
                      tFlip.link_name_tip_cb)
        n_taxels = 29
    elif opt.connector:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, tFlip.local_coord_frames_connector_cb)
        rospy.Service('taxels/srv/link_name', None_String,
                      tFlip.link_name_connector_cb)
        n_taxels = 12
    else:
        rospy.logerr('Specify --mid or --tip or --connector')
        sys.exit()

    rospy.init_node('fabric_tactile_flipper_driver_node')

    baudrate = 115200
    dev = apn.setup_serial(opt.serial_dev_name, baudrate)

    for i in range(10):
        dev.readline()

    rospy.loginfo('Started publishing data')

    mid_rta = RawTaxelArray()
    tip_rta = RawTaxelArray()
    connector_rta = RawTaxelArray()
    while not rospy.is_shutdown():
        adc_data = apn.get_adc_data(dev, 32)
        if opt.mid:
            mid_rta.val_z = mid_vals = adc_data[0:20] + adc_data[24:32]
            # mid_rta.val_z = mid_vals = adc_data[0:20]
            raw_data_mid_pub.publish(mid_rta)
        elif opt.tip:
            tip_rta.val_z = tip_vals = adc_data[0:29]
            raw_data_tip_pub.publish(tip_rta)
        elif opt.connector:
            connector_rta.val_z = connector_vals = adc_data[0:4] + adc_data[8:16]
            raw_data_connector_pub.publish(connector_rta)

    dev.close()
