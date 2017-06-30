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
    def __init__(self, arm):
        self.arm = arm

        self.tar_forearm = None_TransformArrayResponse()
        self.setup_forearm_twenty_two_taxels_transforms()

        self.tar_upperarm = None_TransformArrayResponse()
        self.setup_upperarm_taxels_transforms()

        self.tar_gripper_right_link = None_TransformArrayResponse()
        self.setup_gripper_right_link_taxels_transforms()

        self.tar_gripper_left_link = None_TransformArrayResponse()
        self.setup_gripper_left_link_taxels_transforms()

        self.tar_gripper_palm = None_TransformArrayResponse()
        self.setup_gripper_palm_taxels_transforms()

        self.tar_left_pps = None_TransformArrayResponse()
        self.setup_pps_left_taxels_transforms()

        self.tar_right_pps = None_TransformArrayResponse()
        self.setup_pps_right_taxels_transforms()

        #self.tar_pps = None_TransformArrayResponse()
        #self.setup_pps_taxels_transforms()


    def setup_forearm_twenty_two_taxels_transforms(self):
        self.link_name_forearm = '/%s_forearm_link'%(self.arm)
        n_taxels = 22

        self.tar_forearm.data = [Transform()] * n_taxels # [Transform() for i in range(n_taxels)] #[None for i in range(n_taxels)]
       #idx_list = [17, 18, 3, 14, 16, 19, 7, 8,  9, 0, 4, 2, 15, 20, 6, 10, 13, 12, 5, 2, 21, 1] #1,3 # 3 is a dead taxel I think. 2 and 11 are for the gripper palm.
        idx_list = [17, 18, 4, 14, 16, 19, 8, 9, 10, 0, 5, 3, 15, 20, 7, 11, 13, 12, 6, 1, 21, 2] #1,3 # 3 is a dead taxel I think. 2 and 11 are for the gripper palm.
                  #[19, 20, 5, 16, 18, 21, 9, 10, 11, 0, 6, 4, 17, 22, 8, 12, 15, 14, 7, 9999, 23, 2]
        x_disp = [.16, .23, .3, .16, .23, .3, .16, .23, .3, .16, .23, .3, .17, .28, .17, .28, .17, .28, .17, .28, .34, .34]
        y_disp = [0., 0., 0., -0.06, -0.06, -0.06, 0., 0., 0., 0.06, 0.06, 0.06, -0.05, -0.05, -0.05, -0.05, 0.05, 0.05, 0.05, 0.05 ,-0.05, 0.05]
        z_disp = [0.04, 0.02, 0.03, 0., 0., 0., -0.05, -0.05, -0.05, 0., 0., 0., 0.04, 0.02, -0.04, -0.04, -0.04, -0.04, 0.04, 0.02, 0., 0.]

        x_ang = [0, 0, 0, -math.pi/2, -math.pi/2, -math.pi/2, math.pi, math.pi, math.pi, math.pi/2, math.pi/2, math.pi/2, -math.pi/4, -math.pi/4, -3*math.pi/4, -3*math.pi/4, 3*math.pi/4, 3*math.pi/4, math.pi/4, math.pi/4, math.radians(-30), math.radians(30)]
        y_ang = [-math.pi/4, math.radians(-15), math.radians(20), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, math.radians(-60), math.radians(-60)]
        z_ang = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = x_disp[i]
            t.translation.y = y_disp[i]
            t.translation.z = z_disp[i]

            rot_mat = tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_forearm.data[idx_list[i]] = t

    def setup_upperarm_taxels_transforms(self):
        n_circum = 4
        n_axis = 1
        self.link_name_upperarm = '/%s_upper_arm_link'%(self.arm)

        angle_along_circum = 2*math.pi / n_circum
        offset_along_circum = math.radians(0)

        n_taxels = n_circum * n_axis

        self.tar_upperarm.data = [None for i in range(n_taxels)]

        # mapping the taxels to the raw ADC list.
        # 0 - top;  	1 - left;   	2 - bottom; 	3 - right

        idx_list = [3, 2, 1, 0]
        x_disp = [.3, .3, .3, 0]
        y_disp = [0.06, 0, -0.06, 0]
        z_disp = [-0.03, -0.09, -0.03, 0]
        x_ang = [0, 0, 3*math.pi/2, 0]
        y_ang = [math.pi/2, math.pi, 0, 0]
        z_ang = [math.pi/2, 0, 0, 0]

        for i in range(n_axis):
            for j in range(n_circum):
                t = Transform()
                t.translation.x = x_disp[j]
                t.translation.y = y_disp[j]
                t.translation.z = z_disp[j]

                rot_mat = tr.Rz(z_ang[j])*tr.Ry(y_ang[j])*tr.Rx(x_ang[j])
                quat = tr.matrix_to_quaternion(rot_mat)

                t.rotation.x = quat[0]
                t.rotation.y = quat[1]
                t.rotation.z = quat[2]
                t.rotation.w = quat[3]
                self.tar_upperarm.data[idx_list[i*n_circum+j]] = t

    def setup_gripper_palm_taxels_transforms(self):
        self.link_name_gripper_palm = '/%s_gripper_palm_link'%(self.arm)
        n_taxels = 2

        self.tar_gripper_palm.data = [None for i in range(n_taxels)]
        idx_list = [0, 1]
        x_disp = [0.06, 0.06]
        y_disp = [-0.02, 0.02]
        z_disp = [0., 0.]
        x_ang = [-math.pi/2, math.pi/2]
        y_ang = [0, 0]
        z_ang = [math.pi/4, -math.pi/4]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = x_disp[i]
            t.translation.y = y_disp[i]
            t.translation.z = z_disp[i]

            rot_mat = tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_gripper_palm.data[idx_list[i]] = t

    def setup_gripper_left_link_taxels_transforms(self):
        self.link_name_gripper_left_link = '/%s_gripper_l_finger_link'%(self.arm)
        n_taxels = 4

        self.tar_gripper_left_link.data = [None for i in range(n_taxels)]
        idx_list = [0, 2, 3, 1]
        x_disp = [.03, .04, .03, 0.1]
        y_disp = [0.02, 0.04, 0.02, 0.02]
        z_disp = [0.03, 0., -0.03, 0.]
        x_ang = [0, math.pi/2, math.pi, math.pi/2]
        y_ang = [math.radians(-5), 0, math.radians(5), 0]
        z_ang = [0, math.radians(10), 0, math.pi/4]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = x_disp[i]
            t.translation.y = y_disp[i]
            t.translation.z = z_disp[i]

            rot_mat = tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_gripper_left_link.data[idx_list[i]] = t

    def setup_gripper_right_link_taxels_transforms(self):
        self.link_name_gripper_right_link = '/%s_gripper_r_finger_link'%(self.arm)
        n_taxels = 4

        self.tar_gripper_right_link.data = [None for i in range(n_taxels)]
        idx_list = [3, 2, 0, 1]
        x_disp = [.03, .04, .03, 0.1]
        y_disp = [-0.02, -0.04, -0.02, -0.02]
        z_disp = [0.03, 0., -0.03, 0.]
        x_ang = [0, -math.pi/2, math.pi, -math.pi/2]
        y_ang = [math.radians(-5), 0, math.radians(5), 0]
        z_ang = [0, math.radians(-10), 0, -math.pi/4]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = x_disp[i]
            t.translation.y = y_disp[i]
            t.translation.z = z_disp[i]

            rot_mat = tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_gripper_right_link.data[idx_list[i]] = t

    def setup_pps_right_taxels_transforms(self):
        self.link_name_right_pps = '/%s_gripper_r_finger_tip_link'%(self.arm)
        n_taxels = 3

        self.tar_right_pps.data = [None for i in range(n_taxels)]
        idx_list = [0, 1, 2]
        x_disp = [0.03, 0.012, 0.012]
        y_disp = [0.01, 0.01, 0.01]
        z_disp = [0.0, -0.011, 0.011]
        x_ang = [0., math.pi, 0.]
        y_ang = [-math.pi/2., 0., 0.]
        z_ang = [0., 0., 0.]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = x_disp[i]
            t.translation.y = y_disp[i]
            t.translation.z = z_disp[i]

            rot_mat = tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_right_pps.data[idx_list[i]] = t

    def setup_pps_left_taxels_transforms(self):
        self.link_name_left_pps = '/%s_gripper_l_finger_tip_link'%(self.arm)
        n_taxels = 3

        self.tar_left_pps.data = [None for i in range(n_taxels)]
        idx_list = [0, 1, 2]
        x_disp = [0.03, 0.012, 0.012]
        y_disp = [-0.01, -0.01, -0.01]
        z_disp = [0.0, -0.011, 0.011]
        x_ang = [0., math.pi, 0.]
        y_ang = [-math.pi/2., 0., 0.]
        z_ang = [0., 0., 0.]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = x_disp[i]
            t.translation.y = y_disp[i]
            t.translation.z = z_disp[i]

            rot_mat = tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_left_pps.data[idx_list[i]] = t

    def local_coord_frames_forearm_cb(self, req):
        return self.tar_forearm

    def local_coord_frames_upperarm_cb(self, req):
        return self.tar_upperarm

    def local_coord_frames_gripper_cb(self, req):
        return self.tar_gripper

    def local_coord_frames_gripper_right_link_cb(self, req):
        return self.tar_gripper_right_link

    def local_coord_frames_gripper_left_link_cb(self, req):
        return self.tar_gripper_left_link

    def local_coord_frames_gripper_palm_cb(self, req):
        return self.tar_gripper_palm

    def local_coord_frames_pps_left_cb(self, req):
        return self.tar_left_pps

    def local_coord_frames_pps_right_cb(self, req):
        return self.tar_right_pps

    def local_coord_frames_pps_cb(self, req):
        return self.tar_pps

    def link_name_forearm_cb(self, req):
        return self.link_name_forearm

    def link_name_upperarm_cb(self, req):
        return self.link_name_upperarm

    def link_name_gripper_cb(self, req):
        return self.link_name_gripper

    def link_name_gripper_right_link_cb(self, req):
        return self.link_name_gripper_right_link

    def link_name_gripper_left_link_cb(self, req):
        return self.link_name_gripper_left_link

    def link_name_gripper_palm_cb(self, req):
        return self.link_name_gripper_palm

    def link_name_pps_cb(self, req):
        return self.link_name_pps

    def link_name_pps_left_cb(self, req):
        return self.link_name_left_pps

    def link_name_pps_right_cb(self, req):
        return self.link_name_right_pps


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--serial_dev', action='store',
                 dest='serial_dev_name', type='string',
                 help='path to the arduino serial device')
    p.add_option('--forearm', action='store_true',
                 dest='forearm',
                 help='node for the forearm taxels of the sleeve')
    p.add_option('--upperarm', action='store_true',
                 dest='upperarm',
                 help='node for the upperarm taxels of the sleeve')
    p.add_option('--arm', action='store',
                 dest='arm', type='string',
                 help='l or r')

    opt, args = p.parse_args()

    raw_data_forearm_pub = rospy.Publisher('taxels/raw_data',
                                           RawTaxelArray)

    if opt.arm != 'r' and opt.arm != 'l':
        rospy.logerr('specify valid arm to use (l or r)')
        sys.exit()

    ts = Tactile_Sleeve(opt.arm)

    if opt.forearm:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_forearm_cb)
        rospy.Service('taxels/srv/link_name', None_String,
                      ts.link_name_forearm_cb)
        n_taxels = 4
    elif opt.upperarm:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_upperarm_cb)
        rospy.Service('taxels/srv/link_name', None_String, ts.link_name_upperarm_cb)
        n_taxels = 4
    else:
        rospy.logerr('Specify either --forearm or --upperarm')
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


