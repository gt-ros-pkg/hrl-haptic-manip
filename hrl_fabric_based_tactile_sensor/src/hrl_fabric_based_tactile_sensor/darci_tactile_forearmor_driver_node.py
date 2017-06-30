#!/usr/bin/env python

import sys
import math
import numpy as np

import rospy

import hrl_lib.transforms as tr

import hrl_fabric_based_tactile_sensor.adc_publisher_node as apn

from m3skin_ros.msg import RawTaxelArray
from geometry_msgs.msg import Transform

from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse
from m3skin_ros.srv import None_String


class Tactile_Sleeve():
    def __init__(self):
        self.tar_front = None_TransformArrayResponse()
        self.setup_front_taxels_transforms()

        self.tar_back = None_TransformArrayResponse()
        self.setup_back_taxels_transforms()

        self.tar_edge = None_TransformArrayResponse()
        self.setup_edge_taxels_transforms()

    def setup_front_taxels_transforms(self):
        self.link_name_front = '/wrist_LEFT'
        n_taxels = 24

        xOffset = 0.0
        yOffset = 0.0
        zOffset = 226.0*.001

        self.tar_front.data = [Transform()] * n_taxels  # [Transform() for i in range(n_taxels)] #[None for i in range(n_taxels)]
        idx_list = range(0, 24)

        armor_offset = 0.0015
        arc_lengths = []
        arc_lengths = np.array([-59.97*.001, -27*.001, 0*.001, 27*.001, 59.94*.001])
        flat_z = []

        flat_z = np.array([-9.5*.001, -(9.+22*1)*.001, -(9.+22*2)*.001, -(9.+22*3)*.001, -(9.+22*4)*.001, -(9.+22*5)*.001, -(9.+22*5+21.5)*.001, -180*.001])
        radius = []  # most of these are 48.3 mm, edges are 50.0 mm
        radius = np.array([48.3*.001, 40*.001])
        flag_side = []

        x_dist_local = []
        y_dist_local = []
        z_dist_local = []
        flag_side = 'front'
        x_dist_local = np.zeros([8, 3])
        y_dist_local = np.zeros([8, 3])
        z_dist_local = np.zeros([8, 3])
        for i in [1, 2, 3]:
            x_dist_local[:, i-1] = radius[0]*math.cos(arc_lengths[i]/radius[0]) + armor_offset
            y_dist_local[:, i-1] = radius[0]*math.sin(arc_lengths[i]/radius[0])
            z_dist_local[:, i-1] = flat_z


#                x_dist_local = -radius[i]*math.cos(arc_lengths[i]/radius[i]) - armor_offset

        x_disp_temp = []
        y_disp_temp = []
        z_disp_temp = []

        for i in xrange(3):
            for j in xrange(8):
                x_disp_temp.append(x_dist_local[j, i]+xOffset)
                y_disp_temp.append(y_dist_local[j, i]+yOffset)
                z_disp_temp.append(z_dist_local[j, i]+zOffset)
#        x_disp_temp = [(i + xOffset) for i in x_dist_local]
#       y_disp_temp = [(i + yOffset) for i in y_dist_local]
#      z_disp_temp = [(i + zOffset) for i in z_dist_local]
#        y_disp_temp.append(np.array(y_dist_local)[i] + yOffset).tolist()) for i in y_dist_local
#       z_disp_temp.append(np.array(z_dist_local)[i] + zOffset).tolist()) for i in z_dist_local

        x_disp = x_disp_temp
        y_disp = y_disp_temp
        z_disp = z_disp_temp
#        print len(x_disp)
#        print x_disp

        z_ang_temp = np.zeros([8, 3])
        z_ang_temp[:, 0] = 23.61*math.pi/180
        z_ang_temp[:, 1] = 0*math.pi/180
        z_ang_temp[:, 2] = -23.61*math.pi/180

# THE ANGLE FOR THE EDGE TAXELS IS ~66 DEGREES.

        x_ang = [0]*24
        y_ang = [math.pi/2]*24
        y_ang[7] = 3*math.pi/4
        y_ang[15] = 3*math.pi/4
        y_ang[23] = 3*math.pi/4
        z_ang = []
        for i in xrange(3):
            for j in xrange(8):
                z_ang.append(z_ang_temp[j, i])


        # x_ang = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0]
        # y_ang = [-1.8900, -1.8900, -1.8900, -1.8900, -1.8900, -1.8100, -1.8100, -1.8100, 1.8900, 1.8900, 1.8900, 1.8900, 1.8900, 1.8100, 1.8100, 1.8100, math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.0, 0.0, 0.0, 0.0, math.pi, -1.8900, -1.8900, -1.8900, -1.890]
        # z_ang = [0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5*math.pi/4, math.pi/4, 5*math.pi/4, -math.pi/4]

        for i in range(n_taxels):
            t = Transform()
            t.translation.x = sum((tr.Rz(360*math.pi/180)*[[x_disp[i]], [y_disp[i]], [z_disp[i]]])[0].tolist(), [])[0]
            t.translation.y = sum((tr.Rz(360*math.pi/180)*[[x_disp[i]], [y_disp[i]], [z_disp[i]]])[1].tolist(), [])[0]
            t.translation.z = sum((tr.Rz(360*math.pi/180)*[[x_disp[i]], [y_disp[i]], [z_disp[i]]])[2].tolist(), [])[0]

            rot_mat = tr.Rz(180*math.pi/180)*tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_front.data[idx_list[i]] = t

    def setup_back_taxels_transforms(self):
        self.link_name_back = '/wrist_LEFT'
        n_taxels = 28

        self.tar_back.data = [Transform()] * n_taxels  # [Transform() for i in range(n_taxels)] #[None for i in range(n_taxels)]
        idx_list = range(0, 28)  #START HERE
#        print idx_list
#        print "idx_list_length: ", len(idx_list)


        x_disp = [0.0]*28
        y_disp = [0.0]*28
        z_disp = [0.0]*28

        x_ang = [0]*28
        y_ang = [0]*28
        z_ang = [0]*28

        xOffset = 0.0
        yOffset = 0.0
        zOffset = 226.0*.001


        armor_offset = 0.0015
        arc_lengths = []
        arc_lengths = -1*np.array([-59.97*.001,-27*.001,0*.001,27*.001,59.94*.001])
        flat_z = []

        flat_z = np.array([-9.5*.001,-(9.+22*1)*.001,-(9.+22*2)*.001,-(9.+22*3)*.001,-(9.+22*4)*.001,-(9.+22*5)*.001,-(9.+22*5+21.5)*.001,-180*.001])
        radius = []  #most of these are 48.3 mm, edges are 50.0 mm
        radius = np.array([48.3*.001,50*.001])
        flag_side =[]

        x_dist_local = []
        y_dist_local = []
        z_dist_local = []
        flag_side = 'back'
        x_dist_local = np.zeros([8,4])
        y_dist_local = np.zeros([8,4])
        z_dist_local = np.zeros([8,4])
        for i in [1,2,3]:
            x_dist_local[:,i-1] = radius[0]*math.cos(arc_lengths[i]/radius[0]+math.pi) - armor_offset
            y_dist_local[:,i-1] = radius[0]*math.sin(arc_lengths[i]/radius[0]+math.pi)
            z_dist_local[:,i-1] = flat_z

        last_taxels_z = np.array([-200*.001]*8)
        z_dist_local[:,3] = last_taxels_z
        x_dist_local[2,3] = radius[1]*math.cos(arc_lengths[4]/radius[1]) + armor_offset
        y_dist_local[2,3] = radius[1]*math.sin(arc_lengths[4]/radius[1])
        x_dist_local[3,3] = radius[1]*math.cos(arc_lengths[0]/radius[1]) + armor_offset
        y_dist_local[3,3] = radius[1]*math.sin(arc_lengths[0]/radius[1])
        x_dist_local[1,3] = radius[1]*math.cos(arc_lengths[4]/radius[1]+math.pi) - armor_offset
        y_dist_local[1,3] = radius[1]*math.sin(arc_lengths[4]/radius[1]+math.pi)
        x_dist_local[0,3] = radius[1]*math.cos(arc_lengths[0]/radius[1]+math.pi) - armor_offset
        y_dist_local[0,3] = radius[1]*math.sin(arc_lengths[0]/radius[1]+math.pi)



#                x_dist_local = -radius[i]*math.cos(arc_lengths[i]/radius[i]) - armor_offset

        x_disp_temp = []
        y_disp_temp = []
        z_disp_temp = []

        for i in xrange(3):
            for j in xrange(8):
                x_disp_temp.append(x_dist_local[j,i]+xOffset)
                y_disp_temp.append(y_dist_local[j,i]+yOffset)
                z_disp_temp.append(z_dist_local[j,i]+zOffset)


        for j in xrange(4):
            x_disp_temp.append(x_dist_local[j,3]+xOffset)
            y_disp_temp.append(y_dist_local[j,3]+yOffset)
            z_disp_temp.append(z_dist_local[j,3]+zOffset)
        print 'x_disp: ', x_disp_temp
        print 'x_disp length: ', len(x_disp_temp)
        print 'y_disp: ',y_disp_temp
        print 'y_disp length: ', len(y_disp_temp)

#
#        x_disp_temp = [(i + xOffset) for i in x_dist_local]
 #       y_disp_temp = [(i + yOffset) for i in y_dist_local]
  #      z_disp_temp = [(i + zOffset) for i in z_dist_local]
#        y_disp_temp.append(np.array(y_dist_local)[i] + yOffset).tolist()) for i in y_dist_local
 #       z_disp_temp.append(np.array(z_dist_local)[i] + zOffset).tolist()) for i in z_dist_local

        x_disp = x_disp_temp
        y_disp = y_disp_temp
        z_disp = z_disp_temp
#        print len(x_disp)
#        print x_disp

        z_ang_temp = np.zeros([8,4])
        z_ang_temp[:,0] = (180-23.61)*math.pi/180
        z_ang_temp[:,1] = (180+0)*math.pi/180
        z_ang_temp[:,2] = (180+23.61)*math.pi/180
        z_ang_temp[2,3] = 66*math.pi/180
        z_ang_temp[3,3] = -66*math.pi/180
        z_ang_temp[1,3] = (180+66)*math.pi/180
        z_ang_temp[0,3] = (180-66)*math.pi/180

# THE ANGLE FOR THE EDGE TAXELS IS ~66 DEGREES.



        x_ang = [0]*28
        y_ang = [math.pi/2]*28
        y_ang[7] = 3*math.pi/4
        y_ang[15] = 3*math.pi/4
        y_ang[23] = 3*math.pi/4
        y_ang[24] = 3*math.pi/4
        y_ang[25] = 3*math.pi/4
        y_ang[26] = 3*math.pi/4
        y_ang[27] = 3*math.pi/4
        z_ang = []
        for i in xrange(3):
            for j in xrange(8):
                z_ang.append(z_ang_temp[j,i])
        for i in [3]:
            for j in xrange(4):
                z_ang.append(z_ang_temp[j,i])

        print "z_ang_length: ", len(z_ang)
        print "y_ang_length: ", len(y_ang)
        print "x_disp_length: ", len(x_disp)
        print "z_disp_length: ", len(y_disp)


        # x_disp = [0.0214, 0.0155, 0.0096, 0.0214, 0.0155, 0.0, 0.0, 0.0, -0.0214, -0.0155, -0.0096, -0.0214, -0.0155, 0.0, 0.0, 0.0, -0.03724, -0.03217, -0.03724, -0.03217, -0.01867, -0.01615, 0.01867, 0.01615, 0.0, -0.0120, 0.0120, -0.0120, 0.0120]
        # y_disp = [-0.0098, -0.0076, 0.0, 0.0098, 0.0076, 0.0282, 0.0246, 0.0211, 0.0098, 0.0076, 0.0, -0.0098, -0.0076, -0.0282, -0.0246, -0.0211, -0.01793, -0.01413, 0.01793, 0.01413, -0.0360, -0.02846, -0.02846, -0.036, 0.0, -0.0195, -0.0195, 0.0195, 0.0195]
        # z_disp = [-0.2208, -0.2385, -0.2562, -0.2208, -0.2385, -0.2174, -0.2315, -0.2457, -0.2208, -0.2385, -0.2562, -0.2208, -0.2385, -0.2174, -0.2315, -0.2457, -0.13103, -0.1513, -0.13103, -0.1513, -0.13067, -0.15077, -0.13067, -0.15077, -0.27, -0.2384, -0.2384, -0.2384, -0.2384]


        # x_ang = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0]
        # y_ang = [-1.8900, -1.8900, -1.8900, -1.8900, -1.8900, -1.8100, -1.8100, -1.8100, 1.8900, 1.8900, 1.8900, 1.8900, 1.8900, 1.8100, 1.8100, 1.8100, math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.0, 0.0, 0.0, 0.0, math.pi, -1.8900, -1.8900, -1.8900, -1.890]
        # z_ang = [0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5*math.pi/4, math.pi/4, 5*math.pi/4, -math.pi/4]


        for i in range(n_taxels):
            t = Transform()
            t.translation.x = sum((tr.Rz(360*math.pi/180)*[[x_disp[i]],[y_disp[i]],[z_disp[i]]])[0].tolist(),[])[0]
            t.translation.y = sum((tr.Rz(360*math.pi/180)*[[x_disp[i]],[y_disp[i]],[z_disp[i]]])[1].tolist(),[])[0]
            t.translation.z = sum((tr.Rz(360*math.pi/180)*[[x_disp[i]],[y_disp[i]],[z_disp[i]]])[2].tolist(),[])[0]

            rot_mat = tr.Rz(-180*math.pi/180)*tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_back.data[idx_list[i]] = t

    def setup_edge_taxels_transforms(self):
        self.link_name_edge = '/wrist_LEFT'
        n_taxels = 32

        self.tar_edge.data = [Transform()] * n_taxels # [Transform() for i in range(n_taxels)] #[None for i in range(n_taxels)]
        idx_list = range(0, 32)


        x_disp = [0.0]*32
        y_disp = [0.0]*32
        z_disp = [0.0]*32

        x_ang = [0]*32
        y_ang = [0]*32
        z_ang = [0]*32

        xOffset = 0.0
        yOffset = 0.0
        zOffset = 226.0*.001


        armor_offset = 0.0015
        arc_lengths = []
        arc_lengths = np.array([-59.97*.001,-27*.001,0*.001,27*.001,59.94*.001])
        flat_z = []

        flat_z = np.array([-9.5*.001,-(9.+22*1)*.001,-(9.+22*2)*.001,-(9.+22*3)*.001,-(9.+22*4)*.001,-(9.+22*5)*.001,-(9.+22*5+21.5)*.001,-(9+22*5+21.5+21.5)*.001])
        radius = []  #most of these are 48.3 mm, edges are 50.0 mm
        radius = np.array([48.3*.001, 50*.001])
        flag_side =[]

        x_dist_local = []
        y_dist_local = []
        z_dist_local = []
        flag_side = 'edge'
        x_dist_local = np.zeros([8,4])
        y_dist_local = np.zeros([8,4])
        z_dist_local = np.zeros([8,4])

        for i in [0]:
            x_dist_local[:,0] = radius[1]*math.cos(arc_lengths[i]/radius[1]) + armor_offset
            y_dist_local[:,0] = radius[1]*math.sin(arc_lengths[i]/radius[1])
            z_dist_local[:,0] = flat_z

        for i in [4]:
            x_dist_local[:,1] = radius[1]*math.cos(arc_lengths[i]/radius[1]) + armor_offset
            y_dist_local[:,1] = radius[1]*math.sin(arc_lengths[i]/radius[1])
            z_dist_local[:,1] = flat_z

        for i in [0]:
            x_dist_local[:,2] = radius[1]*math.cos(arc_lengths[i]/radius[1]+math.pi) - armor_offset
            y_dist_local[:,2] = radius[1]*math.sin(arc_lengths[i]/radius[1]+math.pi)
            z_dist_local[:,2] = flat_z

        for i in [4]:
            x_dist_local[:,3] = radius[1]*math.cos(arc_lengths[i]/radius[1]+math.pi) - armor_offset
            y_dist_local[:,3] = radius[1]*math.sin(arc_lengths[i]/radius[1]+math.pi)
            z_dist_local[:,3] = flat_z



#                x_dist_local = -radius[i]*math.cos(arc_lengths[i]/radius[i]) - armor_offset

        x_disp_temp = []
        y_disp_temp = []
        z_disp_temp = []

        for i in xrange(4):
            for j in xrange(8):
                x_disp_temp.append(x_dist_local[j,i]+xOffset)
                y_disp_temp.append(y_dist_local[j,i]+yOffset)
                z_disp_temp.append(z_dist_local[j,i]+zOffset)
#        x_disp_temp = [(i + xOffset) for i in x_dist_local]
 #       y_disp_temp = [(i + yOffset) for i in y_dist_local]
  #      z_disp_temp = [(i + zOffset) for i in z_dist_local]
#        y_disp_temp.append(np.array(y_dist_local)[i] + yOffset).tolist()) for i in y_dist_local
 #       z_disp_temp.append(np.array(z_dist_local)[i] + zOffset).tolist()) for i in z_dist_local

        x_disp = x_disp_temp
        y_disp = y_disp_temp
        z_disp = z_disp_temp
#        print len(x_disp)
#        print x_disp

        z_ang_temp = np.zeros([8,4])
        z_ang_temp[:,0] = 66*math.pi/180
        z_ang_temp[:,1] = -66*math.pi/180
        z_ang_temp[:,2] = (180+66)*math.pi/180
        z_ang_temp[:,3] = (180-66)*math.pi/180

# THE ANGLE FOR THE EDGE TAXELS IS ~66 DEGREES.



        x_ang = [0]*32
        y_ang = [math.pi/2]*32
#        y_ang[7] = 3*math.pi/4
#        y_ang[15] = 3*math.pi/4
#        y_ang[23] = 3*math.pi/4
        z_ang = []
        for i in xrange(4):
            for j in xrange(8):
                z_ang.append(z_ang_temp[j,i])



        # x_disp = [0.0214, 0.0155, 0.0096, 0.0214, 0.0155, 0.0, 0.0, 0.0, -0.0214, -0.0155, -0.0096, -0.0214, -0.0155, 0.0, 0.0, 0.0, -0.03724, -0.03217, -0.03724, -0.03217, -0.01867, -0.01615, 0.01867, 0.01615, 0.0, -0.0120, 0.0120, -0.0120, 0.0120]
        # y_disp = [-0.0098, -0.0076, 0.0, 0.0098, 0.0076, 0.0282, 0.0246, 0.0211, 0.0098, 0.0076, 0.0, -0.0098, -0.0076, -0.0282, -0.0246, -0.0211, -0.01793, -0.01413, 0.01793, 0.01413, -0.0360, -0.02846, -0.02846, -0.036, 0.0, -0.0195, -0.0195, 0.0195, 0.0195]
        # z_disp = [-0.2208, -0.2385, -0.2562, -0.2208, -0.2385, -0.2174, -0.2315, -0.2457, -0.2208, -0.2385, -0.2562, -0.2208, -0.2385, -0.2174, -0.2315, -0.2457, -0.13103, -0.1513, -0.13103, -0.1513, -0.13067, -0.15077, -0.13067, -0.15077, -0.27, -0.2384, -0.2384, -0.2384, -0.2384]


        # x_ang = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0]
        # y_ang = [-1.8900, -1.8900, -1.8900, -1.8900, -1.8900, -1.8100, -1.8100, -1.8100, 1.8900, 1.8900, 1.8900, 1.8900, 1.8900, 1.8100, 1.8100, 1.8100, math.pi/2, math.pi/2, math.pi/2, math.pi/2, 0.0, 0.0, 0.0, 0.0, math.pi, -1.8900, -1.8900, -1.8900, -1.890]
        # z_ang = [0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, -math.pi/2, -math.pi/2, -math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5*math.pi/4, math.pi/4, 5*math.pi/4, -math.pi/4]


        for i in range(n_taxels):
            t = Transform()
            t.translation.x = sum((tr.Rz(360*math.pi/180)*[[x_disp[i]],[y_disp[i]],[z_disp[i]]])[0].tolist(),[])[0]
            t.translation.y = sum((tr.Rz(360*math.pi/180)*[[x_disp[i]],[y_disp[i]],[z_disp[i]]])[1].tolist(),[])[0]
            t.translation.z = sum((tr.Rz(360*math.pi/180)*[[x_disp[i]],[y_disp[i]],[z_disp[i]]])[2].tolist(),[])[0]

            rot_mat = tr.Rz(-180*math.pi/180)*tr.Rz(z_ang[i])*tr.Ry(y_ang[i])*tr.Rx(x_ang[i])
            quat = tr.matrix_to_quaternion(rot_mat)

            t.rotation.x = quat[0]
            t.rotation.y = quat[1]
            t.rotation.z = quat[2]
            t.rotation.w = quat[3]
            self.tar_edge.data[idx_list[i]] = t


    def local_coord_frames_front_cb(self, req):
        return self.tar_front

    def link_name_front_cb(self, req):
        return self.link_name_front

    def local_coord_frames_back_cb(self, req):
        return self.tar_back

    def link_name_back_cb(self, req):
        return self.link_name_back

    def local_coord_frames_edge_cb(self, req):
        return self.tar_edge

    def link_name_edge_cb(self, req):
        return self.link_name_edge


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--front', action='store_true',
                 dest='front',
                 help='node for the front taxels of the forearmor')
    p.add_option('--back', action='store_true',
                 dest='back',
                 help='node for the back taxels of the forearmor and the end of the edges')
    p.add_option('--edge', action='store_true',
                 dest='edge',
                 help='node for the edge taxels of the forearmor')
    p.add_option('--serial_dev', action='store',
                 dest='serial_dev_name', type='string',
                 help='path to the arduino serial device')

    opt, args = p.parse_args()

    raw_data_front_pub = rospy.Publisher('taxels/raw_data',
                                           RawTaxelArray)

    raw_data_back_pub = rospy.Publisher('taxels/raw_data',
                                           RawTaxelArray)

    raw_data_edge_pub = rospy.Publisher('taxels/raw_data',
                                           RawTaxelArray)

    ts = Tactile_Sleeve()

    if opt.front:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_front_cb)
        rospy.Service('taxels/srv/link_name', None_String,
                      ts.link_name_front_cb)
        n_taxels = 24
    elif opt.back:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_back_cb)
        rospy.Service('taxels/srv/link_name', None_String,
                      ts.link_name_back_cb)
        n_taxels = 28
    elif opt.edge:
        rospy.Service('taxels/srv/local_coord_frames',
                      None_TransformArray, ts.local_coord_frames_edge_cb)
        rospy.Service('taxels/srv/link_name', None_String,
                      ts.link_name_edge_cb)
        n_taxels = 32
    else:
        rospy.logerr('Specify --front or --back or --edge')
        sys.exit()

    rospy.init_node('fabric_tactile_sleeve_driver_node')

    baudrate = 115200
    dev = apn.setup_serial(opt.serial_dev_name, baudrate)

    for i in range(10):
        dev.readline()

    rospy.loginfo('Started publishing data')

    front_rta = RawTaxelArray()
    back_rta = RawTaxelArray()
    edge_rta = RawTaxelArray()
    while not rospy.is_shutdown():
        adc_data = apn.get_adc_data(dev, 32)
        if opt.front:
            if adc_data == []:
                rospy.logwarn('Data from front taxels was emtpy ...')
            else:
                front_rta.val_z = front_vals = adc_data[0:24]# + adc_data[24:32]
                #mid_rta.val_z = mid_vals = adc_data[0:20]
                raw_data_front_pub.publish(front_rta)
        elif opt.back:
            if adc_data == []:
                rospy.logwarn('Data from back taxels was emtpy ...')
            else:
                back_rta.val_z = back_vals = adc_data[0:25]+adc_data[26:27]+adc_data[28:29]+adc_data[30:31]
                raw_data_back_pub.publish(back_rta)
        elif opt.edge:
            if adc_data == []:
                rospy.logwarn('Data from edge taxels was emtpy ...')
            else:
                edge_rta.val_z = edge_vals = adc_data[0:32] #+ adc_data[8:16]
                raw_data_edge_pub.publish(edge_rta)

    dev.close()


