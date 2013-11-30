#!/usr/bin/python
#
#
# Copyright (c) 2013, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \authors: Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
# \adviser: Charles Kemp (Healthcare Robotics Lab, Georgia Tech.)

import roslib; roslib.load_manifest('hrl_dynamic_mpc') 
import rospy
import sys, time, os
import math, numpy as np
import time
from hrl_haptic_manipulation_in_clutter_msgs.msg import TaxelArray
from threading import RLock, Timer
import copy
from hrl_lib import transforms as tr
from collections import *

##
#Class SkinClient()
#gives access to skin data that is published across ros given a list of topics to subscribe to
#

class SkinClient():
    def __init__(self, topic_list, noise_thresh = 0.1):
        try:
            rospy.init_node('skin_client')
        except:
            pass
        self.values = {}
        self.nrmls = {}
        self.locs = {}
        self.values_out = {}
        self.nrmls_out = {}
        self.locs_out = {}
        self.frames = {}
        self.lock = RLock()
        self.topic_list = topic_list
        self.noise_thresh = noise_thresh

        for topic in self.topic_list:
            self.values_out[topic] =[]
            self.locs_out[topic] = []
            self.nrmls_out[topic] = []


        for topic in topic_list:
            rospy.Subscriber('/'+topic+'/taxels/forces', TaxelArray, self.make_callback_func(topic))

        ## Other capabilities that we could add to this driver
        # /fabric_wrist_sensor/zero_sensor
        # /fabric_wrist_sensor/disable_sensor
        # /fabric_wrist_sensor/enable_sensor
        # /fabric_wrist_sensor/taxels/forces


    def make_callback_func(self, topic):
        def callback(msg):
            with self.lock:
                self.locs[topic] = [[msg.centers_x[i], msg.centers_y[i], msg.centers_z[i]] for i in xrange(len(msg.centers_x))]
                self.nrmls[topic] = [[msg.normals_x[i], msg.normals_y[i], msg.normals_z[i]] for i in xrange(len(msg.normals_x))]
                self.values[topic] = [[msg.values_x[i], msg.values_y[i], msg.values_z[i]] for i in xrange(len(msg.values_x))]
                self.frames[topic] = msg.header.frame_id
        return callback

    def updateSkinInTorsoFrame(self, positions, rotations, q):
        if self.frames == {}:
            for topic in self.topic_list:
                self.values_out[topic] =[]
                self.locs_out[topic] = []
                self.nrmls_out[topic] = []
        else:
            with self.lock:
                for topic in self.topic_list:
                    t1 = positions[topic]
                    r1 = rotations[topic]

                    #Trimming forces and other vectors according to noise threshold
                    values_n = np.array([np.linalg.norm(value) for value in self.values[topic]])
                    ind = (np.where(values_n > self.noise_thresh)[0]).tolist()

                    if len(ind) > 0:
                        if topic == 'fabric_forearm_sensor':
                            joint_inds = [0, 3, 6, 9]
                            for test in joint_inds:
                                if test in ind and (abs(q[5]) > 0.52 or abs(q[6]) > 0.52):
                                    ind.remove(test)

                        values_out = np.array(self.values[topic])[ind]
                        locs_out = np.array(self.locs[topic])[ind]
                        nrmls_out = np.array(self.nrmls[topic])[ind]

                        self.values_out[topic] = (r1*np.matrix(values_out).T).T
                        self.locs_out[topic] = (r1*np.matrix(locs_out).T + t1).T
                        self.nrmls_out[topic] = (r1*np.matrix(nrmls_out).T).T
                    else:
                        self.values_out[topic] =[]
                        self.locs_out[topic] = []
                        self.nrmls_out[topic] = []

    def getNormals(self):
        with self.lock:
            return self.nrmls_out

    def getValues(self):
        with self.lock:
            return self.values_out

    def getLocs(self):
        with self.lock:
            return self.locs_out

if __name__ == '__main__':
    topic_list = ['fabric_forearm_sensor', 'fabric_wrist_sensor']
    skin = SkinClient(topic_list)
    rate = rospy.Rate(100)

    start = time.time()
    while not rospy.is_shutdown():
        print "time is :", time.time() - start
        start = time.time()
        skin.updateSkinInTorsoFrame()
        print "time to get skin frame :", time.time() - start
        next = time.time()
        skin.getLocs()
        print "time to get locs:", time.time()- next
        next = time.time()
        skin.getNormals()
        print "time to get nrmls:", time.time()- next
        next = time.time()

        skin.getValues()
        print "time to get values:", time.time()- next
        next = time.time()

        #print "Location:"
        # print skin.getLocs(), "\n\n\n"
        # print "normals:"
        # print skin.getNormals(), "\n\n\n"
        # print "values:"
        # print skin.getValues(), "\n\n\n"
