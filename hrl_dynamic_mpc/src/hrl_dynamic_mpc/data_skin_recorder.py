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
from hrl_dynamic_mpc.srv import LogData, LogDataResponse
import copy
from hrl_lib import transforms as tr
from collections import *
import darci_client as dc
import datetime
import threading

class SkinRecorder():
    def __init__(self, topic_list, write_function_list, title_list, prefix=''):
        try:
            rospy.init_node('skin_client')
        except:
            pass

        self.files = {}
        self.write_function_list = write_function_list
        self.running = False
        self.title_list = title_list
        self.file_name = None

        self.jts_dict = {'fabric_forearm_sensor':4, 'fabric_wrist_sensor':6}
        self.values = {}
        self.nrmls = {}
        self.locs = {}
        self.times = {}
        self.values_out = {}
        self.times_out = {}
        self.nrmls_out = {}
        self.locs_out = {}
        self.frames = {}
        self.lock = RLock()
        self.topic_list = topic_list
        self.positions = {}
        self.rotations = {}
        self.robot = dc.DarciClient() #RobotHapticStateServer(opt)

        for topic in self.topic_list:
            self.values_out[topic] = deque()
            self.locs_out[topic] = deque()
            self.nrmls_out[topic] = deque()
            self.times_out[topic] = deque()

        for topic in topic_list:
            rospy.Subscriber('/'+topic+'/taxels/forces', TaxelArray, self.make_callback_func(topic))
            

        self.s = rospy.Service('log_skin_data', LogData, self.start_writing)

        self.write_thread = threading.Thread(target=self.write_data)
        self.write_thread.start()

        rospy.spin()

    def make_callback_func(self, topic):
        def callback(msg):
            with self.lock:
                self.locs[topic] = [[msg.centers_x[i], msg.centers_y[i], msg.centers_z[i]] for i in xrange(len(msg.centers_x))]
                self.nrmls[topic] = [[msg.normals_x[i], msg.normals_y[i], msg.normals_z[i]] for i in xrange(len(msg.normals_x))]
                self.values[topic] = [[msg.values_x[i], msg.values_y[i], msg.values_z[i]] for i in xrange(len(msg.values_x))]
                self.frames[topic] = msg.header.frame_id
                self.times[topic] = msg.header.stamp 
            
                q = self.robot.getJointAngles()
                pos, rot = self.robot.kinematics.FK(q, self.jts_dict[topic])
                self.positions[topic] = pos
                self.rotations[topic] = rot 

                t1 = self.positions[topic]
                r1 = self.rotations[topic]

                #Trimming forces and other vectors according to noise threshold
                values_n = np.array([np.linalg.norm(value) for value in self.values[topic]])

                values_out = np.array(self.values[topic])
                locs_out = np.array(self.locs[topic])
                nrmls_out = np.array(self.nrmls[topic])
                
                self.values_out[topic].append((r1*np.matrix(values_out).T).T)
                self.locs_out[topic].append((r1*np.matrix(locs_out).T + t1).T)
                self.nrmls_out[topic].append((r1*np.matrix(nrmls_out).T).T)
                self.times_out[topic].append(self.times[topic])
        return callback


    def start_writing(self, req):
        self.running = not(self.running)
        resp = LogDataResponse()
        self.lock.acquire()
        if self.running == True:
            resp.response = 'running'
            self.file_name = req.file_name
        else:
            resp.response = 'not running'
            self.file_name = None
        self.lock.release()

        return resp


    def write_data(self):
        # fmt='%Y-%m-%d-%H-%M-%S_{name}'
        print "GOT IN HERE"
        while not rospy.is_shutdown():
            if self.file_name != None:
                for i in xrange(len(topic_list)):
                    self.files[self.topic_list[i]] = open('/media/HRL_fast_write_d/'+self.file_name+self.topic_list[i]+'.txt', 'a')
                    self.files[topic_list[i]].write(self.title_list[i])
                while self.file_name != None:
                    for i in xrange(len(self.topic_list)):
                        if self.times_out[self.topic_list[i]] != deque():
                            try:
                                self.lock.acquire()
                                vals = self.values_out[self.topic_list[i]].popleft()
                                nrmls = self.nrmls_out[self.topic_list[i]].popleft()
                                locs = self.locs_out[self.topic_list[i]].popleft()
                                times = self.times_out[self.topic_list[i]].popleft()
                                self.lock.release()
                                self.files[self.topic_list[i]].write(self.write_function_list[i](times, vals, locs, nrmls))
                            except IndexError:
                                pass
                    time.sleep(0.001)

                for name in self.topic_list:
                    self.files[name].close()
                time.sleep(0.1)

            else:
                for topic in self.topic_list:
                    self.values_out[topic].clear()
                    self.locs_out[topic].clear()
                    self.nrmls_out[topic].clear()
                    self.times_out[topic].clear()
                time.sleep(0.001)
                      

if __name__ == '__main__':

    topic_list = ['fabric_forearm_sensor', 'fabric_wrist_sensor']
    title_list = ['time,\t'+'value,\t'+'nrml,\t'+'location,\t'+'value_nrml_loc_repeated\n',
                  'time,\t'+'value,\t'+'nrml,\t'+'location,\t'+'value_nrml_loc_repeated\n']

    def skin_write_function(time, values, locs, nrmls):
        output = [str(time)+', '] + [str(np.linalg.norm(values[i,:]))+', '+str(nrmls[i,0])+', '+str(nrmls[i,1])+', '+str(nrmls[i,2])+', '+\
                                     str(locs[i,0])+', '+str(locs[i,1])+', '+str(locs[i,2])+', ' for i in xrange(len(locs))]
        return ''.join(output)+'\n'
        
    recorder = SkinRecorder(topic_list, [skin_write_function, skin_write_function], title_list, 'test_')

    rospy.spin()

