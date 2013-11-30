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


import roslib
roslib.load_manifest('hrl_dynamic_mpc')
from hrl_dynamic_mpc.srv import LogData, LogDataResponse
import rospy
from geometry_msgs.msg import WrenchStamped
from m3skin_ros.msg import RawTaxelArray
#from m3ctrl_msgs.msg import M3JointCmd
from sensor_msgs.msg import JointState
import time
import threading
from collections import deque
import datetime

# don't think we need this anymore, can delete if it runs properly
#roslib.load_manifest('sandbox_marc_darpa_m3')


class Recorder():
    def __init__(self,topic_list, write_function_list, title_list, msg_type_list, prefix = ''):
        self.topic_list = topic_list
        self.data = {}
        self.files = {}
        self.lock = threading.RLock()
        self.write_function_list = write_function_list
        self.running = False
        self.title_list = title_list

        rospy.init_node('data_recorder')

        for i in xrange(len(topic_list)):
            self.data[topic_list[i]] = deque()
            rospy.Subscriber('/'+topic_list[i], msg_type_list[i], self.make_callback_func(topic_list[i]))

        self.file_name = None

        self.s = rospy.Service('log_data', LogData, self.start_writing)

        self.write_thread = threading.Thread(target=self.write_data)
        self.write_thread.start()
        rospy.spin()

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

    def make_callback_func(self, topic):
        def callback(msg):
            with self.lock:
                self.data[topic].append(msg)
        return callback

    def write_data(self):
        print "GOT IN HERE"
        while not rospy.is_shutdown():
            if self.file_name != None:
                for i in xrange(len(topic_list)):
                    topic_name_ls = self.topic_list[i].split('/')
                    self.files[self.topic_list[i]] = open('/media/HRL_fast_write_d/'+self.file_name+topic_name_ls[0]+'.txt', 'a')
                    self.files[self.topic_list[i]].write(self.title_list[i])

                while self.file_name != None:
                    for i in xrange(len(self.topic_list)):
                        #this writes a message from each list until they are all empty, might be better to empty them one at a time instead, not sure -marc
                        if self.data[self.topic_list[i]] != deque():
                            self.lock.acquire()
                            buf_msg = self.data[self.topic_list[i]].popleft()
                            self.lock.release()
                            self.files[self.topic_list[i]].write(self.write_function_list[i](buf_msg))
                    time.sleep(0.001)
                for name in self.topic_list:
                    self.files[name].close()
                time.sleep(0.001)
            else:
                for i in xrange(len(topic_list)):
                    self.data[topic_list[i]].clear()

                time.sleep(0.001)

if __name__ == '__main__':

    def ft_write_function(msg):
        return str(msg.header.seq)+',\t'+str(msg.header.stamp.secs+msg.header.stamp.nsecs*1e-9)+\
                   ',\t'+str(msg.wrench.force.x)+',\t'+str(msg.wrench.force.y)+',\t'+str(msg.wrench.force.z)+\
                   ',\t'+str(msg.wrench.torque.x)+',\t'+str(msg.wrench.torque.y)+',\t'+str(msg.wrench.torque.z)+\
                   ' \n'

    def skin_calibration_write_function(msg):
        time_stamp = rospy.get_time()
        time_output = [repr(time_stamp)]
        output = time_output + [',\t'+str(msg.val_z[i]) for i in xrange(len(msg.val_z))]
        return ''.join(output)+'\n'

    def q_cmd_write_function(msg):
        output = str(msg.header.seq)+', \t'+str(msg.header.stamp.secs+msg.header.stamp.nsecs*1e-9)+', \t'
        for i in xrange(6):
            output = output+str(msg.position[i])+', \t'
        output = output+str(msg.position[6])+'\n'

        return output
            
    def robot_state_write_function(msg):
        output = str(msg.header.seq)+', \t'+str(msg.header.stamp.secs+msg.header.stamp.nsecs*1e-9)+', \t'
        for i in xrange(7,14):
            output = output+str(msg.position[i])+', \t'
        for i in xrange(7,14):
            output = output+str(msg.velocity[i])+', \t'
        for i in xrange(7,13):
            output = output+str(msg.effort[i])+', \t'
        output = output+str(msg.effort[6])+' \n'

        return output

    title_list = ['sequence,\t'+'time,\t'+'f_x,\t'+'f_y,\t'+'f_z,\t'+'tau_x,\t'+'tau_y,\t'+'tau_z\n',
                  #'sequence,\t'+'time,\t'+'f_x,\t'+'f_y,\t'+'f_z,\t'+'tau_x,\t'+'tau_y,\t'+'tau_z\n',
                  #'sequence,\t'+'time,\t'+'f_x,\t'+'f_y,\t'+'f_z,\t'+'tau_x,\t'+'tau_y,\t'+'tau_z\n',
                  ''.join(['sequence']+[',\t taxel_'+str(i).zfill(2) for i in xrange(24)])
                  # 'sequence,\t'+'time,\t'+'q_0,\t'+'q_1,\t'+'q_2,\t'+'q_3,\t'+'q_4,\t'+'q_5,\t'+'q_6,\t'+'q_7,\t'+\
                  #     'qd_0,\t'+'qd_1,\t'+'qd_2,\t'+'qd_3,\t'+'qd_4,\t'+'qd_5,\t'+'qd_6,\t'+'qd_7\n',
                  # 'sequence,\t'+'time,\t'+'q_des_0,\t'+'q_des_1,\t'+'q_des_2,\t'+'q_des_3,\t'+'q_des_4,\t'+'q_des_5,\t'+'q_des_6\n']
                 ]
                  

    #msg_type_list = [WrenchStamped, WrenchStamped, WrenchStamped, JointState, M3JointCmd]
    #msg_type_list = [WrenchStamped, JointState, M3JointCmd]
    #msg_type_list = [JointState, M3JointCmd]
    msg_type_list = [WrenchStamped, RawTaxelArray]


    topic_list = [#'force_torque_ft8', 
                  'force_torque_ft11', 
                  #'force_torque_ft9', 
                  'fabric_forearmor_front_sensor/taxels/raw_data'
                  #'humanoid_state',
                  #'humanoid_command'
                  ]

    function_list = [#ft_write_function,
                     ft_write_function,
                     skin_calibration_write_function,
                     #ft_write_function,
                     #robot_state_write_function,
                     #q_cmd_write_function]
                     ]

    recorder = Recorder(topic_list, function_list, title_list, msg_type_list, 'test_')

