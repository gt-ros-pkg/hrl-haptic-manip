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
roslib.load_manifest('sandbox_marc_darpa_m3')
roslib.load_manifest('hrl_dynamic_mpc')
import subprocess
import sys
import time
import hrl_lib.util as ut
import numpy as np
import os
from hrl_dynamic_mpc.srv import LogData, LogDataResponse
import threading
import rospy
import signal

class RosbagRecorder():
    def __init__(self,):
        self.lock = threading.RLock()
        self.running = False
        rospy.init_node('rosbag_recorder')

        self.file_name = None
        self.s = rospy.Service('rosbag_data', LogData, self.start_writing)

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

    def write_data(self):
        print "GOT IN HERE"
        while not rospy.is_shutdown():
            if self.file_name!=None:
                rosbag = subprocess.Popen(['rosbag',
                                           'record',
                                           '-a',
                                           '-O',
                                           '/media/HRL_fast_write_d/'+self.file_name])
                while self.file_name != None:
                    time.sleep(0.001)
                rosbag.send_signal(signal.SIGINT)
            else:
                time.sleep(0.001)

#  Popen.send_signal(signal)
# import os
# 
# import subprocess

# popen = subprocess(...)
# os.kill(popen.pid, signal.SIGINT)
if __name__ == '__main__':

    recorder = RosbagRecorder()
