#!/usr/bin/python

import numpy as np
import roslib
roslib.load_manifest('hrl_software_simulation_darpa_m3')
import rospy

from hrl_msgs.msg import FloatArrayBare
from hrl_srvs.srv import None_FloatArray, None_FloatArrayResponse, None_FloatArrayRequest
from hrl_lib.transforms import *
import hrl_lib.viz as hv
import threading
import time
import copy

class SynchControl:
    def __init__(self):
        self.lock = threading.RLock()
        rospy.init_node('synch_control_with_simulation')
        rospy.Subscriber("/sim_arm/command/jep", FloatArrayBare, self.control_callback)
        serv = rospy.Service('synch_controller', None_FloatArray, self.synch_service)
        #start a service call here, this is the server
        self.jep_cmd = []
        self.updated_cmd = False
        self.time_last = time.time()

    def control_callback(self, msg):
        with self.lock:
            self.updated_cmd = True
            self.jep_cmd = msg.data
            print "time diff is :", time.time()-self.time_last
            self.time_last = time.time()

    def synch_service(self, req):
        res = None_FloatArrayResponse()
        while self.updated_cmd == False:
            time.sleep(0.1)
            #pass
            #would waiting for a message work instead????
            #
        with self.lock:
            print "got jep"
            res.value = copy.copy(self.jep_cmd)
        self.updated_cmd = False
        return res

if __name__ == '__main__':

    synch = SynchControl()
    rospy.loginfo('Started node to synchronize control and simulation at 100 Hz')
    rospy.spin()

                        
