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

if __name__ == '__main__':

    rospy.init_node('testing_synch_control_with_simulation')
    test_synch = rospy.ServiceProxy('synch_controller', None_FloatArray)
    start = time.time()
    while not rospy.is_shutdown():
        resp1 = test_synch()
        print "resp1.value :", resp1.value
        print "time diff is :", time.time() - start
        start = time.time()
        rospy.sleep(0.01)


                        
