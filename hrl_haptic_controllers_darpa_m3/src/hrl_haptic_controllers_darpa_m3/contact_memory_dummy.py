# This file is required so that the qp controller doesn't crash. The
# QP controller is expecting a service call, this node provides a
# dummy service call.  If you want to estimate the contact
# characteristics you can use the original contact_memory from Charlie
# it is in the old Darpa M3 folder or the one from Advait in his
# sandbox.

import numpy as np, math
import copy
import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('hrl_haptic_controllers_darpa_m3')
import rospy
from hrl_srvs.srv import FloatArray_Float, FloatArray_FloatResponse

class DummyContactEstimation():
    def __init__(self):
        pass
    #--------------- ROS stuff -------------------
    def dummy_cb(self, req):
        print "THIS IS A DUMMY CALLBACK TO KEEP THE QP CONTROLLER RUNNING"
        return FloatArray_FloatResponse(-1)


if __name__ == '__main__':
    dummy = DummyContactEstimation()
    rospy.Service('/contact_memory/estimate_contact_stiffness', 
                  FloatArray_Float,
                  dummy.dummy_cb)
    rospy.init_node('dummy_contact_memory_node')
    rospy.loginfo('Started Dummy Contact Memory.')

    rt = rospy.Rate(10)
    while not rospy.is_shutdown():
        rt.sleep()



