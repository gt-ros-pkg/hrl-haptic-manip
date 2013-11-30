
import numpy as np, math

import roslib;
roslib.load_manifest('hrl_hardware_in_loop_simulation_darpa_m3')
import rospy

import openravepy as orpy

env = orpy.Environment()
env.SetViewer('qtcoin')
env.Load('../cody_skin_env.xml')
cody_openrave = env.GetRobots()[0]
cody_openrave.SetTransform(np.diag([0.,0.,0.,1.]))

raw_input('Hit ENTER to end.')
orpy.RaveDestroy()



