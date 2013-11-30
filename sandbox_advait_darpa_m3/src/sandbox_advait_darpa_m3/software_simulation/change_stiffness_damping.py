
import numpy as np, math

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
import rospy

import hrl_software_simulation_darpa_m3.ode_sim_arms as osa
import hrl_software_simulation_darpa_m3.ode_sim_guarded_move as sgm

rospy.init_node('joint_impedance_setter')

ode_arm = osa.ODESimArm()
rospy.sleep(0.1)

kp = np.array([30., 20., 15.])
kd = np.array([15., 10., 8.])

#kp = np.array([3., 2., 1.5])
#kd = np.array([4.5, 3., 0.6])

kp_scale = 0.25
kd_scale = 1.

ode_arm.set_joint_impedance(kp*kp_scale, kd*kd_scale)
rospy.sleep(0.1)

kp, kd = ode_arm.get_joint_impedance()
print 'New Stiffness:', kp
print 'New Damping:', kd

jep1 = np.radians([90.0, 90, 0])
ode_arm.set_ep(jep1)

raw_input('Hit Enter for next configuration')

jep1 = np.radians([-90.0, -90, 90])
ode_arm.set_ep(jep1)




