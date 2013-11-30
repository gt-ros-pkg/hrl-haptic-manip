

#
#
# This doesn't really run. Here for historical purposes.
#
#
#
#
#


import numpy as np, math

import roslib;
roslib.load_manifest('hrl_hardware_in_loop_simulation_darpa_m3')
import rospy

import hrl_cody_arms.cody_arm_client as cac

import openravepy as orpy


env = orpy.Environment()
env.SetViewer('qtcoin')
#env.Load('./cody_skin_env.xml')
env.Load('./openrave_ik.xml')
cody_openrave = env.GetRobots()[0]
cody_openrave.SetTransform(np.diag([0.,0.,0.,1.]))

link_list = cody_openrave.GetLinks()

for lnk in link_list:
    print lnk.GetName()

if False:
    #    env.GetCollisionChecker().SetCollisionOptions(orpy.CollisionOptions.Contacts)
    goal = [0.4, 0.3, 0.]

    ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(cody_openrave,
                                iktype=orpy.IkParameterization.Type.Translation3D)
    ikmodel.load()
    ik_param = orpy.IkParameterization(goal, orpy.IkParameterization.Type.Translation3D)
    filter_options = orpy.IkFilterOptions.CheckEnvCollisions | orpy.IkFilterOptions.IgnoreSelfCollisions
    idx_list = range(7)
    a = np.radians([90, 120, -90, 120, 90, 0, 0])
    cody_openrave.SetDOFValues(a, idx_list)
    solution = ikmodel.manip.FindIKSolution(ik_param, filter_options)
    cody_openrave.SetDOFValues(solution, idx_list)

    bodies = env.GetBodies()[1:]
    b = bodies[0]
    t = b.GetTransform()
    t[0,3] = 0.3
    t[1,3] = 0.22 + 0.2
    b.SetTransform(t)

    raw_input('Hit ENTER to perform IK again.')
#    cody_openrave.SetDOFValues(a, idx_list)
    solution = ikmodel.manip.FindIKSolution(ik_param, filter_options)
    cody_openrave.SetDOFValues(solution, idx_list)



# update joints from Cody
if False:
    idx_list = range(7)
    arm = 'r'
    ac = cac.CodyArmClient(arm)
    while not rospy.is_shutdown():
        rospy.sleep(0.02)
        q = ac.get_joint_angles()
        cody_openrave.SetDOFValues(q, idx_list)


raw_input('Hit ENTER to end.')
orpy.RaveDestroy()



