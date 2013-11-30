
import numpy as np, math

import roslib; roslib.load_manifest('epc_core')
import cody_arms.arm_client as cac
import cody_arms.arms as ar
import rospy

import openravepy as orpy

env = orpy.Environment()
env.SetViewer('qtcoin')
env.Load('/home/advait/svn/robot1/src/projects/darpa_m3/viz/cody_skin.env.xml')
cody_openrave = env.GetRobots()[0]
cody_openrave.SetTransform(np.diag([0.,0.,0.,1.]))


ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(cody_openrave,
                            iktype=orpy.IkParameterization.Type.Translation3D)
if not ikmodel.load():
    print 'auto generating IK model'
    ikmodel.autogenerate()

#goal = [0.4, -0.25, 0.]
goal = [0.34101324037196029, -0.45722757606194592, 0.096394611852133424]

ik_param = orpy.IkParameterization(goal, orpy.IkParameterization.Type.Translation3D)
filter_options = orpy.IkFilterOptions.CheckEnvCollisions | orpy.IkFilterOptions.IgnoreSelfCollisions
#solutions = ikmodel.manip.FindIKSolutions(ik_param, filter_options)
#print 'len(solutions):', len(solutions)

idx_list = range(7)
a = np.radians([90, 120, -90, 120, 90, 0, 0])
a = [ 1.53349316,  2.02274781, -1.39585567,  1.91450589,  1.60124238, -0.0739788, -0.19345394]


cody_openrave.SetDOFValues(a, idx_list)

solution = ikmodel.manip.FindIKSolution(ik_param, filter_options)
cody_openrave.SetDOFValues(solution, idx_list)

env.plot3(np.array(goal), pointsize=15.0)

#if __name__ == '__main__':
raw_input('Hit ENTER to end.')
orpy.RaveDestroy()


# update joints from Cody
if False:
    idx_list = range(7)
    arms = ar.M3HrlRobot()
    ac = cac.MekaArmClient(arms)
    r_arm = 'right_arm'
    while not rospy.is_shutdown():
        rospy.sleep(0.02)
        q = ac.get_joint_angles(r_arm)
        cody_openrave.SetDOFValues(q, idx_list)




