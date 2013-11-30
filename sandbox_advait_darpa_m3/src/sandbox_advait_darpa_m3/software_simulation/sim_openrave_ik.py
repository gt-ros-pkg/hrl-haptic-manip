
import numpy as np, math

import openravepy as orpy

import roslib; roslib.load_manifest('darpa_m3')
import geometric_search.write_xml_openrave as wxo
import hrl_lib.transforms as tr

import sandbox_advait.openrave_util as ou
# generate openrave xml from some python config file.


def create_openrave_env():
    import geometric_search.three_link_planar as tlp

    robot_xml = 'sim_robot.xml'
    wxo.write_linkage_xml_file(tlp.b_jts, tlp.bodies, robot_xml)
    xml_filename = 'sim_ik.xml'
    file_obj = open(xml_filename, 'w')
    wxo.environment_start(file_obj)
    file_obj.write('\t <Robot name="3DOFRobot" file="'+robot_xml+'"> \n'+
                   '\t </Robot> \n\n')

    extent = [0.3, 0.001, 1.5]
    wxo.write_cuboid_obstacles(file_obj, 10, -0.7, extent)
    wxo.environment_end(file_obj)
    file_obj.close()

    env = orpy.Environment()
    env.Load('sim_ik.xml')
    #env.SetViewer('qtcoin') # start the viewer
    return env

def create_openrave_ik_model(env):
    robot = env.GetRobots()[0] # get the first robot
    ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot,
                        iktype = orpy.IkParameterization.Type.TranslationXY2D)
    if not ikmodel.load():
        print 'auto generating the IK solver'
        ikmodel.autogenerate()
    return ikmodel

def create_openrave_dict():
    d = {}
    d['env'] = create_openrave_env()
    env = d['env']
    d['robot'] = env.GetRobots()[0]
    d['bodies'] = env.GetBodies()[1:]
    d['ik_model'] = create_openrave_ik_model(env)
    d['ik_type'] = orpy.IkParameterization.Type.TranslationXY2D
    d['ik_param_func'] = orpy.IkParameterization
    d['filter_options'] = orpy.IkFilterOptions.CheckEnvCollisions | orpy.IkFilterOptions.IgnoreSelfCollisions
    d['dof_idxs'] = [0, 1, 2]
    return d

if __name__ == '__main__':
    d = create_openrave_dict()
    env = d['env']
    env.SetViewer('qtcoin')
    ik_model = d['ik_model']
    obstacles = d['bodies']
    robot = d['robot']

    q_curr = [-60/180.0*3.14, -57.0/180.0*3.14, 157.0/180.0*3.14]
    robot.SetDOFValues(q_curr, [0, 1, 2])
    
    
    filter_options = d['filter_options']
    ik_param_func = d['ik_param_func']
    ik_type = d['ik_type']

    # test IK
    if False:
        #p = np.matrix([0.0, -0.3]).T
        p = np.matrix([0.3, -0.5]).T
        ang = math.radians(90.)
        ou.move_obstacle(obstacles[0], p, ang)

        goal = [0.4, -0.1]
        ik_param = ik_param_func(goal, ik_type)
        q = ik_model.manip.FindIKSolution(ik_param, filter_options)

        if q == None:
            print 'No IK solution found.'
        else:
            robot.SetDOFValues(q, [0, 1, 2])

        raw_input('Hit a key to end.')
        env.Destroy()

    # in case goal_within_workspace returns something non-intuitive,
    # uncomment some lines to print out the info from the task monitor
    # and use the code below to visually look at the solution the it
    # found.
    if True:
        q_curr = (0.24243008444347147, 1.1281183325160136, 0.0018579105954179553)
        robot.SetDOFValues(q_curr, [0, 1, 2])

        p = np.matrix([ 0.48190133, -0.6047239]).T
        ang = -1.76918117638
        ou.move_obstacle(obstacles[0], p, ang)

        raw_input('Hit a key to see the computed solution.')

        q_goal = [ 1.82196096, -1.00487565,  0.00185791]
        robot.SetDOFValues(q_goal, [0, 1, 2])

        raw_input('Hit a key to end.')
        env.Destroy()



# current q: (0.24243008444347147, 1.1281183325160136, 0.0018579105954179553)
# p: [ 0.48190133 -0.6047239 ]
# ang: -1.76918117638
# q_goal: [ 1.82196096 -1.00487565  0.00185791]




