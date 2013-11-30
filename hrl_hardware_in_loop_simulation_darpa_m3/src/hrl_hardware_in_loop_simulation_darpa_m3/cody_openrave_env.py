
import roslib;
roslib.load_manifest('hrl_hardware_in_loop_simulation_darpa_m3')
import hrl_motion_planners_darpa_m3.write_xml_openrave as wxo
import functools as ft


#----------- configuration --------


def skin_simulation_config_dict(arm):
    xml_filename = 'cody_skin_env.xml'

#    obstacle_nm_l = ['force_torque_ft'+str(i) for i in range(1,n_ft_sensors+1)]
    obstacle_nm_l = []
    obstacle_nm_l.append('force_torque_ft1')
    obstacle_nm_l.append('force_torque_ft2')
    #obstacle_nm_l.append('force_torque_ft4')
    obstacle_nm_l.append('force_torque_ft5')
    #obstacle_nm_l.append('force_torque_ft6')
    obstacle_nm_l.append('force_torque_ft7')
    obstacle_nm_l.append('force_torque_ft8')
    obstacle_nm_l.append('force_torque_ft9')
    obstacle_nm_l.append('force_torque_ft10')
    obstacle_nm_l.append('force_torque_ft11')

    #netft_flag_list = [False] * 2 + [True] * 8
    netft_flag_list = [False] * 2 + [True] * 6

    # list of radii
    #radius_list = [0.039, 0.045, 0.042, 0.043, 0.038, 0.042, 0.043,
    #               0.041, 0.041, 0.047]
    radius_list = [0.039, 0.045, 0.043, 0.042, 0.043,
                   0.041, 0.041, 0.047]
    radius_list = [r+0.01 for r in radius_list]

    # single obstacle (pillow)
    #radius_list = [0.11]

    # single obstacle (bubble wrap only)
    #radius_list = [0.0475]
    
    assert(len(obstacle_nm_l) == len(netft_flag_list) == len(radius_list))
    n_ft_sensors = len(obstacle_nm_l)

    # initial position of obstacles
    pos_list = [[(i+1) * -0.4, 0. , 0.] for i in range(n_ft_sensors)]

    # create a new obstacle whose radius is radius_step less than the previous
    radius_step = 0.002
    n_radius_steps = 20

    # single obstacle (pillow)
    #radius_step = 0.0005
    #n_radius_steps = 100

    # single obstacle (bubble wrap only)
    #radius_step = 0.00025
    #n_radius_steps = 100
    
    d = {}
    d['xml_filename'] = xml_filename
    d['obstacle_nm_l'] = obstacle_nm_l
    d['netft_flag_list'] = netft_flag_list
    d['radius_list'] = radius_list
    d['pos_list'] = pos_list
    d['radius_step'] = radius_step
    d['n_radius_steps'] = n_radius_steps
    d['arm'] = arm
    return d


#------- now for the XML writing ----------
def write_xml(xml_filename, n_radius_steps, radius_step, pos_list,
              radius_list, obstacle_nm_l, netft_flag_list, arm):
    file_obj = open(xml_filename, 'w')

    if arm == 'r':
        ee = 'end_effector_RIGHT'
    if arm == 'l':
        ee = 'end_effector_LEFT'

    wxo.environment_start(file_obj)
    wxo.write_robot(file_obj, 'cody.dae', ee, 'torso_lift_link',
                    [0, 0.075, 0], [1, 0, 0, 0])

    for i in range(n_radius_steps):
        nm_l = [nm+'_%d'%i for nm in obstacle_nm_l]
        write_kin_body = ft.partial(wxo.write_openrave_cylinder_KinBody,
                                    file_obj, rotationaxis=[1, 0, 0, -90],
                                    height = 1.5, diffuse_color=[1, 0.2, 0.2])

        for nm, trans, rad in zip(nm_l, pos_list, radius_list):
            trans[1] = i*0.1
            write_kin_body(nm, trans, radius=rad-i*radius_step)

    wxo.environment_end(file_obj)

def write_openrave_IK_controller_xml():
    xml_filename = 'openrave_ik.xml'
    n_thin_obstacles = 5

    file_obj = open(xml_filename, 'w')

    wxo.environment_start(file_obj)
    wxo.write_robot(file_obj, 'cody.dae', 'end_effector',
                    'torso_lift_link', [0, 0.075, 0], [1, 0, 0, 0])
    extent = [0.001, 0.2, 1.5]
    wxo.write_cuboid_obstacles(file_obj, n_thin_obstacles, -0.4, extent)

    wxo.environment_end(file_obj)


if __name__ == '__main__':
    #cd = skin_simulation_config_dict()
    #write_xml(**cd)

    # separate write function because I want box obstacles instead of
    # cylinders which is what the write_xml function assumes.
    write_openrave_IK_controller_xml()

