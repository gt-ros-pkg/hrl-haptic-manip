
import os
import sys, copy
import numpy as np, math
from threading import RLock

import ft_sensors
import contacts_openrave as co

import roslib;
roslib.load_manifest('hrl_hardware_in_loop_simulation_darpa_m3')
import rospy
import tf

import hrl_common_code_darpa_m3.visualization.viz as viz

import hrl_lib.viz as hv
import hrl_lib.util as ut
import hrl_lib.transforms as tr

from hrl_haptic_manipulation_in_clutter_msgs.msg import SkinContact
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

def contact_info_list_to_dict(cont_info_list):
    ci = cont_info_list[0]
    frm = ci.header.frame_id
#    print 'frame:', frm
    b1 = ci.contact_body_1
    b2 = ci.contact_body_2
    contact_dict = {}
    pts_list = []
    for ci in cont_info_list:
        if frm != ci.header.frame_id:
            rospy.logerr('initial frame_id: %s and current frame_id: %s'%(frm, ci.header.frame_id))

        b1 = ci.contact_body_1
        b2 = ci.contact_body_2
        two_bodies = b1 + '+' + b2

        if two_bodies not in contact_dict:
            contact_dict[two_bodies] = []
        contact_dict[two_bodies].append((ci.position.x, ci.position.y, ci.position.z))

    return contact_dict

def skin_ros_msg(cd, fts):
    sc = SkinContact()
    f_dict = fts.get_forces()
    objects = f_dict.keys()

# if two links in contact with same object according to the geometric
# collision detector, then divide force equally between the different
# links.
    count_l = [0] * len(objects)
    count_dict = dict(zip(objects, count_l))

    for i, k in enumerate(cd.keys()):
        collided_objects = k.split('+')
        oid = collided_objects[1]
        #print 'oid:', oid #force_torque_ft<n>
        if oid not in f_dict:
            rospy.logerr('Unknown Object ID: ' + oid)
            continue
        count_dict[oid] += 1

    for i, k in enumerate(cd.keys()):
        pts = np.matrix(cd[k]).T
        mn = np.mean(pts, 1)
        collided_objects = k.split('+')

        oid = collided_objects[1]
        if oid not in f_dict:
            rospy.logerr('Unknown Object ID: ' + oid)
            continue

        if oid not in count_dict:
            # i am combining the last few links into one object and
            # these will not have separate items in the count_dict
            continue
        f = f_dict[oid] / count_dict[oid]

        if f == None or np.linalg.norm(f) < 0.8:
            # already recorded collision with this object, or no
            # collision with the object.
            continue

        sc.forces.append(Vector3(f[0,0], f[1,0], f[2,0]))
        f_mag = np.linalg.norm(f)
        sc.normals.append(Vector3(f[0,0]/f_mag, f[1,0]/f_mag, f[2,0]/f_mag))
        sc.link_names.append(collided_objects[0])

        sc.pts_x.append(FloatArrayBare(pts[0,:].A1))
        sc.pts_y.append(FloatArrayBare(pts[1,:].A1))
        sc.pts_z.append(FloatArrayBare(pts[2,:].A1))
        sc.locations.append(Point(mn[0,0], mn[1,0], mn[2,0]))

    sc.header.stamp = rospy.Time.now()
    sc.header.frame_id = '/torso_lift_link'

    #if len(sc.link_names) == 0:
    #    sc = None
    return sc


if __name__ == '__main__':
    import hrl_cody_arms.cody_arm_client as cac
    import optparse
    import cody_openrave_env as coe

    p = optparse.OptionParser()
    p.add_option('--obstacle_pkl', action='store', dest='obs_pkl', type='string',
                 help='pkl with info about the fixed obstacles.', default=None)
    p.add_option('--display', action='store_true', dest='display',
                 help='show GUI with the OpenRAVE models', default=False)
    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)
    opt, args = p.parse_args()

    rospy.init_node('cody_skin_simulate')
    skin_pub = rospy.Publisher('/skin/contacts', SkinContact)

    if opt.arm == None:
        rospy.logerr('Please specify an arm to use. Exiting...')
        sys.exit()

    #oid_list = ['force_torque_ft1', 'force_torque_ft2', 'force_torque_ft3', 'force_torque_ft4']
    cd = coe.skin_simulation_config_dict(opt.arm)
    oid_list = cd['obstacle_nm_l']
    netft_flag_list = cd['netft_flag_list']
    fts = ft_sensors.Object_FT_Sensors(oid_list, netft_flag_list)

    print 'Move the arm away from obstacles so that I can bias the FT sensors.'
    ut.get_keystroke('Hit any key when done.')
    fts.bias_fts()

    coe.write_xml(**cd)
    env_file = cd['xml_filename']
    col_det2 = co.Online_Collision_Detector(env_file, oid_list,
                                    cd['n_radius_steps'], opt.display)

    #ac = cac.CodyArmClient('r')
    ac = cac.CodyArmClient(opt.arm)
    #ac.kinematics.set_tooltip(np.matrix([0.,0.,-0.16]).T)

    col_det2.create_openrave_idx_list(ac)
    col_det2.init_obstacle_locations(opt.obs_pkl, opt.obs_pkl == None)

    while ac.get_joint_angles() == None:
        rospy.sleep(0.05)

    rospy.loginfo('Starting the skin simulation loop.')
    rt = rospy.Rate(100)
    while not rospy.is_shutdown():
        q = ac.get_joint_angles()
        try:
            contact_dict = col_det2.create_contact_dict_cody(q)
        except tf.ConnectivityException, e:
            rospy.logwarn(str(e))
            rt.sleep()
            continue
        sc = skin_ros_msg(contact_dict, fts)
        skin_pub.publish(sc)
        rt.sleep()



