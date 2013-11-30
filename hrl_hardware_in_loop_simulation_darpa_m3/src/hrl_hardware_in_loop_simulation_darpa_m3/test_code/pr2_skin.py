
import sys, copy
import numpy as np, math
from threading import RLock

import ati_ft as aft
import viz
import contacts_openrave as co

import roslib; roslib.load_manifest('darpa_m3')
import rospy
from mapping_msgs.msg import CollisionObject
from sensor_msgs.msg import JointState

import hrl_lib.viz as hv
import hrl_lib.transforms as tr

from darpa_m3.msg import SkinContact
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
    got_ft2, got_ft1 = False, False
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
#    count_l = [0] * len(objects)
#    for i, k in enumerate(cd.keys()):
#        collided_objects = k.split('+')
#        oid = collided_objects[1]
#        if oid in objects:
#            idx = objects.index(oid)
#            count_l[idx] += 1

# if two links in contact with same object according to the geometric
# collision detector, then skin will report the same force vector for
# both contacts.
    count_l = [1] * len(objects)
    count_dict = dict(zip(objects, count_l))

    for i, k in enumerate(cd.keys()):
        pts = np.matrix(cd[k]).T
        mn = np.mean(pts, 1)
        collided_objects = k.split('+')

        oid = collided_objects[1]
        if oid not in f_dict:
            rospy.logerr('Unknown Object ID: ' + oid)
            continue
        f = f_dict[oid] / count_dict[oid]

        if f == None or np.linalg.norm(f) < 0.5:
            # already recorded collision with this object, or no
            # collision with the object.
            continue

        sc.forces.append(Vector3(f[0,0], f[1,0], f[2,0]))
        sc.link_names.append(collided_objects[0])

        sc.pts_x.append(FloatArrayBare(pts[0,:].A1))
        sc.pts_y.append(FloatArrayBare(pts[1,:].A1))
        sc.pts_z.append(FloatArrayBare(pts[2,:].A1))
        sc.locations.append(Point(mn[0,0], mn[1,0], mn[2,0]))

    sc.header.stamp = rospy.Time.now()
    sc.header.frame_id = 'base_footprint'

    #if len(sc.link_names) == 0:
    #    sc = None
    return sc


if __name__ == '__main__':
    import pr2_arms.pr2_arms as pa

    rospy.init_node('pr2_skin_simulate')
    skin_pub = rospy.Publisher('/skin/contacts', SkinContact)

    oid_list = ['force_torque_ft1', 'force_torque_ft2']
    fts = aft.Object_FT_Sensors(oid_list)
    print 'Move the arm away from obstacles so that I can bias the FT sensors.'
    raw_input('Hit ENTER when done.')
    fts.bias_fts()

    env_file = '/home/advait/svn/robot1/src/projects/darpa_m3/viz/pr2_skin.env.xml'
    col_det2 = co.Online_Collision_Detector(env_file)

    pr2_arms = pa.PR2Arms()
    pr2_arms.arms.set_tooltip(arm, np.matrix([0.225, 0., 0.]).T)
    torso_height = pr2_arms.torso_position
    r_arm, l_arm = 0, 1
    arm = r_arm
    col_det2.register_obstacles(pr2_arms, arm, torso_height)

    rospy.loginfo('Starting the skin simulation loop.')
    rt = rospy.Rate(50)
    while not rospy.is_shutdown():
        q = pr2_arms.get_joint_angles(arm)

        contact_dict = col_det2.create_contact_dict_pr2(q)
        sc = skin_ros_msg(contact_dict, fts)
        skin_pub.publish(sc)

        rt.sleep()



