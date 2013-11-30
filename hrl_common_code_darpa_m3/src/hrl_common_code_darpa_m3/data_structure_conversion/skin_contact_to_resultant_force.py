#!/usr/bin/python

#
# April 17, 2012
#
# This node will take as input the SkinContact message from the
# software simulation (which contains the contact locations and the
# full contact force vector), and will output another SkinContact
# message that has the force and contact location for the resultant
# force.
#
# The hope is to feed this into MPC and compare what whole-arm skin
# gives us relative to having a force-torque sensor at the base of
# each link.
#
#

import numpy as np, math
import copy

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.viz as hv
import hrl_lib.util as ut
import hrl_lib.transforms as tr

import rospy
import tf

from visualization_msgs.msg import Marker
from hrl_haptic_manipulation_in_clutter_msgs.msg import SkinContact

from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3


#
# Using Harvey Lipkin, Chapter 4, page 7, Poinsot's Wrench Theorem
#
# fc_mat - 3xN np matrix of contact forces
# xc_mat - 3xN np matrix of contact locations (torso_lift_link)
# o - 3x1 np matrix of point on the joint axis (torso_lift_link)
# o_next - 3x1 np matrix of point on end of the link (torso_lift_link)
def simulate_ft_sensor_on_link_base(fc_mat, xc_mat, o, o_next):
    f_res = fc_mat.sum(1)
    tau_res = np.matrix(np.cross((xc_mat-o).T.A, fc_mat.T.A)).T.sum(1)

    if np.linalg.norm(f_res) < 0.01:
        return np.matrix([0,0,0]).T, None

    poc = np.cross(f_res.A1, tau_res.A1) / (f_res.T * f_res)[0,0]
    nrml = np.cross(np.array([0.,0.,1.]), (o_next - o).A1)
    nrml = nrml / np.linalg.norm(nrml)

    # From Harvey Lipkin's notes, C is a point on the line of action
    # of the resultant force that is closest to O.
    # What I want is the point on the line of action that lies on the
    # link.
    # Let the vector from O to this desired point be Poc + l * f_res
    # where l is a scalar that I need to compute. For this vector to
    # be oriented along the link, it must be perpendicular to the
    # vector in the plane but normal to the link. This gives us:
    # (poc + l * f_res).T * nrml = 0
    # we can rearrange this equation to get:
    l = -np.dot(poc, nrml) / np.dot(nrml, f_res.A1)

    return f_res, o + np.matrix(poc).T + l * f_res

def skin_contact_cb(sc, callback_args):
    sc_pub, kinematics = callback_args
    sf = SkinContact() # sf - single force

    sf.header.frame_id = '/torso_lift_link' # has to be this and no other coord frame.
    sf.header.stamp = sc.header.stamp

    if len(sc.link_names) == 0:
        sc_pub.publish(sc)
        return

    fc_l = []
    xc_l = []
    
    # no guarantee that contacts for a particular link will be
    # together in the list. so, i am sorting the list and preserving
    # the indices to get the appropriate forces and locations.
    link_names = sc.link_names
    idxs = range(len(link_names))
    sorted_list = zip(link_names, idxs)
    sorted_list.sort()

    i = 0
    for nm, idx in sorted_list:
        jt_num = int(nm[-1]) - 1 # assume that links are named link1 ...

        fc_l.append(np.matrix([sc.forces[idx].x, sc.forces[idx].y, sc.forces[idx].z]).T)
        xc_l.append(np.matrix([sc.locations[idx].x, sc.locations[idx].y, sc.locations[idx].z]).T) 

        if i == len(sorted_list)-1 or sorted_list[i+1][0] != nm:
            o, _ = kinematics.FK(q, jt_num)
            o_next, _ = kinematics.FK(q, jt_num+1)
            f_res, loc = simulate_ft_sensor_on_link_base(np.column_stack(fc_l),
                                                         np.column_stack(xc_l),
                                                         o, o_next)
            if loc == None:
                continue

            nrml = np.cross(np.array([0.,0.,1.]), (o_next - o).A1)
            nrml = np.matrix(nrml / np.linalg.norm(nrml)).T
            if (nrml.T * f_res)[0,0] < 0.:
                nrml = -1. * nrml

            f_res_direc = f_res / np.linalg.norm(f_res)
            link_length = np.linalg.norm(o-o_next)
            link_direc = (o_next - o) / link_length
            if (link_direc.T * f_res_direc)[0,0] > 0.93:
                # here angle b/w nrml and f_res is < 20 degrees
                loc = o_next
            elif (link_direc.T * f_res_direc)[0,0] < -0.93:
                f_res = f_res * -1
                loc = o_next
                f_res_direc = -f_res_direc

            dist_from_o =  np.linalg.norm(loc-o)
            dist_from_o_next =  np.linalg.norm(loc-o_next)
            if dist_from_o > link_length or dist_from_o_next > link_length:
                if dist_from_o > dist_from_o_next:
                    loc = o_next
                else:
                    loc = o

            # contact close to the tip, assume circular tip and so
            # contact normal can be anything.
            if np.linalg.norm(loc-o_next) < 0.02:
                nrml = f_res_direc



            sf.link_names.append(nm)
            sf.forces.append(Vector3(f_res[0,0], f_res[1,0], f_res[2,0]))
            sf.locations.append(Point(loc[0,0], loc[1,0], loc[2,0]))
            sf.normals.append(Vector3(nrml[0,0], nrml[1,0], nrml[2,0]))
            sf.pts_x.append(FloatArrayBare([loc[0,0]]))
            sf.pts_y.append(FloatArrayBare([loc[1,0]]))
            sf.pts_z.append(FloatArrayBare([loc[2,0]]))

            fc_l = []
            xc_l = []

        i+=1
    
    sc_pub.publish(sf)


def joint_angles_cb(fab):
    global q
    q = copy.copy(fab.data)



if __name__ == '__main__':
    import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
    roslib.load_manifest('hrl_software_simulation_darpa_m3')
    import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa

    node_nm = 'skin_contact_to_resultant_force'
    rospy.init_node(node_nm)

    kinematics = gsa.RobotSimulatorKDL(d_robot)
    q = [0.,0.,0.]

    skin_topic = '/skin/contacts'
    sc_pub = rospy.Publisher(skin_topic, SkinContact)

    rospy.Subscriber('/skin/contacts_all', SkinContact,
                     skin_contact_cb,
                     callback_args = (sc_pub, kinematics))
    rospy.Subscriber('/sim_arm/joint_angles', FloatArrayBare,
                     joint_angles_cb)

    rospy.loginfo('Started skin_contact to resultant force!')
    rospy.spin()


