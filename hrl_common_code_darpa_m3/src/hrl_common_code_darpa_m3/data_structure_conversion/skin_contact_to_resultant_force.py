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

# October 2012
#
# Charlie wants this updated so that the resultant force is not 
# crossing the axis of the link, but the outer geometry of the link. 
# This update is only for cylindrical links and will break for any 
# other type of geometry. If you use this for other geometry, 
# beware!!! and update accordingly - marc


import numpy as np, math
import copy

import roslib; roslib.load_manifest('hrl_common_code_darpa_m3')
import hrl_lib.viz as hv
import hrl_lib.util as ut
import hrl_lib.transforms as tr

import rospy
import tf
import math

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
        return np.matrix([0,0,0]).T, None, np.matrix([0,0,0]).T

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

    return f_res, o + np.matrix(poc).T + l * f_res, tau_res

def check_line(p1, p2, p3, p4):
    # see http://paulbourke.net/geometry/lineline2d/ for equation meaning
    den = (p4[1]-p3[1])*(p2[0]-p1[0]) - (p4[0]-p3[0])*(p2[1]-p1[1])

    if den == 0:
        return False, None

    ua = ((p4[0] - p3[0]) * (p1[1] - p3[1]) - (p4[1] - p3[1]) * (p1[0] - p3[0])) / den
    ub = ((p2[0] - p1[0]) * (p1[1] - p3[1]) - (p2[1] - p1[1]) * (p1[0] - p3[0])) / den

    if ua >= 0 and ua <= 1 and ub >= 0 and ub <= 1:
        return True, (ua, ub)
    else:
        return False, (None, None)


def get_body_loc(loc, f_res_direc, nrml, line1_pt1, line1_pt2, line2_pt1, line2_pt2, o, o_next, radius):
    p1 = loc[0:2,:].A1
    p2 = (loc+f_res_direc*2)[0:2,:].A1

    (ptx, pty) = get_line_intersection(p1, p2, line1_pt1[0:2,:].A1, line1_pt2[0:2,:].A1)
    if ptx!= None:
        loc = np.matrix([[ptx, pty, 0.]]).reshape(3,1)
    else:
        (ptx, pty) = get_line_intersection(p1, p2, line2_pt1[0:2,:].A1, line2_pt2[0:2,:].A1)
        if ptx!= None:
            loc = np.matrix([[ptx, pty, 0.]]).reshape(3,1)
        else:
            pts = get_circle_intersection(p1, p2, o_next[0:2,:].A1, radius)

            if pts[0] != None or pts[1] != None:
                nrml = f_res_direc
                if pts[0] != None and pts[1] != None:
                    pt1 = np.matrix([[pts[0][0], pts[0][1], 0.]]).reshape(3,1)
                    pt2 = np.matrix([[pts[1][0], pts[1][1], 0.]]).reshape(3,1)
                    if np.linalg.norm(pt1-loc) > np.linalg.norm(pt2-loc):
                        loc = pt1
                    else:
                        loc = pt2
                else:
                    if pts[0] != None:
                        loc = np.matrix([[pts[0][0], pts[0][1], 0.]]).reshape(3,1)
                    else:
                        loc = np.matrix([[pts[1][0], pts[1][1], 0.]]).reshape(3,1)
            else:
                pts = get_circle_intersection(p1, p2, o[0:2,:].A1, radius)                            
                if pts[0] != None or pts[1] != None:
                    nrml = f_res_direc
                    if pts[0] != None and pts[1] != None:
                        pt1 = np.matrix([[pts[0][0], pts[0][1], 0.]]).reshape(3,1)
                        pt2 = np.matrix([[pts[1][0], pts[1][1], 0.]]).reshape(3,1)
                        if np.linalg.norm(pt1-loc) > np.linalg.norm(pt2-loc):
                            loc = pt1
                        else:
                            loc = pt2
                    else:
                        if pts[0] != None:
                            loc = np.matrix([[pts[0][0], pts[0][1], 0.]]).reshape(3,1)
                        else:
                            loc = np.matrix([[pts[1][0], pts[1][1], 0.]]).reshape(3,1)
                else:
                    # print "o  is :\n", o
                    # print "o_next  is :\n", o_next
                    # print "loc is :\n ", loc
                    # print "f_res_direc :\n", f_res_direc
                    # print "line1 pts are :\n", line1_pt1, line1_pt2
                    # print "line2 pts are :\n", line2_pt1, line2_pt2
                    print "there was no intersection at all, must be an error!!!"
                    assert(False)
    return loc, nrml

def check_circle(p1, p2, c, radius):
    # see http://paulbourke.net/geometry/sphereline/ for equations and diagram
    a = (p2[0] - p1[0])*(p2[0] - p1[0]) + (p2[1] - p1[1])*(p2[1] - p1[1])
    b = 2 * ( ( p2[0] - p1[0] ) * ( p1[0] - c[0] ) + ( p2[1] - p1[1] ) * ( p1[1] - c[1] ) )
    c = c[0]*c[0] + c[1]*c[1] + p1[0]*p1[0] + p1[1]*p1[1] - 2 * ( c[0] * p1[0] + c[1] * p1[1] ) - radius*radius
    
    if b*b-4*a*c <= 0:
        return False, (a, b, c)
    else:
        return True, (a, b, c)

def get_circle_intersection(p1, p2, sp, radius):
    # see http://paulbourke.net/geometry/sphereline/ for equations and diagram
    intersects, (a, b, c) = check_circle(p1, p2, sp, radius)
    if intersects:
        u1 = (-b + math.sqrt(b*b-4*a*c))/(2*a)
        u2 = (-b - math.sqrt(b*b-4*a*c))/(2*a)
        
        pt1 = None
        pt2 = None

        if u1 > 0 and u1 < 1:
            pt1 = np.array(p1) + u1*(np.array(p2)-np.array(p1))
        if u2 > 0 and u2 < 1:
            pt2 = np.array(p1) + u2*(np.array(p2)-np.array(p1))

        return (pt1, pt2)
    else:
        return (None, None)

def get_line_intersection(p1, p2, p3, p4):
    # see http://paulbourke.net/geometry/lineline2d/ for equation meaning
    intersects, (ua, ub) = check_line(p1, p2, p3, p4)
    if intersects:
        return (p1[0]+ua*(p2[0]-p1[0]), p1[1]+ua*(p2[1]-p1[1]))
    else:
        return (None, None)

def skin_contact_cb(sc, callback_args):
    sc_pub, kinematics, new_ft_sensor, body_intersection, radius, last_link = callback_args
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

            f_res, loc, tau_res_at_base = simulate_ft_sensor_on_link_base(np.column_stack(fc_l),
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

            # this is to fix the problem that our KDL chain goes all the way to the end-effector
            # while the rest of our results give joint positions in the middle of the capsule head.
            if nm == last_link:
                o_next = o_next - radius*link_direc
                link_length = link_length-radius

            dist_from_o =  np.linalg.norm(loc-o)
            dist_from_o_next = np.linalg.norm(loc-o_next)
            two_contacts = False

            # use this for more accurate ft approx, if force line of
            # action doesn't cross midline within body this adds two
            # more forces at opposite joints to compensate moment that
            # is ignored when the force is capped at the joint.
            if new_ft_sensor:   #use this for more accurate ft approx
                if dist_from_o > link_length or dist_from_o_next > link_length:
                    two_contacts = True

                    if dist_from_o > dist_from_o_next:
                        loc = o_next
                        f_o_next = (tau_res_at_base[2,0] - (nrml.T * f_res)[0,0]*link_length)/link_length*nrml
                        f_o = -f_o_next
                    else:
                        loc = o
                        f_o = -tau_res_at_base[2,0]/link_length*nrml
                        f_o_next = -f_o

                    if (nrml.T * f_o_next)[0,0] < 0.:
                        nrml_o_next = -1. * nrml
                        nrml_o = nrml
                    elif (nrml.T * f_o)[0,0] < 0.:
                        nrml_o_next = nrml
                        nrml_o = -1. * nrml

                    nrml = f_res_direc

            # this caps forces at end of links but puts all contact
            # forces where they cross the perimiter of the body
            # instead the mid-line axis.
            elif body_intersection:
                line_1_pt1 = o + nrml*radius
                line_1_pt2 = o_next + nrml*radius

                line_2_pt1 = o - nrml*radius
                line_2_pt2 = o_next - nrml*radius

                if dist_from_o > link_length or dist_from_o_next > link_length:
                    if dist_from_o > dist_from_o_next:
                        loc = o_next
                    else:
                        loc = o

                #Inside this function is messy, could probably clean it up eventually
                loc, nrml = get_body_loc(loc, f_res_direc, nrml, line_1_pt1, line_1_pt2, line_2_pt1, line_2_pt2, o, o_next, radius)

                dist_from_o =  np.linalg.norm(loc-o)
                dist_from_o_next = np.linalg.norm(loc-o_next)

            # no capping is happening here, just using the actual
            # direction of the resultant force, not the normal of the
            # surface
            else:
                nrml = f_res_direc
                

            if two_contacts == False:
                sf.link_names.append(nm)
                sf.forces.append(Vector3(f_res[0,0], f_res[1,0], f_res[2,0]))
                sf.locations.append(Point(loc[0,0], loc[1,0], loc[2,0]))
                sf.normals.append(Vector3(nrml[0,0], nrml[1,0], nrml[2,0]))
                sf.pts_x.append(FloatArrayBare([loc[0,0]]))
                sf.pts_y.append(FloatArrayBare([loc[1,0]]))
                sf.pts_z.append(FloatArrayBare([loc[2,0]]))
            else:
                sf.link_names.append(nm)
                sf.link_names.append(nm)
                sf.link_names.append(nm)
                sf.forces.append(Vector3(f_res[0,0], f_res[1,0], f_res[2,0]))
                sf.forces.append(Vector3(f_o[0,0], f_o[1,0], f_o[2,0]))
                sf.forces.append(Vector3(f_o_next[0,0], f_o_next[1,0], f_o_next[2,0]))
                sf.locations.append(Point(loc[0,0], loc[1,0], loc[2,0]))
                sf.locations.append(Point(o[0,0], o[1,0], o[2,0]))
                sf.locations.append(Point(o_next[0,0], o_next[1,0], o_next[2,0]))
                sf.normals.append(Vector3(nrml[0,0], nrml[1,0], nrml[2,0]))
                sf.normals.append(Vector3(nrml_o[0,0], nrml_o[1,0], nrml_o[2,0]))
                sf.normals.append(Vector3(nrml_o_next[0,0], nrml_o_next[1,0], nrml_o_next[2,0]))
                sf.pts_x.append(FloatArrayBare([loc[0,0]]))
                sf.pts_y.append(FloatArrayBare([loc[1,0]]))
                sf.pts_z.append(FloatArrayBare([loc[2,0]]))
                sf.pts_x.append(FloatArrayBare([o[0,0]]))
                sf.pts_y.append(FloatArrayBare([o[1,0]]))
                sf.pts_z.append(FloatArrayBare([o[2,0]]))
                sf.pts_x.append(FloatArrayBare([o_next[0,0]]))
                sf.pts_y.append(FloatArrayBare([o_next[1,0]]))
                sf.pts_z.append(FloatArrayBare([o_next[2,0]]))

            fc_l = []
            xc_l = []

        i+=1
    
    sc_pub.publish(sf)


def joint_angles_cb(fab):
    global q
    q = copy.copy(fab.data)



if __name__ == '__main__':
    import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot
    #import hrl_common_code_darpa_m3.robot_config.three_link_planar_cody as d_robot

    roslib.load_manifest('hrl_software_simulation_darpa_m3')
    import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa

    node_nm = 'skin_contact_to_resultant_force'
    rospy.init_node(node_nm)

    kinematics = gsa.RobotSimulatorKDL(d_robot)
    q = [0.,0.,0.]

    skin_topic = '/skin/contacts'
    sc_pub = rospy.Publisher(skin_topic, SkinContact)

    new_ft_sensor = False
    body_intersection = False
    radius = 0.015
    last_link = "link3"

    rospy.Subscriber('/skin/contacts_all', SkinContact,
                     skin_contact_cb,
                     callback_args = (sc_pub, kinematics, new_ft_sensor, body_intersection, radius, last_link))
    rospy.Subscriber('/sim_arm/joint_angles', FloatArrayBare,
                     joint_angles_cb)

    rospy.loginfo('Started skin_contact to resultant force!')
    rospy.spin()


