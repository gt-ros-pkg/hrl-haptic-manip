

import numpy as np, math
import copy
from threading import RLock

import roslib; roslib.load_manifest('darpa_m3')
import rospy
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_lib.viz as hv

import tf

from darpa_m3.msg import TaxelArray


class Taxels():
    # size - in meters.
    def __init__(self, taxel_size, length, radius, frame):
        self.taxel_size = taxel_size
        self.length = length
        self.radius = radius
        self.create_taxels(taxel_size, length, radius)
        self.taxel_array_pblshr = rospy.Publisher('/skin/taxel_array', TaxelArray)
        self.frame = frame

    def create_taxels(self, taxel_size, length, radius):
        # indexed by z_index, theta._index
        n_z = int((length / taxel_size) + 0.5)
        n_circum = int(2*math.pi*radius / taxel_size + 0.5)
        z_step = taxel_size
        theta_step = (math.pi * 2.) / n_circum

        self.z_step = z_step
        self.theta_step = theta_step

        self.centers = np.zeros((n_z, n_circum, 3))
        self.normals = np.zeros((n_z, n_circum, 3))
        self.forces = np.zeros((n_z, n_circum, 3))
        for i in range(n_z):
            for j in range(n_circum):
                self.centers[i,j,0] = radius*math.cos(j*theta_step)
                self.centers[i,j,1] = radius*math.sin(j*theta_step)
                self.centers[i,j,2] = i*z_step

                self.normals[i,j,0] = math.cos(j*theta_step)
                self.normals[i,j,1] = math.sin(j*theta_step)
                self.normals[i,j,2] = 0.

    def publish_taxel_array(self, sc):
        link_names = sc.link_names
        frame = sc.header.frame_id
        stamp = sc.header.stamp
        self.clear_forces()

        for i in range(len(link_names)):
            if link_names[i] !=  'wrist_RIGHT':
                continue

            pts = np.matrix([sc.pts_x[i].data, sc.pts_y[i].data,
                             sc.pts_z[i].data])
            l = sc.locations[i]
            mn = np.matrix([l.x, l.y, l.z]).T

            ff = sc.forces[i]
            f = np.matrix([ff.x, ff.y, ff.z]).T

            #trans, quat = tf_lstnr.lookupTransform(self.frame, frame, stamp)
            trans, quat = tf_lstnr.lookupTransform(forearm_taxels.frame, frame, rospy.Time(0))
            rot = tr.quaternion_to_matrix(quat)
            trans = np.matrix(trans).reshape(3,1)
            f = rot * f
            mn = trans + rot * mn
            self.map_to_taxel(f, mn)

        shp = self.centers.shape
        pts = self.centers.reshape(shp[0]*shp[1], 3)
        nrmls = self.normals.reshape(shp[0]*shp[1], 3)
        fs = self.forces.reshape(shp[0]*shp[1], 3)

        ta = TaxelArray()
        ta.header.stamp = stamp
        ta.header.frame_id = self.frame

        ta.centers_x = pts[:,0].tolist()
        ta.centers_y = pts[:,1].tolist()
        ta.centers_z = pts[:,2].tolist()

        ta.normals_x = nrmls[:,0].tolist()
        ta.normals_y = nrmls[:,1].tolist()
        ta.normals_z = nrmls[:,2].tolist()

        ta.forces_x = fs[:,0].tolist()
        ta.forces_y = fs[:,1].tolist()
        ta.forces_z = fs[:,2].tolist()

        self.taxel_array_pblshr.publish(ta)

    # force and loc are in the link coord frame.
    def map_to_taxel(self, f, loc):
        z_idx = int(loc[2,0] / self.z_step)
        theta = math.atan2(loc[1,0], loc[0,0])
        theta_idx = int(theta / self.theta_step) # between -n_circum and + n_circum
        nrm = self.normals[z_idx, theta_idx]
        if np.dot(f.A1, nrm) < 0:
            # flip the taxel to the diagonally opposite one.
            theta_idx += self.normals.shape[1]
        self.forces[z_idx, theta_idx, 0] = f[0,0]
        self.forces[z_idx, theta_idx, 1] = f[1,0]
        self.forces[z_idx, theta_idx, 2] = f[2,0]

    def clear_forces(self):
        self.forces[:,:,:] = 0.


def skin_cb(sc):
    forearm_taxels.publish_taxel_array(sc)


if __name__ == '__main__':
    from darpa_m3.msg import SkinContact

    rospy.init_node('taxel_simulator')
    forearm_taxels = Taxels(0.02, 0.22, 0.04, 'wrist_RIGHT')
    tf_lstnr = tf.TransformListener()

    skin_topic = '/skin/contacts'
    rospy.Subscriber(skin_topic, SkinContact, skin_cb)

    rospy.spin()

#    while not rospy.is_shutdown():
#        forearm_taxels.publish_taxel_markers()
#        rospy.sleep(0.03)


#    def publish_taxel_markers_old(self):
#        markers = MarkerArray()
#        shp = self.centers.shape
#        pts = self.centers.reshape(shp[0]*shp[1], 3)
#        nrmls = self.normals.reshape(shp[0]*shp[1], 3)
#        fs = self.forces.reshape(shp[0]*shp[1], 3)
#        t_now = rospy.Time.now()
#
#        force_marker_scale = 0.04
#
#        for i in range(pts.shape[0]):
#            p = np.matrix(pts[i]).T
#            n1 = np.matrix(nrmls[i]).T
#            n2 = np.matrix(fs[i]).T
#
#            q1 = hv.arrow_direction_to_quat(n1)
#            l1 = (n2.T * n1)[0,0] * force_marker_scale
#            m = hv.single_marker(p, q1, 'arrow', self.frame,
#                                 duration=0.5, scale=(l1, 0.06,0.06),
#                                 m_id=2*i)
#            m.header.stamp = t_now
#            markers.markers.append(m)
#
#            q2 = hv.arrow_direction_to_quat(n2)
#            l2 = np.linalg.norm(n2) * force_marker_scale
#            m = hv.single_marker(p, q2, 'arrow', self.frame,
#                                 duration=0.5, scale=(l2, 0.06,0.06),
#                                 color=(0.,1.,0.,0.5), m_id=2*i+1)
#            m.header.stamp = t_now
#            markers.markers.append(m)
#
#            #self.marker_pub.publish(m)
#        self.marker_array_pub.publish(markers)


