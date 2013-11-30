#!/usr/bin/python

# starting to write this along the lines of what Charlie had done in
# haptic_memory.py
# do not want to disturb that file, hence a new file with a different
# name (though I like haptic_memory more).

from threading import RLock

import sys
import numpy as np, math
import copy
import matplotlib.pyplot as pp

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')

import rospy

import hrl_lib.circular_buffer as cb
import hrl_lib.geometry as hg
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_lib.matplotlib_util as mpu

from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyResponse
from hrl_srvs.srv import FloatArray_Float, FloatArray_FloatResponse

from std_msgs.msg import Empty
from hrl_msgs.msg import FloatArray, FloatArrayBare
from geometry_msgs.msg import TransformStamped

#-----------------------------------------------------------
# ROS stuff
#-----------------------------------------------------------


# ROS interface for the contact memory
class ROSContactMemory():
    def __init__(self):
        rospy.Service('/contact_memory/srv/clear', Empty_srv,
                      self.clear_service_cb)
        rospy.Subscriber('/contact_memory/save', Empty, self.save_cb)

    # ROS service call to clear the contact memory.
    def clear_service_cb(self, req):
        self.clear()
        return EmptyResponse()

    def save_cb(self, msg):
        global torso_rot, torso_trans
        contact_mem_list = self.memory_to_pklable()
        #ut.save_pickle(contact_mem_list, 'contact_memory_'+ut.formatted_time()+'.pkl')

        cm_dict = {}
        cm_dict['contact_memory_list'] = contact_mem_list
        cm_dict['last_torso_position'] = torso_trans
        cm_dict['last_torso_rotation'] = torso_rot
        ut.save_pickle(cm_dict, 'contact_memory_'+ut.formatted_time()+'.pkl')

    def publish_contact_memory_locs(self):
        pts_x_l, pts_y_l, pts_z_l = [], [], []
        for ch in self.convex_hull_list():
            a = np.array(ch)
            pts_x_l.append(FloatArrayBare(a[:,0]))
            pts_y_l.append(FloatArrayBare(a[:,1]))
            pts_z_l.append(FloatArrayBare(a[:,0]*0.))

        cml = ContactMemoryLocs(pts_x_l, pts_y_l, pts_z_l)
        self.contact_mem_loc_pub.publish(cml)


def step_cb(data):
    global lock, step_cm_flag
    with lock:
        step_cm_flag = True


def torso_pose_cb(msg):
    global lock, torso_rot, torso_trans
    with lock:
        t = msg.transform.translation
        q = msg.transform.rotation

        torso_rot = tr.quaternion_to_matrix((q.x, q.y, q.z, q.w))
        torso_trans = np.matrix([t.x, t.y, t.z]).T


#-----------------------------------------------------------
# NOT ros stuff
#-----------------------------------------------------------

# pts: 3xn np matrix
# forces: 3xn np matrix
# normal: 3x1 np matrix
def estimate_stiffness(pts, forces, normal, visualize):
    force_scale = 100.
    loc_proj = (normal.T * pts).A1
    f_proj = (normal.T * forces).A1 / force_scale

    scatter_pts = np.vstack((loc_proj, f_proj))
    mn = np.mean(scatter_pts, 1)
    cov = np.cov(scatter_pts)

    U, sig, _ = np.linalg.svd(cov)
    k_estimate = (U[1,0] / U[0,0]) * force_scale
    sing_val_ratio = sig[1] / sig[0]

    contact_motion_std = np.std(loc_proj)

    if visualize:
        pp.figure()
        pp.title('Stiffness: %.2f, std motion: %.4f sing_val_ratio: %.3f'%(k_estimate, contact_motion_std, sing_val_ratio))
        pp.plot(loc_proj, f_proj, 'xb')
        pp.axis('equal')
        mpu.plot_ellipse_cov(mn, cov, 'k')

    return k_estimate, contact_motion_std, sing_val_ratio


# there will be a stiffness estimator object as a member variable in
# every ContactTracker
class StiffnessEstimator():
    def __init__(self, min_hist_to_estimate, hist_len):
        self.hist_len = hist_len
        self.min_hist_to_estimate = min_hist_to_estimate
        self.force_buf = cb.CircularBuffer(self.hist_len, (3,))
        self.loc_buf = cb.CircularBuffer(self.hist_len, (3,))
        self.normal = None
        self.stiffness_buf = cb.CircularBuffer(1000, ())
        self.current_estimated_stiffness = -1
        self.contact_motion_std_buf = cb.CircularBuffer(1000, ())
        self.stiffness_estimation_ratio_buf = cb.CircularBuffer(1000, ())
 
    def update(self, f, loc, nrml):
        self.force_buf.append(f.A1)
        self.loc_buf.append(loc.A1)
        self.normal = copy.copy(nrml)

    # if visualize is True then the estimated stiffness is not added
    # to the stiffness buf.
    def estimate_stiffness(self, visualize=False):
        if len(self.force_buf) < self.min_hist_to_estimate:
            return

        loc_mat = np.matrix(self.loc_buf.get_array()).T
        f_mat = np.matrix(self.force_buf.get_array()).T
        n = self.normal

        k_estimate, contact_motion_std, sing_val_ratio = estimate_stiffness(loc_mat, f_mat, n, False)

        k_estimate = abs(k_estimate)
        #if sing_val_ratio > 0.06 or contact_motion_std < 0.003 or \
        if sing_val_ratio > 0.15:
            # bad data, not estimating the stiffness
            k_estimate = np.inf
        else:
            self.current_estimated_stiffness = k_estimate
            #print '===================================='
            #print 'Estimated Stiffness:', k_estimate
            #print '===================================='

        self.stiffness_buf.append(k_estimate)
        self.contact_motion_std_buf.append(contact_motion_std)
        self.stiffness_estimation_ratio_buf.append(sing_val_ratio)

    def visualize_stiffness_estimate(self):
        self.estimate_stiffness(visualize=True)

    def to_dict(self):
        d = {}
        d['estimated_stiffness_list'] = self.stiffness_buf.to_list()
        d['contact_motion_std_list'] = self.contact_motion_std_buf.to_list()
        d['stiffness_estimation_ratio_list'] = self.stiffness_estimation_ratio_buf.to_list()
        return d

class ContactTracker():
    def __init__(self, min_hist_to_estimate_stiffness):
        self.hist_size = 5000
        self.contact_normal_buf = cb.CircularBuffer(self.hist_size, (3,))
        self.force_buf = cb.CircularBuffer(self.hist_size, (3,))
        self.loc_buf = cb.CircularBuffer(self.hist_size, (3,))
        self.convex_hull = None # Mx2 numpy array
        self.stamp_buf = cb.CircularBuffer(self.hist_size, ())
        self.stiffness_estimator = StiffnessEstimator(min_hist_to_estimate_stiffness, hist_len = 20)

    def update(self, f, loc, stamp, n):
        self.contact_normal_buf.append(n.A1)
        self.force_buf.append(f.A1)
        self.loc_buf.append(loc.A1)
        self.stamp_buf.append(stamp)

        self.stiffness_estimator.update(f, loc, n)
        self.stiffness_estimator.estimate_stiffness()

        if self.convex_hull == None:
            pts = loc.A1[0:2].reshape((1,2))
        else:
            pts = np.row_stack((self.convex_hull, loc.A1[0:2]))
        self.convex_hull = hg.convex_hull(pts)

    def k(self):
        return self.stiffness_estimator.current_estimated_stiffness

    ## can make this more complex later.
    def distance(self, loc, stamp):
        if len(self.loc_buf) >= 1:
            p_loc = np.matrix(self.loc_buf[-1]).T
            return np.linalg.norm(loc-p_loc)
        else:
            return None

    ## return a dict that can be pkled.
    def to_dict(self):
        d = {}
        d['contact_normals_list'] = self.contact_normal_buf.to_list()
        d['force_list'] = self.force_buf.to_list()
        d['loc_list'] = self.loc_buf.to_list()
        d['stamp_list'] = self.stamp_buf.to_list()
        d['convex_hull'] = self.convex_hull.tolist()
        d.update(self.stiffness_estimator.to_dict())
        return d

    def centroid(self):
        return np.mean(np.column_stack(self.loc_buf.to_list()), 1)



# collection of ContactTracker objects. One for each contact. yet to decide
# what I will do once contacts become old.
# There might possibly be multiple implementations of ContactMemory.
# All Advait hopes for is that each is derived from ROSContactMemory,
# thereby providing the same ROS interface.
class SimpleContactMemory(ROSContactMemory):
    def __init__(self, min_hist_to_estimate_stiffness):
        ROSContactMemory.__init__(self)
        rospy.Service('/contact_memory/estimate_contact_stiffness', FloatArray_Float,
                      self.estimate_stiffness_cb)
        self.min_hist_to_estimate_stiffness = min_hist_to_estimate_stiffness
        self.clear()

    ## update memory given measured contact state.
    # f_l - list of force vectors.
    # loc_l - kist of contact locations.
    # stamp - time of measured contact state.
    # n_l - list of contact normals.
    def update(self, f_l, loc_l, stamp, n_l):
        n = len(f_l)
        for i in range(n):
            tr = self.find_tracker(loc_l[i], stamp)
            tr.update(f_l[i], loc_l[i], stamp, n_l[i])

    ## find tracker corresponding to this contact. create new tracker
    # if this is a new contact.
    def find_tracker(self, loc, stamp, create_new=True):
        #min_dist = 0.03
        # on a whim, advait changed this threshold from 0.03 to 0.01
        # on Oct 12, 2011.
        min_dist = 0.01
        the_tr = None
        for tr in self.trackers:
            d = tr.distance(loc, stamp)
            if (d != None) and (d < min_dist):
                min_dist = d
                the_tr = tr
        if the_tr == None and create_new:
            # new contact.
            the_tr = ContactTracker(self.min_hist_to_estimate_stiffness)
            self.trackers.append(the_tr)
        return the_tr

    def clear(self):
        self.trackers = []
        self.n_trackers = 0

    def memory_to_pklable(self):
        tr_dict_list = []
        for trk in self.trackers:
            d = trk.to_dict()
            tr_dict_list.append(d)
        return tr_dict_list
        
    def convex_hull_list(self):
        return [trk.convex_hull.tolist() for trk in self.trackers]

    def fixed_obstacle_centroids(self):
        return [trk.centroid() for trk in self.trackers]

    #--------------- ROS stuff -------------------
    def estimate_stiffness_cb(self, req):
        loc = np.matrix(req.val[0:3]).T
        stamp = req.val[3]
        tr = self.find_tracker(loc, stamp, create_new=False)
        if tr == None:
            k = -1
        else:
            k = tr.k()
        return FloatArray_FloatResponse(k)


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()

    p.add_option('--cody', action='store_true', dest='cody',
                 help='contact memory for cody')
    p.add_option('--arm_to_use', action='store', dest='arm',
                 type='string', help='which arm to use (l or r)',
                 default=None)

    p.add_option('--meka_sensor', action='store_true', dest='meka_sensor',
                 help='use Meka forearm sensor with Cody')
    p.add_option('--fabric_sensor', action='store_true', dest='fabric_sensor',
                 help='use HRL fabric sensor with Cody')
    p.add_option('--hil', action='store_true', dest='hil',
                 help='hardware-in-loop simulation with Cody')

    p.add_option('--sim3', action='store_true', dest='sim3',
                 help='contact memory in software simulation')
    p.add_option('--sim3_with_hand', action='store_true',
                 dest='sim3_with_hand',
                 help='contact memory in software simulation')
    p.add_option('--sim6', action='store_true', dest='sim6',
                 help='contact memory in software simulation')

    opt, args = p.parse_args()

    if opt.cody:
        if opt.meka_sensor:
            skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_ft']
        elif opt.fabric_sensor:
            skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_wrist']
        elif opt.hil:
            skin_topic_list = ['/skin/contacts']
        else:
            raise RuntimeError('Missing command line argument for the testbed')
    else:
        skin_topic_list = ['/skin/contacts']

    global lock, step_cm_flag, torso_trans, torso_rot
    step_cm_flag = False
    lock = RLock()
    torso_rot, torso_trans = None, None

    rospy.init_node('contact_memory_node')

    if opt.cody:
        from sandbox_advait_darpa_m3.cody.cody_guarded_move import Cody_SkinClient

        if opt.arm == None:
            rospy.logerr('Please specify an arm to use. Exiting...')
            sys.exit()

        skin_client = Cody_SkinClient(skin_topic_list)

        jep_cmd_topic = '/'+opt.arm+'_arm/command/jep'
        torso_pose_topic = '/cody_torso_trackable/pose'

        rospy.Subscriber(jep_cmd_topic, FloatArray, step_cb)
        rospy.Subscriber(torso_pose_topic, TransformStamped, torso_pose_cb)

        min_hist_to_estimate_stiffness = 10

    elif opt.sim3 or opt.sim3_with_hand or opt.sim6:
        from hrl_software_simulation_darpa_m3.ode_sim_guarded_move import ode_SkinClient

        skin_client = ode_SkinClient(skin_topic_list)
        jep_cmd_topic = '/sim_arm/command/jep'
        # sync logging and task monitoring with new cmds.
        rospy.Subscriber(jep_cmd_topic, FloatArrayBare, step_cb)
        # for some strange reason we are publishing JEPs, joint angles
        # etc as FloatArrayBare in simulation. (It looks like I did
        # that)
        min_hist_to_estimate_stiffness = 5

    else:
        rospy.logerr('Please specify a testbed.\nExiting ...')
        sys.exit()

    contact_mem = SimpleContactMemory(min_hist_to_estimate_stiffness)

    if opt.sim3 or opt.sim3_with_hand or opt.sim6:
        rospy.logwarn('Need to implement torso position tracking for software simulation.')
        torso_trans = np.matrix([0.,0.,0.]).T
        torso_rot = np.matrix(np.eye(3))

    rospy.loginfo('Waiting to receive torso pose')
    while torso_trans == None or torso_rot == None:
        rospy.sleep(0.1)
    rospy.loginfo('received torso pose')

    rospy.loginfo('Started Contact Memory.')

    rt = rospy.Rate(150)
    while not rospy.is_shutdown():
        if step_cm_flag:
            step_cm_flag = False
            f_l, n_l, loc_l, _, stamp = skin_client.force_normal_loc_joint_list(False, True)
            with lock:
                p = copy.copy(torso_trans)
                r = copy.copy(torso_rot)
            world_loc_l = [p + r * loc for loc in loc_l]
            world_f_l = [r * f for f in f_l]
            world_n_l = [r * n for n in n_l]
            contact_mem.update(world_f_l, world_loc_l, stamp, world_n_l)
        rt.sleep()



