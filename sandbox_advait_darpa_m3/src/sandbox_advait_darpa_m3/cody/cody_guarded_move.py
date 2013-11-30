
import numpy as np, math
import copy
from threading import RLock

import roslib; roslib.load_manifest('sandbox_advait_darpa_m3')
roslib.load_manifest('segway_vo')

import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.transforms as tr

import equilibrium_point_control.epc as epc
import segway_vo.segway_command as sc

import hrl_cody_arms.cody_arm_client as cac
import hrl_haptic_controllers_darpa_m3.epc_skin as es
import hrl_haptic_controllers_darpa_m3.mobile_skin_epc as mse

from geometry_msgs.msg import TransformStamped

class Cody_SkinClient(es.SkinClient):
    def __init__(self, skin_topic_list):
        es.SkinClient.__init__(self, skin_topic_list)

    # list of force vectors, list of normal vectors, list of joint number after which the
    # joint torque will have no effect on the contact force.
    def force_normal_loc_joint_list(self, normal_component_only,
                                    return_time=False):
        f_l, n_l, nm_l, loc_l, stamp = self.get_snapshot()

        #self.lock.acquire()
        #f_l = copy.copy(self.force_list)
        #n_l = copy.copy(self.normal_list)
        #nm_l = copy.copy(self.link_names)
        #loc_l = copy.copy(self.loc_list)
        #stamp = copy.copy(self.stamp)
        #self.lock.release()

        jt_l = []
        for i in range(len(f_l)):
            f = f_l[i]
            n = n_l[i]
            if normal_component_only:
                f_l[i] = n * np.linalg.norm(f)

            nm = nm_l[i]
            # link names are from the Cody URDF.
            if 'shoulderupper' in nm:
                jt_num = 0
            elif 'shoulderpitch' in nm:
                jt_num = 1
            elif 'bicep' in nm:
                jt_num = 2
            elif 'elbowclevis' in nm:
                jt_num = 3
            elif 'wrist' in nm:
                jt_num = 3
#            elif 'handmount' in nm:
#                jt_num = 5
            else:
                jt_num = 6
            jt_l.append(jt_num)

        if return_time:
            return f_l, n_l, loc_l, jt_l, stamp
        else:
            return f_l, n_l, loc_l, jt_l

#___________________________________________________________________________________

class MobileBase_Cody(mse.MobileBase):
    def __init__(self):
        mse.MobileBase.__init__(self)

        try:
            rospy.init_node('move_segway')
        except rospy.ROSException, e:
            pass

        self.segway_command_node = sc.SegwayCommand(vel_topic='/hrl/segway/command',
                                        pose_local_topic='/hrl/segway/pose_local',
                                        pose_global_topic='/hrl/segway/pose_global',
                                        stop_topic='/hrl/segway/stop',
                                        max_vel_topic='/hrl/segway/max_vel',
                                        ros_init_node=False)
        rospy.sleep(1.)
        self.segway_command_node.set_max_velocity(0.1, 0.08, math.radians(5))
        self.xtol = 0.03
        self.ytol = 0.03
        self.atol = math.radians(5)
        self.moving = False

    def left(self, dist, blocking):
        dist = dist + self.ytol
        self.moving = True
        self.segway_command_node.go_xya_pos_local(0., dist, 0.,
                                                  blocking = False)
        if blocking:
            self.segway_command_node.wait_for_tolerance_pos(self.xtol,
                                                    self.ytol, self.atol)
            self.moving = False

    def right(self, dist, blocking):
        dist = dist + self.ytol
        self.moving = True
        self.segway_command_node.go_xya_pos_local(0., -dist, 0.,
                                                  blocking=False)
        if blocking:
            self.segway_command_node.wait_for_tolerance_pos(self.xtol,
                                                    self.ytol, self.atol)
            self.moving = False

    def back(self, dist, blocking):
        dist = dist + self.xtol
        self.moving = True
        self.segway_command_node.go_xya_pos_local(-dist, 0., 0.,
                                                  blocking = False)
        if blocking:
            self.segway_command_node.wait_for_tolerance_pos(self.xtol,
                                                    self.ytol, self.atol)
            self.moving = False

    def fwd(self, dist, blocking):
        dist = dist + self.xtol
        self.moving = True
        self.segway_command_node.go_xya_pos_local(dist, 0., 0.,
                                                  blocking = False)
        if blocking:
            self.segway_command_node.wait_for_tolerance_pos(self.xtol,
                                                    self.ytol, self.atol)
            self.moving = False
    
    # vec - 3x1 np vector in local coord frame.
    def go(self, vec, ang, blocking):
        x = vec[0,0] + np.sign(vec[0,0])*self.xtol*0.5
        y = vec[1,0] + np.sign(vec[1,0])*self.ytol*0.5
        self.moving = True
        self.segway_command_node.go_xya_pos_local(x, y, ang,
                                                  blocking = False)
        if blocking:
            self.segway_command_node.wait_for_tolerance_pos(self.xtol,
                                                    self.ytol, self.atol)
            self.moving = False

    def stop(self):
        self.segway_command_node.stop()
        self.moving = False

    def is_moving(self):
        if self.segway_command_node.within_tolerance_pos(self.xtol,
                                                self.ytol, self.atol):
            self.stop()
            self.moving = False
        return self.moving

    def set_velocity(self, xvel, yvel, avel):
        self.segway_command_node.set_velocity(xvel, yvel, avel)

#___________________________________________________________________________________

class MobileSkinEPC_Cody(mse.MobileSkinEPC):
    def __init__(self, robot, skin_client):
        mse.MobileSkinEPC.__init__(self, robot, skin_client)

        self.torso_trans = None
        self.torso_rot = None
        self.lock = RLock()

        rospy.Subscriber('/cody_torso_trackable/pose',
                         TransformStamped, self.cody_trackable_pose_cb)

        self.base = MobileBase_Cody()

    #---------------------------------
    # ROS callbacks
    #---------------------------------
    
    def cody_trackable_pose_cb(self, msg):
        with self.lock:
            t = msg.transform.translation
            q = msg.transform.rotation

            # A huge assumption here is that the /cody_torso_trackable
            # frame and the /torso_lift_link frame are coincident.
            # see darpa_m3/launch/optitrak.launch
            self.torso_rot = tr.quaternion_to_matrix((q.x, q.y, q.z, q.w))
            self.torso_trans = np.matrix([t.x, t.y, t.z]).T

    #---------------------------------
    # transforms etc.
    #---------------------------------

    ## given a position and orientation for the torso, find the
    # position of base in the /world frame.
    def compute_base_position(self, p_torso, rot_torso):
        offset = np.matrix([0.25, 0., 0.]).T # torso origin in base_link (static)
        #p_base + rot_torso * offset =  p_torso
        p_base = p_torso - rot_torso * offset
        return p_base

    def current_torso_pose(self):
        with self.lock:
            trans = copy.copy(self.torso_trans)
            rot = copy.copy(self.torso_rot)
            return trans, rot

    #--------- motion --------------

    def move_base_till_hit(self, fwd_distance):
        start_torso_pos = self.current_torso_pose()[0]
        time_step = 0.005
        #fwd_distance = 0.2

        def ep_gen_func(ep_gen):
            time_step = ep_gen.time_step
            jep = self.robot.get_ep()
            stop = ''

            now_torso_pos = self.current_torso_pose()[0]

            if np.linalg.norm(now_torso_pos - start_torso_pos) > fwd_distance:
                stop = 'moved forward quite a bit'

            f_mag_max = self.scl.max_force_mag()
            if f_mag_max > 1.:
                stop = 'hit something while moving the segway'
                print 'f_mag_max:', f_mag_max
            
            if stop == '':
                ep_gen.base_command_counter += 1
                if ep_gen.base_command_counter == 20:
                    ep_gen.base_command_counter = 0
                    self.base.set_velocity(0.08, 0., 0.)
            else:
                self.base.set_velocity(-0.08, 0., 0.)
                self.base.stop()
                # if arm is in a reasonable configuration, then forces
                # do not appear to become very large. I am not going
                # to change the jep.
                #q = self.robot.get_joint_angles()
                #p = self.robot.kinematics.FK(q) [0]
                #J_full = self.robot.kinematics.Jacobian(q, p)
                #J = J_full[0:3, :]
                #vec = np.matrix([-0.02, 0., 0.]).T
                #d_jep = np.linalg.pinv(J) * vec
                #jep = (np.matrix(jep).T + d_jep).A1
                # this is an extreme situation, so I am setting the ep
                # within the generator function.
                #self.robot.set_ep(jep)

            return (stop, (jep, time_step*1.5))

        ep_gen = epc.EP_Generator(ep_gen_func, self.robot.set_ep)
        ep_gen.time_step = time_step
        ep_gen.base_command_counter = 0
        res = self.epc_motion(ep_gen, time_step)
        return res


if __name__ == '__main__':
    rospy.init_node('cody_guarded_move')

    if True:
        skin_topic_list = ['/skin/contacts_forearm', '/skin/contacts_ft']
        scl = Cody_SkinClient(skin_topic_list)

        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            f_l, n_l, _, _ = scl.force_normal_loc_joint_list(True)
            fmag_l = [np.linalg.norm(f) for f in f_l]
            print 'f_mag_l:', fmag_l


    if False:
        # testing mobile base.
        mbc = MobileBase_Cody()
        mbc.back(0.2)
        mbc.left(0.10)
        mbc.fwd(0.2)









