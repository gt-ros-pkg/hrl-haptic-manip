
import numpy as np, math
import copy

import roslib; roslib.load_manifest('hrl_software_simulation_darpa_m3')
import rospy
import hrl_lib.util as ut
import hrl_lib.transforms as tr
import equilibrium_point_control.epc as epc

# hrl_haptic_controllers_darpa_m3 has been deprecated and archived. Anything
# using it is now also deprecated, and broken (including this file)
from hrl_haptic_controllers_darpa_m3.epc_skin import SkinClient
import hrl_haptic_controllers_darpa_m3.mobile_skin_epc as mse


from hrl_msgs.msg import FloatArrayBare

class ode_SkinClient(SkinClient):
    def __init__(self, skin_topic_list):
        SkinClient.__init__(self, skin_topic_list)

    def force_normal_loc_joint_list(self, normal_component_only,
                                    return_time=False):
        f_l, n_l, nm_l, loc_l, stamp = self.get_snapshot()

        jt_l = []
        for i in range(len(f_l)):
            f = f_l[i]
            n = n_l[i]
            if normal_component_only:
                f_l[i] = n * np.linalg.norm(f)

            nm = nm_l[i]
            jt_num = int(nm[-1]) - 1 # assume that links are named link1 ...
            jt_l.append(jt_num)

        if return_time:
            return f_l, n_l, loc_l, jt_l, stamp
        else:
            return f_l, n_l, loc_l, jt_l

#___________________________________________________________________________________

class MobileBase_ODE(epc.EPC, mse.MobileBase):
    def __init__(self):
        epc.EPC.__init__(self, None)
        self.base_ep_pub = rospy.Publisher('/sim_arm/command/base_ep',
                                           FloatArrayBare)
        rospy.Subscriber('/sim_arm/base_ep', FloatArrayBare,
                         self.base_ep_cb)
        rospy.Subscriber('/sim_arm/odometry', FloatArrayBare,
                         self.odom_cb)
        self.ep_x = None
        self.ep_y = None
        self.ep_ang = None

        # changed to 0.,0.,0. on Jan 10, 2012 so that I can use the
        # same classs with and without a mobile base implemented
        # within demo_kinematic
        self.x = 0.
        self.y = 0.
        self.ang = 0.

    def base_ep_cb(self, msg):
        self.ep_x = msg.data[0]
        self.ep_y = msg.data[1]
        self.ep_ang = msg.data[2]

    def odom_cb(self, msg):
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.ang = msg.data[2]

    def publish_ep(self, x, y, ang):
        fa = FloatArrayBare([x, y, ang])
        self.base_ep_pub.publish(fa)

    def get_ep(self):
        return self.ep_x, self.ep_y, self.ep_ang

    def fwd(self, dist, blocking):
        v = np.matrix([dist, 0., 0.]).T
        self.go(v, 0., blocking)

    def back(self, dist, blocking):
        v = np.matrix([-dist, 0., 0.]).T
        self.go(v, 0., blocking)

    def left(self, dist, blocking):
        v = np.matrix([0., dist, 0.]).T
        self.go(v, 0., blocking)

    def right(self, dist, blocking):
        v = np.matrix([0., -dist, 0.]).T
        self.go(v, 0., blocking)

    # vec - 3x1 np vector in local coord frame.
    def go(self, vec, ang, blocking):
        if not blocking:
            raise RuntimeError('Non-blocking is unimplemented.')

        x, y, ang = self.x, self.y, self.ang
        target_pos = vec + np.matrix([x, y, 0.]).T

        def ep_gen_func(ep_gen):
            ep_x, ep_y, ep_ang = self.ep_x, self.ep_y, self.ep_ang
            x, y, ang = self.x, self.y, self.ang
            err = target_pos - np.matrix([x, y, 0.]).T
            err_mag = np.linalg.norm(err)
            err = err / err_mag * min(err_mag, 0.01)
            d_ep_x = err[0,0]
            d_ep_y = err[1,0]
            d_ep_ang = 0

            if err_mag < 0.005:
                stop = 'Reached'
            else:
                stop = ''

            return stop, (ep_x+d_ep_x, ep_y+d_ep_y, ep_ang+d_ep_ang)

        ep_gen = epc.EP_Generator(ep_gen_func, self.publish_ep)
        time_step = 0.5
        res = self.epc_motion(ep_gen, time_step)

#___________________________________________________________________________________

class MobileSkinEPC_ODE(mse.MobileSkinEPC):
    def __init__(self, robot, skin_client):
        mse.MobileSkinEPC.__init__(self, robot, skin_client)
        self.base = MobileBase_ODE()

    #---------------------------------
    # transforms etc.
    #---------------------------------

    ## given a position and orientation for the torso, find the
    # position of base in the /world frame.
    def compute_base_position(self, p_torso, rot_torso):
        return p_torso

    def current_torso_pose(self):
        x = self.base.x
        y = self.base.y
        a = self.base.ang
        trans = np.matrix([x, y, 0]).T
        rot = tr.rotZ(a)
        return trans, rot


    #--------- motion --------------

    def move_base_till_hit(self, fwd_distance):
        start_torso_pos = self.current_torso_pose()[0]
        time_step = 0.005

        def ep_gen_func(ep_gen):
            time_step = ep_gen.time_step
            jep = self.robot.get_ep()
            stop = ''

            now_torso_pos = self.current_torso_pose()[0]

            if np.linalg.norm(now_torso_pos - start_torso_pos) > fwd_distance:
                stop = 'moved forward quite a bit'

            f_mag_max = self.scl.max_force_mag()
            if f_mag_max > 1.:
                stop = 'hit something while moving'
                print 'f_mag_max:', f_mag_max
            
            if stop == '':
                ep_x, ep_y, ep_ang = self.base.get_ep()
                ep_x += 0.03 * time_step # speed in m/s 
                self.base.publish_ep(ep_x, ep_y, ep_ang)

            return (stop, (jep, time_step*1.5))

        ep_gen = epc.EP_Generator(ep_gen_func, self.robot.set_ep)
        ep_gen.time_step = time_step
        res = self.epc_motion(ep_gen, time_step)
        return res


if __name__ == '__main__':
    rospy.init_node('move_base_ODE')

    if False:
        # simple motion (go)
        mbo = MobileBase_ODE()
        ut.get_keystroke('Hit a key to send move command')
        v = np.matrix([-0.2, 0., 0.]).T
        mbo.go(v, 0., blocking = True)

    if True:
        import hrl_software_simulation_darpa_m3.gen_sim_arms as gsa
        import hrl_common_code_darpa_m3.robot_config.three_link_planar_capsule as d_robot

        # move_base_till_hit
        scl = ode_SkinClient(['/skin/contacts'])
        robot = gsa.ODESimArm(d_robot)
        mso = MobileSkinEPC_ODE(robot, scl)
        ut.get_keystroke('Hit a key to send move command')
        mso.move_base_till_hit(0.1)




