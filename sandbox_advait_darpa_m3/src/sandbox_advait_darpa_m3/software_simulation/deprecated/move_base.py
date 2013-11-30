
import numpy as np, math
import copy

import roslib; roslib.load_manifest('darpa_m3')
import rospy
import tf

import equilibrium_point_control.epc as epc

import hrl_lib.util as ut
import hrl_lib.transforms as tr
import hrl_lib.ode_utils as ou

from hrl_msgs.msg import FloatArrayBare


class MoveBase_ODE(epc.EPC):
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

        self.x = None
        self.y = None
        self.ang = None

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

    # vec - 3x1 np vector in local coord frame.
    def go(self, vec, ang, blocking=False):
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



if __name__ == '__main__':
    rospy.init_node('move_base_ODE')
    mbo = MoveBase_ODE()
    rospy.sleep(1.)

#    v = np.matrix([0., 0., 0.]).T
#    mbo.go_absolute(v, 0.)

    ut.get_keystroke('Hit a key to send move command')
    #mbo.print_rotation_axis_angle()

    v = np.matrix([0.2, 0., 0.]).T
    mbo.go(v, 0., blocking = True)




#------------------------------------------
# OLD CODE
#------------------------------------------

#    # go to equilibrium point in coord frame fixed to the world.
#    def go_absolute(self, vec, ang, blocking=False):
#        self.publish_ep(vec[0,0], vec[1,0], ang)
#        if blocking:
#            # have something that waits for the base to stop moving.
#            rospy.logwarn('Blocking is unimplemented.')
#            return


    #def debug_cb(self, msg):
    #    l = msg.data
    #    r = ou.ode_rotation_to_matrix(l)
    #    self.angle, self.axis = tr.matrix_to_axis_angle(r)
    #
    #def print_rotation_axis_angle(self):
    #    print 'axis:', self.axis
    #    print 'angle:', math.degrees(self.angle)


#    # vec - 3x1 np vector in local coord frame.
#    def go(self, vec, ang, blocking=False):
#        x_rel, y_rel = vec[0,0], vec[1,0]
#        rospy.logwarn('angle is unimplemented.')
#        # important - update only equilibrium point.
#        x = self.ep_x + x_rel
#        y = self.ep_y + y_rel
#        a = 0
#        self.publish_ep(x, y, a)
#        if blocking:
#            # have something that waits for the base to stop moving.
#            rospy.logwarn('Blocking is unimplemented.')
#            return






