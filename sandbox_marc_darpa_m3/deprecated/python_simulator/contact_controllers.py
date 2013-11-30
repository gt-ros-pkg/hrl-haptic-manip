import numpy as np
import math

class ContactControllers:
    def __init__(self, dim):
#        self.alpha = 0.0005   #1 was the old normal value
        self.alpha = 1
        self.kp = 180/math.pi*600
#        self.kp_mat = np.matrix([[self.kp, 0],[0, self.kp]])
        self.kd = 180/math.pi*20   #old was 30 for small
        self.kp_mat = np.matrix(np.eye(dim))*self.kp

    def torque_control(self, q, q_eq, qdot, g):
        print "in controller"
#        print "q :", q, "q_eq :", q_eq, "qdot :", qdot, "g :", g
        torque = self.alpha*(-self.kp_mat*(q-q_eq)-self.kd*qdot) + g
        print "torque command :", torque
        return torque

    def calc_hand_position_planar(self, q, lengths, z):
        pos = np.matrix([0,0,0]).T
        for i in xrange(len(lengths)):
            pos = pos + np.matrix([lengths[i]*math.cos(np.sum(q[0:i+1])), lengths[i]*math.sin(np.sum(q[0:i+1])), 0]).T
        pos = pos + np.matrix([0, 0, z]).T
        return pos

    def calc_cost(self, h_g, h_cur, f_max, f_cur, alpha):
        if f_cur > f_max:
            print 'f_cur is greater than maximum allowed, you are not regulating it so I will'
            f_cur = f_max-f_max/100.0
        return 2*np.linalg.norm(h_cur-h_g)+alpha*(1.0/(1-(f_cur/f_max))-1)

    def calc_joint_cost(self, q_eq, q, f_max, f_cur, alpha):
        if f_cur > f_max:
            print 'f_cur is greater than maximum allowed, you are not regulating it so I will'
            f_cur = f_max-f_max/100.0
            return 2*np.linalg.norm(q-q_eq)+alpha*(1.0/(1-(f_cur/f_max))-1)
