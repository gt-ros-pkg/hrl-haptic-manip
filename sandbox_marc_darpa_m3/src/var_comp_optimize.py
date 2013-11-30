#!/usr/bin/env python  

import threading
import roslib; roslib.load_manifest('sandbox_marc_darpa_m3')
import rospy
import copy
import numpy as np
import itertools as it
import time 

#######################################
# this is used to do variable impedance
from hrl_haptic_manipulation_in_clutter_msgs.msg import MechanicalImpedanceParams


class VarCompOpt():
    def __init__(self):
        rospy.init_node('variable_compliance_optimizer')
        self.var_imped_pub = rospy.Publisher('/sim_arm/command/joint_impedance', MechanicalImpedanceParams)
        self.var_imped_sub = rospy.Subscriber('/sim_arm/joint_impedance', MechanicalImpedanceParams, self.get_impedance)
        self.impedance = MechanicalImpedanceParams()
        self.k_p = None
        self.k_d = None
        self.k_p_buf = None
        self.k_d_buf = None

        self.lock = threading.RLock()
        
        print  "about to wait ..."
        while self.k_p_buf == None:
            time.sleep(0.01)
        print "done waiting"
        self.k_p_nom = list(self.k_p_buf)
        self.k_d_nom = list(self.k_d_buf)

    def opt_func(self, k_p, *args):
        Jc_l = args[0]
        f_l = args[1]
        k_bounds = args[2]
        f_thresh = args[3]
        cost = 0.0
        K = np.matrix(np.diag(k_p))
        for i in xrange(len(Jc_l)):
            J = np.matrix(Jc_l[i])
            f = f_l[i]
            f_mag = np.linalg.norm(f)
            cost = cost - f.T*J*np.linalg.inv(K)*J.T*f
            # if f_mag > f_thresh:
            #     cost = cost - f.T*J*np.linalg.inv(K)*J.T*f

        # cost = cost + 0.0001*((K.diagonal()-np.matrix(self.k_p_nom))*(K.diagonal()-np.matrix(self.k_p_nom)).T)[0,0] #this is because the answer is a matrix still
        print "cost is : ", cost
        return cost

    def get_impedance(self, msg):
        self.lock.acquire()
        self.k_p_buf = list(msg.k_p.data)
        self.k_d_buf = list(msg.k_d.data)
        self.lock.release()


    def pub_once(self, k_p, k_d):
        self.impedance.k_p.data = k_p
        self.impedance.k_d.data = k_d
        count = 1
        while (count < 1000):
            self.var_imped_pub.publish(self.impedance)
            time.sleep(0.01)


if __name__ == '__main__':
    print "CALLED ME HERE"

    import optparse
    p = optparse.OptionParser()

    p.add_option('--optimize', action='store_true', dest='optimize',
                 default=False, help='run continuous optimization to vary compliance')
    p.add_option('--pub_once', action='store', dest='pub_once',
                 default=None, help='publish impedance until it takes, make sure you included [[kp1, kp2, kp3 ...],[kd1, kd2, kd3, ...]] as an argument')
    p.add_option('--pub_once_alpha', action='store', dest='pub_once_alpha',
                 default=None, help='change compliance parameters by a single gain, make sure you included alpha argument')

    opt, args = p.parse_args()

    variable_compliance = VarCompOpt()
    print "CALLED ME HERE too"

    if opt.pub_once != None:
        gains = eval(opt.pub_once)
        variable_compliance.pub_once(gains[0], gains[1])
    elif opt.pub_once_alpha != None:
        print "got in here as well"
        alpha = eval(opt.pub_once_alpha)
        new_kp = (alpha*np.array(variable_compliance.k_p_nom)).tolist()
        #THIS IS A FIXED NOMINAL DAMPING WHICH DOESN'T MAKE MUCH SENSE
        #new_kd = variable_compliance.k_d_nom

        #this damping is varying to keep the damping ratio for each joint constant
        # k_d_nom * sqrt(k_new/k_nom) for each joint individually 
        new_kd = (np.array(variable_compliance.k_d_nom)*np.sqrt(np.array(new_kp)/np.array(variable_compliance.k_p_nom))).tolist()
        variable_compliance.pub_once(new_kp, new_kd)
    else:
        print "you didn't specify an option, try the --help command"
