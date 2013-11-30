#!/usr/bin/env python

import roslib; roslib.load_manifest('sandbox_marc_darpa_m3')
import rospy
from sandbox_marc_darpa_m3.srv import *  #OptTestCallResponse, OptTestCall
import cPickle as pkl
import numpy as np
from hrl_lib.msg import FloatArrayBare

data = pkl.load(file('delta_qp_opt_dict.pkl'))
time_ind = data.keys()
time_ind.sort()
rospy.init_node('mpc_test_data_server')
jep_stored = FloatArrayBare()
pub = rospy.Publisher("/sim_arm/delta_jep_mpc_slow", FloatArrayBare)

def send_data(req):
    res = OptTestCallResponse()
    data_now = data.pop(time_ind.pop(0))
    res.delta_x_d = data_now['delta_x_g'].A1.tolist()
    res.J = data_now['J_h'].A1.tolist()
    res.x_0 = list(data_now['q'])
    res.I = [1.0, 0., 0., 0., 1., 0., 0., 0., 1.]
    res.KP_t_KP = (data_now['K_j'].T*data_now['K_j']).A1.tolist()
    res.q_min = np.radians(np.array([-150., -63, 0.])).tolist()
    res.q_max = np.radians(np.array([150., 162, 159.])).tolist()
    res.u_min = np.radians(np.array([-150., -63, 0.])).tolist()
    res.u_max = np.radians(np.array([150., 162, 159.])).tolist()
    res.f_min = data_now['delta_f_min'].A1.tolist()
    res.f_max = data_now['delta_f_max'].A1.tolist()

    sum_matrix = np.matrix(np.zeros((3,3)))
    for ii in xrange(len(data_now['Jc_l'])):
        sum_matrix = sum_matrix+data_now['Jc_l'][ii].T*data_now['Kc_l'][ii]*data_now['Jc_l'][ii]
    res.B = (np.linalg.inv(data_now['K_j']+sum_matrix)*data_now['K_j']).A1.tolist()

    n_K_ci_J_ci_buff = []
    for ii in xrange(len(data_now['Jc_l'])):
        buff_list = (data_now['n_l'][ii].T*data_now['Kc_l'][ii]*data_now['Jc_l'][ii]).A1.tolist()
        n_K_ci_J_ci_buff.append(buff_list)
    res.n_K_ci_J_ci = np.array(n_K_ci_J_ci_buff).flatten().tolist()

    jep_stored.data = data_now['delta_phi_opt']

    pub.publish(jep_stored)

    return res

if __name__ == "__main__":
    s = rospy.Service('get_mpc_data', OptTestCall, send_data)
    print "Server for mpc test data is running ..."
    rospy.spin()
