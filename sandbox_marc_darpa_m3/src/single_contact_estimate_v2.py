import roslib
roslib.load_manifest("sandbox_marc_darpa_m3")
import rospy

import hrl_lib.transforms as tr
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import geometry_msgs.msg
import hrl_haptic_mpc.haptic_mpc_util as mpc_util
import hrl_haptic_mpc.multiarray_to_matrix as multiarray_to_matrix
import time

import numpy as np
import threading, copy
from matplotlib import pyplot as pl
import sys
import threading

lock = threading.RLock()
ind = None
R = 0.00005  #best .22 for 10,  0.022 for 100, 
# did 10 and got 260 with kp = 100. as initial
#did 0.1 and got 470 with kp = 0. as initial 
#kp = np.matrix([[0., 0]]).reshape(2,1)
kp = np.matrix([[500.]])
actual_kp = 1000
#P = np.matrix([[10000000000000., 0.], [0., 10000000000000.]])
P = np.matrix([[1000.]])
prev_x = None
prev_x_dot = None
prev_F = None
kp_list = []
diff_list = []
diff_list_est = []
q_dot = []
Jc_l = []
ma_to_m = multiarray_to_matrix.MultiArrayConverter()
time_list = []

def stateCallback(msg):
    global q_dot, Jc_l, ma_to_m, lock
    with lock:
        q_dot = copy.copy(msg.joint_velocities)
        Jc_l = ma_to_m.multiArrayToMatrixList(msg.contact_jacobians)
        # print "q_dot :\n", q_dot
        # print "Jc_l :\n", Jc_l
        for J in Jc_l:
            print J*np.matrix(q_dot).reshape(3,1)


def skinCallback(msg):
    global ind, R, P, prev_x, prev_x_dot, prev_F, kp_list, kp, diff_list, diff_list_est, time_list, lock

    with lock:
        ind_new = np.where(np.array(msg.values_x) != 0.0)   #or np.array(msg.values_y) != 0.0 or np.array(msg.values_z) != 0.0)

        if ind == None:
            ind = copy.copy(ind_new)
            prev_F_buf = []
            prev_x_buf = []
            prev_x_dot_buf = []
            for i in ind:
                prev_F_buf.append([msg.values_x[i], msg.values_y[i], msg.values_z[i]])
                prev_x_buf.append([msg.centers_x[i], msg.centers_y[i], msg.centers_z[i]])
                prev_x_dot_buf.append((Jc_l[i]*np.matrix(q_dot).reshape(3,1)).A1.tolist())
            prev_F = np.matrix(prev_F_buf)
            prev_x = np.matrix(prev_x_buf)
            prev_x_dot = np.matrix(prev_x_dot_buf)

            return

        #could do another np.where on these to see where indices coincide, although eventually
        # I want to make it efficient enough to run on all taxels at once (even with zero meas)

        count = 0
        cur_F_buf = []
        cur_x_buf = []
        cur_x_dot_buf = []

        print "ind_new is :", ind_new
        if np.all(ind_new[0] == ind[0]):
            print "in if"
            print "len of ind_new is :", len(ind_new[0])
            for i in xrange(len(ind_new[0])):
                print "in for loop"
                cur_F_buf.append([msg.values_x[ind_new[0][i]], msg.values_y[ind_new[0][i]], msg.values_z[ind_new[0][i]]])
                cur_x_buf.append([msg.centers_x[ind_new[0][i]], msg.centers_y[ind_new[0][i]], msg.centers_z[ind_new[0][i]]])
                #cur_x_dot_buf.append((Jc_l[i]*np.matrix(q_dot).reshape(3,1)).A1.tolist())
                print "OUT OF Try :"
                #print "cur_x_dot_buf :\n", (Jc_l[i]*np.matrix(q_dot).reshape(3,1)).A1.tolist()
                nrml = np.array([msg.normals_x[ind_new[0][i]], msg.normals_y[ind_new[0][i]], msg.normals_z[ind_new[0][i]]])
                #try:
                try:
                    print "in try statement"
                    print "cur_F_buf :\n", cur_F_buf[i]
                    delta_F = np.linalg.norm(np.array(cur_F_buf[i])) - np.linalg.norm(prev_F[i])
                    print "got past delta_f"
                    print "cur_x_buf :\n", np.array(cur_x_buf[i])
                    print "prev x :\n", prev_x[i].A1
                    delta_x = np.dot(nrml,(np.array(cur_x_buf[i]) - prev_x[i].A1))
                    print "got past delta_x"
                    #print "cur_x_dot_buf :\n", np.array(cur_x_dot_buf[i])
                    #print "prev x dot :\n", prev_x_dot[i].A1
                    #delta_x_dot = np.dot(nrml, (np.array(cur_x_dot_buf[i]) - prev_x_dot[i].A1))

                    #R = 1./abs(50*delta_F) + 1./abs(500*delta_x)
                    print "R is :", R

                    print "delta_F :", delta_F
                    print "delta_x :", delta_x
                    #print "delta_x_dot:", delta_x_dot
                    H = np.matrix([delta_x])#, delta_x_dot])

                    print "H is :\n", H
                    print "P is :\n", P
                    print "P*H.T :\n", P*H.T
                    print "H*P*H.T +R :\n", H*P*H.T + R
                    print "np.linalg.inv(H*P*H.T + R) :\n", np.linalg.inv(H*P*H.T + R)

                    Kg = P*H.T*np.linalg.inv(H*P*H.T + R)  #this needs to be changeds so it is not a matrix inverse, but solution

                    print "Kg is :", Kg

                    print "single est is :", delta_F/H


                    kp_list.append(kp[0,0])
                    time_list.append(rospy.get_time())

                    kp = kp + Kg*(delta_F - H*kp)
                    print "diff is :", (delta_F - H*actual_kp)
                    diff_list.append((delta_F - H*actual_kp)[0,0])
                    diff_list_est.append((delta_F - H*kp)[0,0])
                    print "R from data is :", np.cov(diff_list)
                    print "est of R is :", np.cov(diff_list_est)

                    print "Kg*(delta_F - kp*delta_x) is :", Kg*(delta_F - H*kp)

                    P = (np.eye(1) - Kg*H)*P*(np.eye(1)-Kg*H).T + Kg*R*Kg.T  #this would also change to matrix form
                    #P = (np.eye(2) - Kg*H)*P*(np.eye(2)-Kg*H).T + Kg*R*Kg.T  #this would also change to matrix form
                    print "P is :\n", P

                    print "kp est is :\n", kp
                except:
                    e = sys.exc_info()[0]
                    print e

        prev_F = copy.copy(np.matrix(cur_F_buf))
        prev_x = copy.copy(np.matrix(cur_x_buf))
        #prev_x_dot = copy.copy(np.matrix(cur_x_dot_buf))

        ind = copy.copy(ind_new)


if __name__ == "__main__":
    rospy.init_node('contact_stiffness_estimation')
    rospy.Subscriber('/skin/taxel_array', haptic_msgs.TaxelArray,skinCallback)
    #rospy.Subscriber('/haptic_mpc/robot_state', haptic_msgs.RobotHapticState, stateCallback)

    try:
        while not rospy.is_shutdown():
            rospy.sleep(0.01)

    except KeyboardInterrupt:
        pl.figure()
        pl.rcParams.update({'font.size':22})
        pl.plot(np.array(time_list) - time_list[0], kp_list, 'b', np.array(time_list) - time_list[0], [actual_kp]*len(time_list), 'k--')
        pl.xlabel('time (s)')
        pl.ylabel('stiffness (N/m)')
        #pl.ylim((400., 1100.))
        pl.title('RLS Stiffness Estimation for Compliant Environment')
        pl.legend(('estimated', 'actual'), loc='lower right')
        pl.show()
