import roslib
roslib.load_manifest("sandbox_marc_darpa_m3")
import rospy

import hrl_lib.transforms as tr
import hrl_haptic_manipulation_in_clutter_msgs.msg as haptic_msgs
import geometry_msgs.msg

import numpy as np
import threading, copy
from matplotlib import pyplot as pl


ind = None
R = 0.001  
# did 10 and got 260 with kp = 100. as initial
#did 0.1 and got 470 with kp = 0. as initial 
kp = 100.
P = 100000000
prev_x = None
prev_F = None
kp_list = []


def skinCallback(msg):
    print "in callback at least ..."
    global ind, R, P, prev_x, prev_F, kp_list, kp

    ind_new = np.where(np.array(msg.values_x) != 0.0)   #or np.array(msg.values_y) != 0.0 or np.array(msg.values_z) != 0.0)

    if ind == None:
        ind = copy.copy(ind_new)
        prev_F = np.array([msg.values_x[i], msg.values_y[i], msg.values_z[i]])
        prev_x = np.array([msg.centers_x[i], msg.centers_y[i], msg.centers_z[i]])
        return

    #could do another np.where on these to see where indices coincide, although eventually
    # I want to make it efficient enough to run on all taxels at once (even with zero meas)
    
    count = 0
    for i in ind_new:
        print "i is ", i

        cur_F = np.array([msg.values_x[i], msg.values_y[i], msg.values_z[i]])
        cur_x = np.array([msg.centers_x[i], msg.centers_y[i], msg.centers_z[i]])
        nrml = np.array([msg.normals_x[i], msg.normals_y[i], msg.normals_z[i]])
        try:
            if ind_new[count] == ind[count]:
                delta_F = np.linalg.norm(cur_F - prev_F)
                delta_x = np.dot(nrml,(cur_x - prev_x))

                R = 1./abs(50*delta_F) + 1./abs(500*delta_x)
                print "R is :", R


                print "cur_F is :", cur_F
                print "cur_x is :", cur_x

                print "delta_F :", delta_F
                print "delta_x :", delta_x
                Kg = P*delta_x/(delta_x*P*delta_x + R)  #this would change to be a matrix inversion later

                print "Kg is :", Kg

                kp = kp + Kg*(delta_F - kp*delta_x)
                print "Kg*(delta_F - kp*delta_x) is :", Kg*(delta_F - kp*delta_x)

                P = (1 - Kg*delta_x)*P*(1-Kg*delta_x) + Kg*R*Kg  #this would also change to matrix form
                kp_list.append(kp)
                print "kp est is :", kp
            else:
                print "indices changed, skipping to next sample"
        except:
            print "indices changed skipping now"

        prev_F = copy.copy(cur_F)
        prev_x = copy.copy(cur_x)

    ind = copy.copy(ind_new)


if __name__ == "__main__":
    rospy.init_node('contact_stiffness_estimation')
    rospy.Subscriber('/skin/taxel_array', haptic_msgs.TaxelArray,skinCallback)

    rospy.spin()
