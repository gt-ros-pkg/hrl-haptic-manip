#
# Copyright (c) 2013, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \authors: Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
# \adviser: Charles Kemp (Healthcare Robotics Lab, Georgia Tech.)


# import epc_skin as es
# import scipy.optimize as so
# import threading
# import roslib; roslib.load_manifest('darpa_m3')
# import rospy
# import software_simulation.ode_sim_arms as osa
# from darpa_m3.msg import SkinContact
# import copy
# import numpy as np
# import itertools as it


# #######################################
# # this is used to do variable impedance
# from darpa_m3.msg import MechanicalImpedanceParams

# class ode_SkinClient(es.SkinClient):
#     def __init__(self, skin_topic):
#         es.SkinClient.__init__(self, skin_topic)

#     def force_normal_loc_joint_list(self, normal_component_only,
#                                     return_time=False):
#         self.lock.acquire()
#         f_l = copy.copy(self.force_list)
#         n_l = copy.copy(self.normal_list)
#         nm_l = copy.copy(self.link_names)
#         loc_l = copy.copy(self.loc_list)
#         stamp = copy.copy(self.stamp)
#         self.lock.release()

#         jt_l = []
#         for i in range(len(f_l)):
#             f = f_l[i]
#             n = n_l[i]
#             if normal_component_only:
#                 f_l[i] = n * np.linalg.norm(f)

#             nm = nm_l[i]
#             if 'link1' in nm:
#                 jt_num = 0
#             elif 'link2' in nm:
#                 jt_num = 1
#             elif 'link3' in nm:
#                 jt_num = 2
#             jt_l.append(jt_num)

#         if return_time:
#             return f_l, n_l, loc_l, jt_l, stamp
#         else:
#             return f_l, n_l, loc_l, jt_l


# class VarCompOpt():
#     def __init__(self, robot, skin_topic):
#         rospy.init_node('variable_compliance_optimizer')
#         self.robot = robot
#         self.scl = ode_SkinClient(skin_topic)
#         self.var_imped_pub = rospy.Publisher('/sim_arm/command/joint_impedance', MechanicalImpedanceParams)
#         self.var_imped_sub = rospy.Subscriber('/sim_arm/joint_impedance', MechanicalImpedanceParams, self.get_impedance)
#         self.impedance = MechanicalImpedanceParams()
#         self.k_p = None
#         self.k_d = None
#         self.k_p_buf = None
#         self.k_d_buf = None

#         self.lock = threading.RLock()
        
#         while self.k_p_buf == None:
#             rospy.sleep(0.01)
#         self.k_p_nom = list(self.k_p_buf)
#         self.k_d_nom = list(self.k_d_buf)

#     def optimize(self, Jc_l, f_l, k_bounds, f_thresh):
#         k_p,_,_ = so.fmin_tnc(self.opt_func, self.k_p_nom, args=(Jc_l, f_l, k_bounds, f_thresh), approx_grad=True, bounds=k_bounds, messages = 0)
#         return k_p.tolist(), self.k_d_nom

#     def opt_func(self, k_p, *args):
#         Jc_l = args[0]
#         f_l = args[1]
#         k_bounds = args[2]
#         f_thresh = args[3]
#         cost = 0.0
#         K = np.matrix(np.diag(k_p))
#         for i in xrange(len(Jc_l)):
#             J = np.matrix(Jc_l[i])
#             f = f_l[i]
#             f_mag = np.linalg.norm(f)
#             cost = cost - f.T*J*np.linalg.inv(K)*J.T*f
#             # if f_mag > f_thresh:
#             #     cost = cost - f.T*J*np.linalg.inv(K)*J.T*f

#         # cost = cost + 0.0001*((K.diagonal()-np.matrix(self.k_p_nom))*(K.diagonal()-np.matrix(self.k_p_nom)).T)[0,0] #this is because the answer is a matrix still
#         print "cost is : ", cost
#         return cost

#     def get_impedance(self, msg):
#         self.lock.acquire()
#         self.k_p_buf = list(msg.k_p.data)
#         self.k_d_buf = list(msg.k_d.data)
#         self.lock.release()

#     def run_opt(self):

#         #######################need a common place for controller dictionary####################
#         #######################right now this is defined within Charlie's QP controller.########
#         #######################This is referrring to "normal_component_only"####################
#         k_bounds = [(1, 60), (1, 40), (1, 30)]
#         f_thresh = 5.0
#         while not rospy.is_shutdown():
#             f_l, n_l, loc_l, jt_l, time_stamp = self.scl.force_normal_loc_joint_list(
#                 normal_component_only = True, 
#                 return_time = True)            

#             n = len(n_l)

#             # f_mag_list is a list of the magnitudes of the contact
#             # forces
#             f_mag_list  = [np.linalg.norm(f_vec) for f_vec in f_l]
#             # stop if a contact force is too high
#             q = self.robot.get_joint_angles()

#             ####################this needs to defined elsewhere as the number of links##############
#             control_point_joint_num = 3
#             ########################################################################################


#             x_h = self.robot.kinematics.FK(q, control_point_joint_num)[0]

#             J_all = self.robot.kinematics.jacobian(q, x_h)
#             active_jt_idxs = [0, 1, 2] #not sure this right   [jt_index_dict[jt] for jt in active_joints]
#             # J_h = Jacobian for the hand (end effector)
#             J_h = J_all[0:3, active_jt_idxs]
#             J_h[:, control_point_joint_num:] = 0.

#             jep = np.array(self.robot.get_ep())
#             # phi_curr = phi[t] = current equilibrium angles for 
#             # the virtual springs on the robot's arm

#             #phi_curr = np.matrix(jep[active_jt_idxs]).T

#             # compute contact Jacobians
#             Jc_l = []
#             for jt_li, loc_li in it.izip(jt_l, loc_l):
#                 Jc = self.robot.kinematics.jacobian(q, loc_li)
#                 Jc[:, jt_li+1:] = 0.0
#                 Jc = Jc[0:3, active_jt_idxs]
#                 Jc_l.append(Jc)
#             if f_mag_list == []:
#                 print "not changing the stiffnesses, could do something intelligent later"
#                 k_p = self.k_p_nom
#                 self.impedance.k_p.data = k_p
#                 diff=1
#                 while (diff > 0.001):
#                     self.var_imped_pub.publish(self.impedance)
#                     diff = np.linalg.norm(np.array(k_p) - np.array(self.k_p_buf))
#                     rospy.sleep(0.001)
#             elif max(f_mag_list) > 100.:
#                 k_p = [1, 1, 1]
#                 self.impedance.k_p.data = k_p
#                 diff = 1
#                 while (diff > 0.001):
#                     self.var_imped_pub.publish(self.impedance)
#                     diff = np.linalg.norm(np.array(k_p) - np.array(self.k_p_buf))
#                     rospy.sleep(0.001)
#                 self.lock.acquire()
#                 #self.k_p = list(self.k_p_buf)
#                 #self.k_d = list(self.k_d_buf)
#                 self.lock.release()
#             else:
#             ########DO OPTIMIZATION HERE######################
#                 k_p, k_d = self.optimize(Jc_l, f_l, k_bounds, f_thresh)
#                 self.impedance.k_p.data = k_p
#                 self.impedance.k_d.data = k_d
#                 print "here is the k_p and k_d values:"
#                 print k_p
#                 print k_d
#                 diff = 1
#                 while (diff > 0.001):
#                     self.var_imped_pub.publish(self.impedance)
#                     diff = np.linalg.norm(np.array(k_p) - np.array(self.k_p_buf))
#                     rospy.sleep(0.001)
#                 self.lock.acquire()
#                 self.k_p = list(self.k_p_buf)
#                 self.k_d = list(self.k_d_buf)
#                 self.lock.release()

#     def pub_once(self, k_p, k_d):
#         self.impedance.k_p.data = k_p
#         self.impedance.k_d.data = k_d
#         diff = 1
#         while (diff > 0.001):
#             self.var_imped_pub.publish(self.impedance)
#             diff = np.linalg.norm(np.array(k_p) - np.array(self.k_p_buf))
#             rospy.sleep(0.001)


if __name__ == '__main__':
    print 'this file is broken with respect to new controller structure, however it could serve as reference for new start on variable impedance control'

    # import optparse
    # p = optparse.OptionParser()

    # p.add_option('--optimize', action='store_true', dest='optimize',
    #              default=False, help='run continuous optimization to vary compliance')
    # p.add_option('--pub_once', action='store', dest='pub_once',
    #              default=None, help='publish impedance until it takes, make sure you included [[kp1, kp2, kp3 ...],[kd1, kd2, kd3, ...]] as an argument')
    # p.add_option('--pub_once_alpha', action='store', dest='pub_once_alpha',
    #              default=None, help='change compliance parameters by a single gain, make sure you included alpha argument')

    # opt, args = p.parse_args()

    # skin_topic = '/skin/contacts'
    # robot = osa.ODESimArm()
    # variable_compliance = VarCompOpt(robot, skin_topic)

    # if opt.optimize == True:
    #     variable_compliance.run_opt()
    # elif opt.pub_once != None:
    #     gains = eval(opt.pub_once)
    #     variable_compliance.pub_once(gains[0], gains[1])
    # elif opt.pub_once_alpha != None:
    #     alpha = eval(opt.pub_once_alpha)/100.0
    #     new_kp = (alpha*np.array(variable_compliance.k_p_nom)).tolist()
    #     #THIS IS A FIXED NOMINAL DAMPING WHICH DOESN'T MAKE MUCH SENSE
    #     #new_kd = variable_compliance.k_d_nom

    #     #this damping is varying to keep the damping ratio for each joint constant
    #     # k_d_nom * sqrt(k_new/k_nom) for each joint individually 
    #     new_kd = (np.array(variable_compliance.k_d_nom)*np.sqrt(np.array(new_kp)/np.array(variable_compliance.k_p_nom))).tolist()
    #     variable_compliance.pub_once(new_kp, new_kd)
    # else:
    #     print "you didn't specify an option, try the --help command"
