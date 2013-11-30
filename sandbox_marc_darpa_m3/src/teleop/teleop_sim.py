#!/usr/bin/env python  


########################THIS is the first thing to fix on Monday morning#################
# self.center_in_torso_frame = center_in_torso_frame  - this can be a static transform in a launch file
# self.scaling_in_base_frame = scaling_in_base_frame - this will need to be found for both arms (cody and sim)
########################################################################################
import sys, os
import roslib
roslib.load_manifest('phantom_omni')
roslib.load_manifest('sandbox_marc_darpa_m3')

from visualization_msgs.msg import Marker
import rospy
import tf
import math
import tf.transformations as tr
import numpy as np
import hrl_lib.tf_utils as tfu
import hrl_lib.transforms as hrl_tr
import coefficients as coeff
#import threading
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Wrench
from phantom_omni.msg import OmniFeedback
from geometry_msgs.msg import Twist
from phantom_omni.msg import PhantomButtonEvent
from collections import deque
import hrl_lib.viz as hv
import copy

class ForceFeedbackFilter:

    def __init__(self, robot, wrench_topic, dest_frame, force_feedback_topic, tflistener):
        self.robot = robot
        self.dest_frame = dest_frame
        self.tflistener = tflistener
        self.omni_fb = rospy.Publisher(force_feedback_topic, OmniFeedback)
        rospy.Subscriber(wrench_topic, PointStamped, self.wrench_callback)
        self.enable = False
        self.IIR_num = coeff.num
        self.IIR_den = coeff.den
        self.input_his = np.matrix(np.zeros((3,coeff.num.size)))
        self.output_his = np.matrix(np.zeros((3,coeff.den.size)))
        self.x_err_his = np.matrix(np.zeros((3,2)))
        # self.prev_time = rospy.Time.now().nsecs*1e-9
        # self.prev_dt = 0.0
        self.omni_max_limit = np.array([7., 7., 7.])
        self.omni_min_limit = np.array([-7., -7., -7.])
        self.kp = 20 #rospy.get_param(kp_name)
        self.kd = 0.01 #rospy.get_param(kd_name)
        self.force_scaling = 1.0
        self.force_old = np.zeros(3)
        self.err_pub = rospy.Publisher('viz_error', PointStamped)
        self.x_filt = np.matrix(np.zeros((3,1)))

    def set_force_scaling(self, scalar):
        self.force_scaling = scalar

    def set_enable(self, v):
        self.enable = v

    def wrench_callback(self, state):
        #calculating force estimate from position error (does not compensate for motion error due to dynamics)
        q = self.robot.get_joint_angles()
        pos,_ = self.robot.kinematics.FK(q)
        x_err = np.matrix([state.point.x-pos[0,0], state.point.y-pos[1,0], state.point.z-pos[2,0]]).T
                                  
        #########this would need to be more seriously filtered and designed to get good damping locally
        # x_dot_err = (3*x_err - 4*self.x_err_his[:,0] + self.x_err_his[:,1])/(1/100.0)
        # self.x_err_his = np.matrix(np.hstack((np.array(x_err).reshape((3,1)), np.array(self.x_err_his[:,0]))))

        ##########This was for debugging purposes
        # ps = PointStamped()
        # ps.header.frame_id = '/torso_lift_link'
        # ps.header.stamp = rospy.get_rostime()
        # ps.point.x = x_dot_err[0,0]
        # ps.point.y = x_dot_err[0,0]
        # ps.point.z = 0

        #self.err_pub.publish(ps)

        feedback = -1.0*self.kp*x_err  # - self.kd*x_dot_err  This term is now included in real-time omni driver

        #5th order IIR Butterworth filter designed in matlab to smooth force estimate
        #this failed due to oscillation induced by filter, could introduce a lower order filter later

        #find and use the rotation matrix from wrench to torso                                                                   
        df_R_ee = tfu.rotate(self.dest_frame, 'torso_lift_link', self.tflistener) 

        wr_df = self.force_scaling*np.array(tr.translation_from_matrix(df_R_ee * tfu.translation_matrix([feedback[0,0], feedback[1,0], feedback[2,0]])))

        #limiting the max and min force feedback sent to omni                                                                    
        wr_df = np.where(wr_df>self.omni_max_limit, self.omni_max_limit, wr_df)
        wr_df = np.where(wr_df<self.omni_min_limit, self.omni_min_limit, wr_df)

        wr = OmniFeedback()
        wr.force.x = wr_df[0]
        wr.force.y = wr_df[1]
        wr.force.z = wr_df[2]
        if self.enable == True:
            self.omni_fb.publish(wr)

# #this could be used for trying to damp the force feedback, didn't work very well
# #         self.force_old[0] = wr.force.x
# #         self.force_old[1] = wr.force.y
# #         self.force_old[2] = wr.force.z
# #         dt = rospy.Time.now().nsecs*1e-9-self.prev_time
# #         self.prev_time = rospy.Time.now().nsecs*1e-9
# #         print "time step: ", dt


def limit_range(numb, lower, upper):
    if lower > upper:
        raise RuntimeError('lower bound > upper bound, wa?')
    return max(min(numb, upper), lower)

class ControlTactileArm:
    def __init__(self, robot,     
            center_in_torso_frame, #=[1,.3,-1], 
            scaling_in_base_frame, #=[3.5, 3., 5.],
            omni_name, #='omni1', 
            set_goal_pose_topic, # = 'l_cart/command_pose',
            tfbroadcast=None, #=None,
            tflistener=None): #=None):

        #threading.Thread.__init__(self)
        self.enabled = False
        self.zero_out_forces = True

        self.robot = robot
        #self.X_BOUNDS = [.3, 1.5] #Bound translation of PR2 arms in the X direction (in torso link frame)
        self.kPos = 15.
        self.kVel = 0.5
        self.kPos_close = 70.
        self.omni_name = omni_name
        self.center_in_torso_frame = center_in_torso_frame
        self.scaling_in_base_frame = scaling_in_base_frame
        self.tflistener = tflistener
        self.tfbroadcast = tfbroadcast
        self.prev_dt = 0.0
        q = self.robot.get_joint_angles()
        self.tip_tt, self.tip_tq = self.robot.kinematics.FK(q, self.robot.kinematics.n_jts)

        # self.tip_tt = np.zeros((3,1))
        # self.tip_tq = np.zeros((4,1))
        self.teleop_marker_pub = rospy.Publisher('/teleop/viz/goal', Marker)

        rate = rospy.Rate(100.0)

        self.omni_fb = rospy.Publisher(self.omni_name + '_force_feedback', OmniFeedback)
        self.set_goal_pub = rospy.Publisher(set_goal_pose_topic, PointStamped)

        rospy.loginfo('Attempting to calculate scaling from omni to arm workspace')
        success = False
        while not success and (not rospy.is_shutdown()):
            rate.sleep()
            try:
                m = tfu.translation_matrix(self.scaling_in_base_frame)
                vec_l0 = tr.translation_from_matrix(tfu.rotate(self.omni_name + '_center', '/torso_lift_link',  self.tflistener)*m)
                self.scale_omni_l0 = np.abs(vec_l0)  
                success = True
            except tf.LookupException, e:
                pass
            except tf.ConnectivityException, e:
                pass
        rospy.loginfo('Finished linking frame for %s' % omni_name)

        rospy.Subscriber(self.omni_name + '_pose', PoseStamped, self.omni_pose_cb)
        #self.gripper_handler = GripperOmniHandler(self.omni_name + '_button', gripper_control_topic)

    def publish_rviz_markers(self):
        # publish the CEP marker.
        tip_tt = np.matrix(self.tip_tt).reshape(3,1)
        o = np.matrix([0.,0.,0.,1.]).T
        teleop_marker = hv.single_marker(tip_tt, o, 'sphere',
                        '/torso_lift_link', color=(0.1, 1., .1, 1.),
                        scale = (0.03, 0.03, 0.03), duration=0.)

        teleop_marker.header.stamp = rospy.Time.now()
        self.teleop_marker_pub.publish(teleop_marker)


    def torso_lift_link_T_omni(self, tip_omni, msg_frame):
        center_T_6 = tfu.transform(self.omni_name+'_center', msg_frame, self.tflistener)
        tip_center = center_T_6*tip_omni
        tip_center_t = (np.array(tr.translation_from_matrix(tip_center))*np.array(self.scale_omni_l0)).tolist()
        tip_center_q = tr.quaternion_from_matrix(tip_center)

        # z_T_6 = tfu.transform(self.omni_name+'_link0', msg_frame, self.tflistener)
        # tip_0 = z_T_6*tip_omni
        # tip_0t = (np.array(tr.translation_from_matrix(tip_0))*np.array(self.scale_omni_l0)).tolist()
        # tip_0q = tr.quaternion_from_matrix(tip_0)

    #     #Transform into torso frame so we can bound arm workspace
        tll_T_0 = tfu.transform('/torso_lift_link', self.omni_name + '_center', self.tflistener)
        tip_torso_mat = tll_T_0 * tfu.tf_as_matrix([tip_center_t, tip_center_q])
        tip_tt, tip_tq = tfu.matrix_as_tf(tip_torso_mat)

        #don't need to bound, arm controller should do this
    #     tip_tt[0] = limit_range(tip_tt[0], self.X_BOUNDS[0], self.X_BOUNDS[1])
        self.tip_tt = tip_tt
        self.tip_tq = tip_tq

    ##
    # Transfrom from torso lift link to link omni base link, taking into account scaling
    def omni_T_torso(self, torso_mat):
        l0_mat = tfu.transform(self.omni_name + '_center', '/torso_lift_link', self.tflistener) * torso_mat
        l0_t = (np.array(tr.translation_from_matrix(l0_mat)) / np.array(self.scale_omni_l0)).tolist()
        l0_q = tr.quaternion_from_matrix(l0_mat)
        omni_pt_mat = tfu.transform(self.omni_name, self.omni_name + '_center', self.tflistener) * tfu.tf_as_matrix((l0_t, l0_q))
        return tfu.matrix_as_tf(omni_pt_mat)

    ##
    # Set whether control should be enabled
    def set_control(self, s):
        self.enabled = s
        if self.enabled:
            #self.gripper_handler.set_enabled(True)
            self.zero_out_forces = True
        else:
            #self.gripper_handler.set_enabled(False)
            pass

    ##
    # Callback for pose of omni
    def omni_pose_cb(self, msg):
        self.publish_rviz_markers()
        if self.enabled:
            #Get the omni's tip pose in the PR2's torso frame
            tip_omni, msg_frame = tfu.posestamped_as_matrix(msg)
            self.torso_lift_link_T_omni(tip_omni, msg_frame)
            #self.torso_T_omni(tip_omni, msg_frame)

            #Publish new arm pose
            ps = PointStamped()
            ps.header.frame_id = '/torso_lift_link'
            ps.header.stamp = rospy.get_rostime()
            ps.point.x = self.tip_tt[0]
            ps.point.y = self.tip_tt[1]
            ps.point.z = self.tip_tt[2]

            self.set_goal_pub.publish(ps)

            if self.zero_out_forces:
                wr = OmniFeedback()
                wr.force.x = 0 
                wr.force.y = 0 
                wr.force.z = 0 
                self.omni_fb.publish(wr)
                self.zero_out_forces = False
        else:
            #this is a zero order hold publishing the last received values until the control loop is active again
            tip_tt = self.tip_tt
            #tip_tq = self.tip_tq
            ps = PointStamped()
            ps.header.frame_id = '/torso_lift_link'
            ps.header.stamp = rospy.get_rostime()
            ps.point.x = tip_tt[0]
            ps.point.y = tip_tt[1]
            ps.point.z = tip_tt[2]

            self.set_goal_pub.publish(ps)

            #this is to make the omni force well move if the arm has moved but the commanded
            #position of the arm has not changed
            tip_omni, msg_frame = tfu.posestamped_as_matrix(msg)
            m_o1 = tfu.transform(self.omni_name, msg_frame, self.tflistener) * tip_omni
            ee_point = np.matrix(tr.translation_from_matrix(m_o1)).T
            q = self.robot.get_joint_angles()
            pos, rot = self.robot.kinematics.FK(q, self.robot.kinematics.n_jts)
            # tip_torso = tfu.transform('/torso_lift_link', self.gripper_tip_frame, self.tflistener) \
            #                       * tfu.tf_as_matrix(([0.,0.,0.], tr.quaternion_from_euler(0,0,0)))

            tip_torso = hrl_tr.composeHomogeneousTransform(rot, pos)
            #tip_torso = tfu.tf_as_matrix(([pos, rot]))
            #def composeHomogeneousTransform(rot, disp):

            center_t, center_q = self.omni_T_torso(tip_torso)
            center_col_vec = np.matrix(center_t).T

            #Send force control info
            wr = OmniFeedback()
            # offsets (0, -.268, -.15) introduced by Hai in phantom driver
            # should be cleaned up at some point so that it is consistent with position returned by phantom -marc
            lock_pos = tr.translation_matrix(np.matrix([0,-.268,-.150]))*tfu.transform(self.omni_name+'_sensable', self.omni_name, self.tflistener)*np.row_stack((center_col_vec, np.matrix([1.])))

            wr.position.x = (lock_pos[0,0])*1000.0  #multiply by 1000 mm/m to get units phantom expects
            wr.position.y = (lock_pos[1,0])*1000.0 
            wr.position.z = (lock_pos[2,0])*1000.0 

            # wr.position.x = tip_tt[0]  #multiply by 1000 mm/m to get units phantom expects
            # wr.position.y = tip_tt[1]
            # wr.position.z = tip_tt[2]
            self.omni_fb.publish(wr)


class OmniTactileTeleop:
    def __init__(self, arm, ff, robot, scaling):
        #rospy.init_node('omni_teleop_multi_contact')
        rospy.loginfo("got into omni_tactile")
        self.enabled = False
        self.tfbroadcast = tf.TransformBroadcaster()
        self.tflistener = tf.TransformListener()
        rospy.loginfo("got up to waiting ")
        self.tflistener.waitForTransform('/torso_lift_link','omni2_center',rospy.Time(),rospy.Duration(100.))
        rospy.loginfo("got past waiting")

        self.controller_list = []
        self.ff_list = []
        cur_pos, _ = robot.kinematics.FK(robot.get_joint_angles(), robot.kinematics.n_jts)
        center_in_torso_frame = (cur_pos.T).tolist()[0]

        rospy.loginfo("Here is the Torso Center: "+str(center_in_torso_frame))
        #print "here is the center: \n", center_in_torso_frame

        if arm == "l":
            self.left_controller = ControlTactileArm(
                                        robot,
                                        center_in_torso_frame, #= [1.2, .3, -1], 
                                        scaling_in_base_frame = scaling,
                                        omni_name ='omni2', 
                                        set_goal_pose_topic = 'test_goal_update',
                                        tfbroadcast=self.tfbroadcast,
                                        tflistener=self.tflistener)
            self.controller_list.append(self.left_controller)
            rospy.Subscriber('omni2_button', PhantomButtonEvent, self.omni_safety_lock_cb)
            if ff == True:
                self.left_feedback = ForceFeedbackFilter(robot, 
                                             wrench_topic = 'test_goal_update',
                                             dest_frame = '/omni2_sensable',
                                             force_feedback_topic = 'omni2_force_feedback',
                                             tflistener = self.tflistener)

                self.ff_list.append(self.left_feedback)
        if arm == "r":
            self.right_controller = ControlTactileArm(
                                        robot,
                                        center_in_torso_frame, #= [1.2, -.3, -1], 
                                        scaling_in_base_frame = scaling,
                                        omni_name ='omni2', 
                                        set_goal_pose_topic = 'test_goal_update',
                                        tfbroadcast=self.tfbroadcast,
                                        tflistener=self.tflistener)
            rospy.Subscriber('omni2_button', PhantomButtonEvent, self.omni_safety_lock_cb)
            self.controller_list.append(self.right_controller)
            if ff == True:
                self.right_feedback = ForceFeedbackFilter(robot,
                                                          wrench_topic = 'test_goal_update',
                                                          dest_frame = '/omni2_sensable',
                                                          force_feedback_topic = 'omni2_force_feedback',
                                                          tflistener = self.tflistener)
                self.ff_list.append(self.right_feedback)
        
        self.set_state(False)

    def omni_safety_lock_cb(self, msg):
        if msg.grey_button == 1 and msg.white_button == 1:
            self.set_state(not self.enabled)

    def set_state(self, s):
        self.enabled = s
        if self.enabled:
            rospy.loginfo('control ENABLED.')
            for cont in self.controller_list:
                cont.set_control(True)
            for f in self.ff_list:
                f.set_enable(True)
        else:
            rospy.loginfo('control disabled.  Follow potential well to pose of arm.')
            for cont in self.controller_list:
                cont.set_control(False)
            for f in self.ff_list:
                f.set_enable(False)

    def run(self):
        rate = rospy.Rate(100.0)
        rospy.loginfo('running...')
        while not rospy.is_shutdown():
            rate.sleep()
            # for cont in self.controller_list:
            #     cont.send_transform_to_link_omni_and_pr2_frame()


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    
    p.add_option('--arms', action='store', type='string', dest='arm', default='r', 
                 help='this allows initialization of right, left or both arms with an input of r, l or b respectively')
    p.add_option('--ff', action='store', type='int', dest='ff', default='0',
                 help='enter 1 to activate force feedback, 0 otherwise')
    p.add_option('--cody', action='store_true', dest='cody', default=False,
                 help='use cody kinematics for teleop')
    p.add_option('--sim', action='store_true',  dest='sim', default=False,
                 help='use sim arm kinematics for teleop')
        
    rospy.init_node('multi_contact_teleop')

    opt, args = p.parse_args()

    if opt.cody == True:
        import hrl_cody_arms.cody_arm_client as cac                                                
        robot = cac.CodyArmClient(opt.arm)
        scaling = [3, 3, 3]
    elif opt.sim == True:
        import hrl_software_simulation_darpa_m3.ode_sim_arms as osa
        robot = osa.ODESimArm()
        scaling = [5, 5, 5]

    o = OmniTactileTeleop(opt.arm, opt.ff, robot, scaling)

    o.run()



