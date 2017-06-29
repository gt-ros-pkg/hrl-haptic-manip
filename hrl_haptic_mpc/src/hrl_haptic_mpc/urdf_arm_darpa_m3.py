#   Copyright 2013 Georgia Tech Research Corporation
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
#  http://healthcare-robotics.com/

# Author: Marc Killpack, Advait Jain, Phillip Grice

import numpy as np
import sys
import copy

import rospy
import tf

from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from equilibrium_point_control.hrl_arm import HRLArm
from pykdl_utils.kdl_kinematics import create_kdl_kin
from hrl_msgs.msg import FloatArrayBare
import hrl_lib.viz as hv

QUEUE_SIZE=100


class URDFArm(HRLArm):

    def __init__(self, arm, tf_listener=None, base_link=None, end_link=None):
        if arm not in ['l', 'r']:
            raise Exception, 'Arm should be "l" or "r"'

        if base_link is None:
            rospy.logerr("[URDFArm] Must specify manipulator base link")
            sys.exit(1)
        else:
            self.base_link = base_link

        if end_link is None:
            rospy.logerr("[URDFArm] Must specify manipulator end link")
            sys.exit(1)
        else:
            self.end_link = end_link

        kinematics = create_kdl_kin(self.base_link, self.end_link, description_param="/robot_description")
        HRLArm.__init__(self, kinematics)
        self.joint_names_list = kinematics.get_joint_names()
        self.arm_efforts = None
        self.delta_jep = None

        try:
            self.kp = [rospy.get_param(
                '/' + arm + '_arm_controller/gains/' + nm + '/p') for nm in self.joint_names_list]
        except:
            rospy.logerr("kp is not on param server. Exiting...")
            sys.exit(1)

        try:
            self.kd = [rospy.get_param(
                '/' + arm + '_arm_controller/gains/' + nm + '/d') for nm in self.joint_names_list]
        except:
            rospy.logerr("kd is not on param server. Exiting...")
            sys.exit(1)

        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)

        # Set desired joint angle - either through a delta from the current
        # position, or as an absolute value.
        rospy.Subscriber(
            "haptic_mpc/q_des", FloatArrayBare, self.set_ep_callback)
        rospy.Subscriber(
            "haptic_mpc/delta_q_des", FloatArrayBare, self.set_delta_ep_callback)
        # rospy.Subscriber("/delta_jep_mpc_cvxgen", FloatArrayBare, self.set_ep_callback)

        self.marker_pub = rospy.Publisher(
            '/' + arm + '_arm/viz/markers', Marker, queue_size=QUEUE_SIZE)
        self.cep_marker_id = 1

        try:
            if tf_listener is None:
                self.tf_lstnr = tf.TransformListener()
            else:
                self.tf_lstnr = tf_listener
        except rospy.ServiceException:
            rospy.loginfo(
                "ServiceException caught while instantiating a TF listener. This seems to be normal.")
            pass

        self.joint_angles_pub = rospy.Publisher('/' + arm + '_arm_controller/command',
                                                JointTrajectory, queue_size=QUEUE_SIZE)
    ##
    # Callback for /joint_states topic. Updates current joint
    # angles and efforts for the arms constantly
    # @param data JointState message recieved from the /joint_states topic

    def joint_states_cb(self, data):
        arm_angles = []
        arm_efforts = []
        arm_vel = []
        jt_idx_list = [0] * len(self.joint_names_list)
        for i, jt_nm in enumerate(self.joint_names_list):
            jt_idx_list[i] = data.name.index(jt_nm)

        for i, idx in enumerate(jt_idx_list):
            if data.name[idx] != self.joint_names_list[i]:
                raise RuntimeError('joint angle name does not match.')
            arm_angles.append(data.position[idx])
            arm_efforts.append(data.effort[idx])
            arm_vel.append(data.velocity[idx])

        with self.lock:
            self.q = arm_angles
            self.arm_efforts = arm_efforts
            self.qdot = arm_vel

    def set_ep(self, jep, duration=0.15):
        jep = copy.copy(jep)
        if (jep is None) or (len(jep) != len(self.joint_names_list)):
            raise RuntimeError("set_jep value is " + str(jep))
        with self.lock:
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names_list
            jtp = JointTrajectoryPoint()
            jtp.positions = jep
            jtp.time_from_start = rospy.Duration(duration)
            trajectory.points.append(jtp)
            self.joint_angles_pub.publish(trajectory)
            self.ep = jep

    def set_delta_ep_callback(self, msg):
        delta_jep = msg.data
        if self.ep is None:
            self.ep = self.get_joint_angles()
        des_jep = (np.array(self.ep) + np.array(delta_jep)).tolist()
        self.set_ep(des_jep)

    def set_ep_callback(self, msg):
        des_jep = msg.data
        self.set_ep(des_jep)

    def wrap_angles(self, q):
        for ind in [4, 6]:
            while q[ind] < -np.pi:
                q[ind] += 2 * np.pi
            while q[ind] > np.pi:
                q[ind] -= 2 * np.pi
        return q

    def publish_rviz_markers(self):
        # publish the CEP marker.
        o = np.matrix([0., 0., 0., 1.]).T
        jep = self.get_ep()
        cep, r = self.kinematics.FK(jep)
        cep_marker = hv.single_marker(cep, o, 'sphere',
                                      self.base_link, color=(0., 0., 1., 1.),
                                      scale=(0.02, 0.02, 0.02), duration=0.,
                                      m_id=1)
        cep_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(cep_marker)

        q = self.get_joint_angles()
        ee, r = self.kinematics.FK(q)
        ee_marker = hv.single_marker(ee, o, 'sphere',
                                     self.base_link, color=(0., 1., 0., 1.),
                                     scale=(0.02, 0.02, 0.02), duration=0.,
                                     m_id=2)
        ee_marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(ee_marker)


# if __name__ == '__main__':
# rospy.init_node('pr2_arms_test')
#     robot = URDFArm('l')

# if False:
#         jep = [0.] * len(robot.joint_names_list)
#         jep = np.radians([-30, 0, -90, -60, 0, 0, 0])
#         rospy.loginfo('Going to home location.')
#         raw_input('Hit ENTER to go')
#         robot.set_ep(jep, duration=2.)

# if True:
# simple go_jep example
# roslib.load_manifest('equilibrium_point_control')
#         import equilibrium_point_control.epc as epc
#         epcon = epc.EPC(robot)

# while robot.get_joint_angles() is None:
# rospy.sleep(0.1)

#         q = robot.get_joint_angles()
# robot.set_ep(q)

#         jep = [0.] * len(robot.joint_names_list)
# jep = np.radians([30, 0, 90, -60, -180, -30, 0])  # for left arm
# jep = np.radians([-30, 0, -90, -60, 0, 0, 0]) # for right arm
#         epcon.go_jep(jep, speed=np.radians(10.))

# if True:
# while robot.get_joint_angles() is None:
# rospy.sleep(0.1)

#         q = robot.get_joint_angles()
#         ee, r = robot.kinematics.FK(q)
# print "ee is at :\n", ee
# print "r is :\n", r
