
# Advait adds OpenRAVE to the python path. This is because he flits
# between using the ROS version and using the verison from the
# openrave debian.
import openravepy as orpy

import numpy as np, math
import sys

import roslib
roslib.load_manifest('hrl_hardware_in_loop_simulation_darpa_m3')

import rospy
import tf

import hrl_lib.util as ut
import hrl_lib.transforms as tr


class Online_Collision_Detector():
    def __init__(self, env_file, oid_list, n_radius_steps, display):
        env = orpy.Environment()
        if display:
            env.SetViewer('qtcoin')
        env.Load(env_file)
        robot_openrave = env.GetRobots()[0]
        # funny rotation matrix for the PR2 and Cody. explicitly
        # setting the transform in the beginning.
        robot_openrave.SetTransform(np.diag([0.,0.,0.,1.]))

        self.env = env
        self.oid_list = oid_list
        self.n_radius_steps = n_radius_steps
        self.robot_openrave = robot_openrave
        self.idx_list = [] # see register_obstacles

        self.tf_lstnr = tf.TransformListener()

    # position and orientation of the top most frame of the robot.
    def get_robot_transform(self):
        t1, q1 = self.tf_lstnr.lookupTransform('/world', '/torso_lift_link', rospy.Time(0))
        t1 = np.matrix(t1).reshape(3,1)
        r1 = tr.quaternion_to_matrix(q1)
        return tr.composeHomogeneousTransform(r1, t1)

    def create_openrave_idx_list(self, robot, torso_height=None):
        if torso_height != None:
            idx = self.robot_openrave.GetJoint('torso_lift_joint').GetDOFIndex()
            self.robot_openrave.SetDOFValues([torso_height], [idx])
        joint_nm_list = robot.joint_names_list
        idx_list = [self.robot_openrave.GetJoint(j).GetDOFIndex() for j in joint_nm_list]
        self.idx_list = idx_list

    def init_obstacle_locations(self, dict_nm=None, save_dict=False):
        if dict_nm != None:
            obs_dict = ut.load_pickle(dict_nm)
        else:
            obs_dict = {}

        for oid in self.oid_list:
            if dict_nm == None:
                ut.get_keystroke('put fixed_obstacle marker on '+ oid +' and then hit ANY KEY.')
                trans, quat = self.tf_lstnr.lookupTransform('/world', '/fixed_obstacle_trackable', rospy.Time(0))
                trans = np.matrix(trans).reshape(3,1)
                print 'fixed_obstacle_trackable translation:', trans.A1
            else:
                trans = obs_dict[oid]

            # for save_dict
            obs_dict[oid] = trans

            for i in range(self.n_radius_steps):
                obs = self.env.GetKinBody(oid+'_%d'%i)
                tra = obs.GetTransform()
                tra[0,3] = trans[0,0]
                tra[1,3] = trans[1,0]
                obs.SetTransform(tra)

        if save_dict:
            if dict_nm == None:
                dict_nm = 'fixed_obstacle_dict_'+ut.formatted_time()+'.pkl'
            ut.save_pickle(obs_dict, dict_nm)

        rospy.loginfo('Added all the objects.')
        # setup the collision checker to return contacts
        self.env.GetCollisionChecker().SetCollisionOptions(orpy.CollisionOptions.Contacts)

    def fine_tune_collision(self, link, oid, report):
        report2 = orpy.CollisionReport()
        for i in range(self.n_radius_steps-1, 0, -1):
            obs = self.env.GetKinBody(oid+'_%d'%i)
            collision = self.env.CheckCollision(link, obs, report=report2)
            if len(report2.contacts) > 0:
                report = report2
                return report
        return report

    def create_contact_dict_cody(self, q):
        t = np.array(self.get_robot_transform())
        self.robot_openrave.SetTransform(t)

        self.robot_openrave.SetDOFValues(q, self.idx_list)
        report = orpy.CollisionReport()
        contact_dict = {}

        trans, quat = self.tf_lstnr.lookupTransform('/world', '/torso_lift_link', rospy.Time(0))
        rot = tr.quaternion_to_matrix(quat)
        trans = np.matrix(trans).reshape(3,1)

        for oid in self.oid_list:
            obs = self.env.GetKinBody(oid+'_0')
            for link in self.robot_openrave.GetLinks():

                link_name = link.GetName()
                if 'shoulderupper' in link_name:
                    continue
                if 'shoulderpitch' in link_name:
                    continue
                if 'w_differential' in link_name:
                    continue

                collision = self.env.CheckCollision(link, obs, report=report)
                if len(report.contacts) > 0:
                    report = self.fine_tune_collision(link, oid, report)
                    body1 = link.GetName()
                    
                    #body2 = report.plink2.GetParent().GetName()
                    #print report.plink2.GetParent().GetName()
                    body2 = oid
                    two_bodies = body1+'+'+body2

                    # clubbing end effector and handmount into one
                    # body.
                    if 'handmount' in body1:
                        body1 = 'end_effector'

                    if 'end_effector' in body1:
                        body1 = 'end_effector'
                        # remove previous instance of
                        # 'end_effector+object', if it exists. This
                        # previous instance is from handmount and we
                        # will ignore it.
                        # the correctness of this code relies on
                        # handmount appearing before end_effector in
                        # the list returned by GetLinks()
                        contact_dict.pop(two_bodies, None)

                    positions = [(c.pos[0], c.pos[1], c.pos[2]) for c in report.contacts]
                    normals = [(c.norm[0], c.norm[1], c.norm[2]) for c in report.contacts]

                    # transform to torso_link frame.
                    pos_np = np.matrix(positions).T
                    pos_np = rot.T * (pos_np - trans)
                    positions = pos_np.T.tolist()

                    normals_np = rot.T * np.matrix(normals).T
                    normals = normals_np.T.tolist()

                    contact_dict[two_bodies] = positions #, normals
                    # Jul 5, 2011 - commented the line below because I
                    # want multiple collisions per obstacle.
                    #break # force there to be only one collision per obstacle.

        return contact_dict


if __name__ == '__main__':
    if True:
        ocd = Online_Collision_Detector()
        joint_nm_list = ['r_shoulder_pan_joint',
                'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                'r_elbow_flex_joint', 'r_forearm_roll_joint',
                'r_wrist_flex_joint', 'r_wrist_roll_joint']
        idx_list = [ocd.pr2_openrave.GetJoint(j).GetDOFIndex() for j in joint_nm_list]
        ocd.pr2_openrave.SetDOFValues([0.]*7, idx_list)

        oid_list = ['force_torque_ft1']
        for i, oid in enumerate(oid_list):
            obs = ocd.obstacles[i]
            tra = obs.GetTransform()
            tra[0,3] = -0.65
            tra[1,3] = 0.13
            obs.SetTransform(tra)
        ocd.env.GetCollisionChecker().SetCollisionOptions(orpy.CollisionOptions.Contacts)

        report = orpy.CollisionReport()
        link = ocd.pr2_openrave.GetLink('r_forearm_link')

        pr2 = ocd.env.GetBodies()[0]

        obs = ocd.obstacles[0]
        collision = ocd.env.CheckCollision(link, obs, report=report)
        #collision = ocd.env.CheckCollision(pr2, report=report)
        #collision = ocd.env.CheckCollision(obs, pr2, report=report)
        contacts = report.contacts

        norm_list = [c.norm for c in contacts]
        pos_list = [c.pos for c in contacts]
        d = {'pos': pos_list, 'norm': norm_list}
        ut.save_pickle(d, 'c.pkl')


        raw_input('Hit ENTER to end.')
        orpy.RaveDestroy()

    if False:
        rospy.init_node('contacts_openrave')
        ocd = Online_Collision_Detector()
        ocd.register_obstacles()

        # now loop and display the contacts.
        pr2_arms = pa.PR2Arms()
        r_arm = 0
        while not rospy.is_shutdown():
            q = pr2_arms.get_joint_angles(r_arm)
            ocd.create_contact_dict(q)

        orpy.RaveDestroy()






# NEED to fix the transforms stuff
#    def create_contact_dict_pr2(self, q):
#        trans, quat = self.tf_lstnr.lookupTransform('/odom', '/base_link', rospy.Time(0))
#        rot = tr.quaternion_to_matrix(quat)
#        t = np.array(tr.composeHomogeneousTransform(rot, trans))
#        self.robot_openrave.SetTransform(t)
#
#        self.robot_openrave.SetDOFValues(q, self.idx_list)
#        report = orpy.CollisionReport()
#        contact_dict = {}
#
#        for obs in self.obstacles:
#            forearm_positions = [] # accumulating all the forearm collisions
#            forearm_normals = []
#            forearm_body = None
#
#            gripper_positions = [] # accumulating all the gripper collisions
#            gripper_normals = []
#            gripper_body = None
#
#            for link in self.robot_openrave.GetLinks():
#                collision = self.env.CheckCollision(link, obs, report=report)
#                if len(report.contacts) > 0:
#                    body1 = link.GetName()
#                    body2 = report.plink2.GetParent().GetName()
#                    two_bodies = body1+'+'+body2
#
#                    if 'forearm' in body1 or 'wrist_flex' in body1:
#                        for c in report.contacts:
#                            forearm_positions.append((c.pos[0], c.pos[1], c.pos[2]))
#                            forearm_normals.append((c.norm[0], c.norm[1], c.norm[2]))
#                            forearm_body = body2
#                    elif 'gripper' in body1 or 'wrist' in body1:
#                        for c in report.contacts:
#                            gripper_positions.append((c.pos[0], c.pos[1], c.pos[2]))
#                            gripper_normals.append((c.norm[0], c.norm[1], c.norm[2]))
#                            gripper_body = body2
#                    else:
#                        positions = [(c.pos[0], c.pos[1], c.pos[2]) for c in report.contacts]
#                        normals = [(c.norm[0], c.norm[1], c.norm[2]) for c in report.contacts]
#                        contact_dict[two_bodies] = positions #, normals
#
#            if forearm_positions != []:
#                contact_dict['forearm+'+forearm_body] = forearm_positions #, forearm_normals
#
#            if gripper_positions != []:
#                contact_dict['gripper+'+gripper_body] = gripper_positions #, gripper_normals
#
#        return contact_dict


