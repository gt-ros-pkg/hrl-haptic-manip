#!/usr/bin/env python
# author: marc killpack
import roslib
roslib.load_manifest('darpa_m3')
import rospy
from hrl_lib.transforms import *
import hrl_lib.kdl_utils as ku
import draw_scene as ds
import math
import numpy as np
import time
import ode
import cPickle
import robot_sim_maker as rsm
import contact_controllers as controllers
import sys
import PyKDL as kdl
from darpa_m3.msg import SkinContact
from hrl_lib.msg import FloatArrayBare
from geometry_msgs.msg import Point, Vector3
exec 'import '+sys.argv[1]+' as config'
from hrl_lib.msg import FloatArrayBare
from roslib.msg import Clock
import threading

class RobotSimulatorKDL():
    def __init__(self):
        self.linkage_offset_from_ground = config.linkage_offset_from_ground #comes from config file
        self.right_chain = self.create_right_chain()
        fk, ik_v, ik_p, jac = self.create_solvers(self.right_chain)
        self.right_fk = fk
        self.right_ik_v = ik_v
        self.right_ik_p = ik_p
        self.right_jac = jac
        self.right_tooltip = np.matrix([0.,0.,0.]).T
        self.num_joints = len(config.b_jt_q_ind)

    def create_right_chain(self):
        ch = kdl.Chain()
        prev_vec = np.copy(self.linkage_offset_from_ground.A1)
        n = len(config.b_jt_q_ind)
        b_jt_axis = config.b_jt_axis
        for i in xrange(n-1):
            if b_jt_axis[i][0] == 1 and b_jt_axis[i][1] == 0 and b_jt_axis[i][2] == 0:
                kdl_jt = kdl.Joint(kdl.Joint.RotX)
            elif b_jt_axis[i][0] == 0 and b_jt_axis[i][1] == 1 and b_jt_axis[i][2] == 0:
                kdl_jt = kdl.Joint(kdl.Joint.RotY)
            elif b_jt_axis[i][0] == 0 and b_jt_axis[i][1] == 0 and b_jt_axis[i][2] == 1:
                kdl_jt = kdl.Joint(kdl.Joint.RotZ)
            else:
                print "can't do off-axis joints yet!!!"
            np_vec = np.array(config.b_jt_anchor[i+1])
            diff_vec = np_vec-prev_vec
            prev_vec = np_vec
            kdl_vec = kdl.Vector(diff_vec[0], diff_vec[1], diff_vec[2])            
            ch.addSegment(kdl.Segment(kdl_jt, kdl.Frame(kdl_vec)))
        np_vec = np.copy(config.ee_location.A1)
        diff_vec = np_vec-prev_vec
        if b_jt_axis[n-1][0] == 1 and b_jt_axis[n-1][1] == 0 and b_jt_axis[n-1][2] == 0:
            kdl_jt = kdl.Joint(kdl.Joint.RotX)
        elif b_jt_axis[n-1][0] == 0 and b_jt_axis[n-1][1] == 1 and b_jt_axis[n-1][2] == 0:
            kdl_jt = kdl.Joint(kdl.Joint.RotY)
        elif b_jt_axis[n-1][0] == 0 and b_jt_axis[n-1][1] == 0 and b_jt_axis[n-1][2] == 1:
            kdl_jt = kdl.Joint(kdl.Joint.RotZ)
        else:
            print "can't do off-axis joints yet!!!"
        kdl_vec = kdl.Vector(diff_vec[0], diff_vec[1], diff_vec[2])            
        ch.addSegment(kdl.Segment(kdl_jt, kdl.Frame(kdl_vec)))
        
        return ch

    def create_solvers(self, ch):
         fk = kdl.ChainFkSolverPos_recursive(ch)
         ik_v = kdl.ChainIkSolverVel_pinv(ch)
         ik_p = kdl.ChainIkSolverPos_NR(ch, fk, ik_v)
         jac = kdl.ChainJntToJacSolver(ch)
         return fk, ik_v, ik_p, jac

    ## define tooltip as a 3x1 np matrix in the wrist coord frame.                               
    def set_tooltip(self, arm, p):
        if arm == 0:
            self.right_tooltip = p
        else:
            rospy.logerr('Arm %d is not supported.'%(arm))

    def FK_kdl(self, arm, q, link_number):
        if arm == 0:
            fk = self.right_fk
            endeffec_frame = kdl.Frame()
            kinematics_status = fk.JntToCart(q, endeffec_frame,
                                             link_number)
            if kinematics_status >= 0:
                return endeffec_frame
            else:
                rospy.loginfo('Could not compute forward kinematics.')
                return None
        else:
            msg = '%s arm not supported.'%(arm)
            rospy.logerr(msg)
            raise RuntimeError(msg)

    ## returns point in torso lift link.
    def FK_all(self, arm, q, link_number = len(config.b_jt_q_ind)):
        q = self.list_to_kdl(q)
        frame = self.FK_kdl(arm, q, link_number)
        pos = frame.p
        pos = ku.kdl_vec_to_np(pos)
        pos = pos + self.linkage_offset_from_ground
        m = frame.M
        rot = ku.kdl_rot_to_np(m)
        if arm == 0:
            tooltip_baseframe = rot * self.right_tooltip
            pos += tooltip_baseframe
        else:
            rospy.logerr('Arm %d is not supported.'%(arm))
            return None
        return pos, rot

    def kdl_to_list(self, q):
        if q == None:
            return None
        n = len(config.b_jt_q_ind)
        q_list = [0] * n
        for i in xrange(n):
            q_list[i] = q[i]
        return q_list

    def list_to_kdl(self, q):
        if q == None:
            return None
        n = len(q)
        q_kdl = kdl.JntArray(n)
        for i in xrange(n):
            q_kdl[i] = q[i]
        return q_kdl


    ## compute Jacobian at point pos. 
    # p is in the ground coord frame.
    def Jacobian(self, arm, q, pos):
        if arm != 0:
            rospy.logerr('Arm %d is not supported.'%(arm))
            return None

        tooltip = self.right_tooltip
        self.right_tooltip = np.matrix([0.,0.,0.]).T
        v_list = []
        w_list = []
        for i in xrange(self.num_joints):
            p, rot = self.FK_all(arm, q, i)
            r = pos - p
            z_idx = self.right_chain.getSegment(i).getJoint().getType() - 1
            z = rot[:, z_idx]
            v_list.append(np.matrix(np.cross(z.A1, r.A1)).T)
            w_list.append(z)
        J = np.row_stack((np.column_stack(v_list), np.column_stack(w_list)))
        self.right_tooltip = tooltip
        return J

class Skin_Simulator():
    def __init__(self):
        self.skin_pub = rospy.Publisher('/skin/contacts', SkinContact)

    def skin_callback(self, con_force, con_pt, name, sc):
        for force in con_force:
            sc.forces.append(Vector3(-force[0,0], -force[1,0], -force[2,0]))
        for n in name:
            sc.link_names.append(n)
        for pt in con_pt:
            sc.locations.append(Point(pt[0,0], pt[1,0], pt[2,0]))
            sc.pts_x.append(FloatArrayBare([pt[0,0]]))
            sc.pts_y.append(FloatArrayBare([pt[1,0]]))
            sc.pts_z.append(FloatArrayBare([pt[2,0]]))
        
    def publish(self, sc):
        self.skin_pub.publish(sc)


class RobotSimulator:
    def __init__(self):
        rospy.init_node('sim_multi_contact')
        rospy.Subscriber("/sim_arm/command/jep", FloatArrayBare, self.callback_jep)
        self.pub_clock = rospy.Publisher('/clock', Clock)
        self.pub_angles = rospy.Publisher('/sim_arm/joint_angles', FloatArrayBare)
        self.RoboSimKDL = RobotSimulatorKDL()
        self.lock = threading.RLock()
        self.bodies = []
        self.b_geoms = []
        self.b_joints = []
        self.obstacles = []
        self.o_geoms = []
        self.o_joints = []

        self.control = controllers.ContactControllers(self.RoboSimKDL.num_joints)

        self.wrench_jt1 = []
        self.wrench_jt_o = []
        self.contact_jt = []
        self.times = []
        self.contact_force = []
        self.dist_old = np.matrix(np.zeros((3,1)))
        
        self.q_eq = np.matrix(np.zeros(self.RoboSimKDL.num_joints))
        self.h_g = config.h_goal
        self.in_contact = False
        self.contact_from_free = False
        self.cost = 0.0
        self.contact_pt = []

        self.draw = ds.SceneDraw()
        self.world = ode.World()
        self.world.setGravity((0,0,-9.81))
        self.space = ode.Space()
        self.sim_make = rsm.RobSimMaker(self.world, self.space)
        self.delta = np.matrix(np.zeros(self.RoboSimKDL.num_joints))
        self.step = 8
        self.ind = None
        self.ind_f = (1)
        self.mu = config.mu.A1
        self.skin_sim = Skin_Simulator()
        self.link_name = []


    def near_callback(self, args, geom1, geom2):
        """callback function for the collide() method.
        Function checks if given geoms collide and then
        creates contact joints if so"""
        contacts = ode.collide(geom1, geom2)
        contact_keep = contacts
        world, contactgroup = args
        b1 = geom1.getBody()
        b2 = geom2.getBody()
        if b1 == None:
            b1 = ode.environment
        if b2 == None:
            b2 = ode.environment
        if (ode.areConnected(b1, b2)):
            return
        # if self.sc == None:
        #     self.sc = SkinContact()

        for c in contacts:
            obs_name = None
            link_name = None
            
            params = c.getContactGeomParams()
            if geom1.name == 'link': 
                obs_name = b2.name
                link_name = b1.name
            elif geom2.name == 'link':
                obs_name = b1.name
                link_name = b2.name
            elif geom1.name == 'obstacle' and geom2.name == 'obstacle':
                obs_name = 'obstacle'
            if obs_name == None:
                # print "collision was not with link"
                continue
            elif obs_name == 'obstacle':
                c.setBounce(0.01)
                c.setMu(1)
                j = ode.ContactJoint(world, contactgroup, c)
                j.attach(b1, b2)
                print "name is :", obs_name
            else:
                ind = None
                for i in xrange(len(self.o_joints)):
                    if obs_name == self.obstacles[i].name:
                        ind = i
                if ind == None:
                    print 'link is not in list'
                else:
                    print "link name is :", link_name
                    if self.o_joints[ind].jt_type == 'prismatic':
                        self.o_joints[ind].setAxis(params[1])
                    self.contact_pt.append(np.matrix(params[0]).T)
                    if self.in_contact == False:
                        self.in_contact = True
                    self.contact_force.append((np.matrix(self.o_joints[ind-1].getFeedback()[0]).T)[:,0])
                    c.setBounce(0.01)
                    c.setMu(1)
                    j = ode.ContactJoint(world, contactgroup, c)
                    j.attach(b1, b2)
                    self.link_name.append(link_name)
#                    self.skin_sim.skin_callback(self.contact_force, self.contact_pt, link_name, self.sc)

    def calc_ee_position(self, q):
        pos, rot = self.RoboSimKDL.FK_all(0, q)    
        return pos

    def calc_cost(self, h_g, h_cur, f_max, f_cur, alpha, f_thresh):
        if f_cur > f_thresh and f_cur<f_max:
            f_cur = f_thresh + (1-math.exp(-(f_cur-f_thresh)*0.025))*(f_max-f_thresh)
            return 2*np.linalg.norm(h_cur-h_g)+alpha*(1.0/(1-(f_cur/f_max))-1)
        elif f_cur >= f_max:
            return 2*np.linalg.norm(h_cur-h_g)+alpha*(1.0/(1-0.99)-1)
        else:
            return 2*np.linalg.norm(h_cur-h_g)

    def calc_joint_cost(self, q_eq, q, f_max, f_cur, alpha):
        if f_cur > f_max:
            print 'f_cur is greater than maximum allowed, you are not regulating it so I will'
            f_cur = f_max-f_max/100.0
        return 2*np.linalg.norm(q-q_eq)+alpha*(1.0/(1-(f_cur/f_max))-1)

    def calc_J(self, q, pos):
        J = self.RoboSimKDL.Jacobian(0, q, pos)    
        return J

    def calc_J_contact(self):
        J = np.matrix(np.zeros((6,self.q_eq.size)))

    def apply_friction(self):
        for i in xrange(len(self.o_joints)):
            if self.o_joints[i].jt_type == 'prismatic':
                friction = 7.5*self.mu[i]*1
                local_contact_force=(np.matrix(self.o_joints[i-1].getFeedback()[0]).T)[:,0]
                if np.linalg.norm(local_contact_force[0:2])<friction and abs(self.o_joints[i].getPositionRate())<0.0000000001:
                    friction = -1*np.linalg.norm(local_contact_force)
                    self.o_joints[i].addForce(friction)
                elif abs(self.o_joints[i].getPositionRate())>0.0:
#                    print 'linear vel :', self.o_joints[i].getPositionRate()
                    if friction*self.o_joints[i].getPositionRate()>0 and np.linalg.norm(local_contact_force) == 0:
                        friction = friction/10*-1
                    elif friction*self.o_joints[i].getPositionRate()>0:
                        friction = friction *-1
                    self.o_joints[i].addForce(friction)
                else:
                    self.obstacles[i].setForce((0,0,0))
#                    print 'linear vel of obstacle 1', self.obstacles[i].getLinearVel()
        
        
    def draw_bodies(self):
        counter = 0
        for jt in self.b_joints:
            if not (jt.jt_type == 'fixed' or jt.jt_type == 'prismatic'):
                self.draw.pub_body(jt.getAnchor(), (0, 0, 0,1), jt.size, jt.color, counter, self.draw.Marker.SPHERE)
            counter = counter + 1
        for body in self.bodies:
            self.draw.pub_body(body.getPosition(), matrix_to_quaternion(self.draw.get_rot_mat(body.getRotation()).T), 
                               body.dim, body.color, counter, body.vis)
            counter = counter + 1
        for obst in self.obstacles:
            self.draw.pub_body(obst.getPosition(), matrix_to_quaternion(self.draw.get_rot_mat(obst.getRotation()).T),
                               obst.dim, obst.color, counter, obst.vis)
            counter = counter + 1
    
    def get_joint_angles(self):
        q_list = []
        q_d_list = []
        for jt in self.b_joints:
            if not (jt.jt_type == 'fixed' or jt.jt_type == 'prismatic'):
                q_list.append(jt.getAngle())
                q_d_list.append(jt.getAngleRate())
            if jt.jt_type == 'prismatic':
                print 'prismatic joint is not yet implemented for the robot linkage'
        q_msg = FloatArrayBare()
        q_msg.data = q_list
        self.pub_angles.publish(q_msg)
        q = np.matrix(q_list).T
        q_dot = np.matrix(q_d_list).T
        return q, q_dot

    def callback_jep(self, msg):
        self.lock.acquire()
        self.q_eq = np.matrix(msg.data).T
        self.lock.release()

    def run_config(self):
        bodies = config.bodies
        for i in xrange(len(bodies['shapes'])):
            body = self.sim_make.create_body(bodies['shapes'][i], bodies['dim'][i],
                                             bodies['com_pos'][i], bodies['color'][i],
                                             bodies['mass'][i], bodies['density'][i],
                                             bodies['rotation'][i], bodies['inertia'][i],
                                             bodies['name'][i])
            body.setGravityMode(False)
            self.bodies.append(body)

        obstacles = config.obstacles
        for i in xrange(len(obstacles['shapes'])):
            obst = self.sim_make.create_body(obstacles['shapes'][i], obstacles['dim'][i],
                                             obstacles['com_pos'][i], obstacles['color'][i],
                                             obstacles['mass'][i], obstacles['density'][i],
                                             obstacles['rotation'][i], obstacles['inertia'][i],
                                             obstacles['name'][i])
            obst.setGravityMode(False)
            self.obstacles.append(obst)

        b_geometries = config.b_geometries
        for i in xrange(len(b_geometries['body'])):
            try:
                body = self.bodies[i]
                self.b_geoms.append(self.sim_make.create_geom(body, b_geometries['name'][i],
                                                        b_geometries['dim'][i], b_geometries['shape'][i],
                                                        b_geometries['pos'][i]))
            except:
                body = None
                self.b_geoms.append(self.sim_make.create_geom(body, b_geometries['name'][i],
                                                        b_geometries['dim'][i], b_geometries['shape'][i],
                                                        b_geometries['pos'][i]))
        o_geometries = config.o_geometries
        for i in xrange(len(o_geometries['body'])):
            try:
                body = self.obstacles[i]
                self.o_geoms.append(self.sim_make.create_geom(body, o_geometries['name'][i],
                                                        o_geometries['dim'][i], o_geometries['shape'][i],
                                                        o_geometries['pos'][i]))
            except:
                body = None
                self.o_geoms.append(self.sim_make.create_geom(body, o_geometries['name'][i],
                                                        o_geometries['dim'][i], o_geometries['shape'][i],
                                                        o_geometries['pos'][i]))

        b_jts = config.b_jts
        for i in xrange(len(config.b_jts['jt_type'])):
            if not b_jts['bodies'][i][1] == -1:
                self.b_joints.append(self.sim_make.create_joint((self.bodies[b_jts['bodies'][i][0]],
                                                                 self.bodies[b_jts['bodies'][i][1]]),
                                                                b_jts['jt_type'][i], b_jts['anchor'][i], 
                                                                b_jts['axis'][i], b_jts['vis_size'][i], 
                                                                b_jts['vis_color'][i], b_jts['feedback'][i],
                                                                b_jts['jt_lim'][i]))
            else:
                self.b_joints.append(self.sim_make.create_joint((self.bodies[b_jts['bodies'][i][0]], ode.environment),
                                                                b_jts['jt_type'][i], b_jts['anchor'][i], 
                                                                b_jts['axis'][i], b_jts['vis_size'][i], 
                                                                b_jts['vis_color'][i], b_jts['feedback'][i],
                                                                b_jts['jt_lim'][i]))
        o_jts = config.o_jts
        for i in xrange(len(o_jts['jt_type'])):
            if not o_jts['bodies'][i][1] == -1:
                self.o_joints.append(self.sim_make.create_joint((self.obstacles[o_jts['bodies'][i][0]],
                                                                 self.obstacles[o_jts['bodies'][i][1]]),
                                                                o_jts['jt_type'][i], o_jts['anchor'][i], 
                                                                o_jts['axis'][i], o_jts['vis_size'][i], 
                                                                o_jts['vis_color'][i], o_jts['feedback'][i],
                                                                o_jts['jt_lim'][i]))
            else:
                self.o_joints.append(self.sim_make.create_joint((self.obstacles[o_jts['bodies'][i][0]], ode.environment),
                                                                o_jts['jt_type'][i], o_jts['anchor'][i], 
                                                                o_jts['axis'][i], o_jts['vis_size'][i], 
                                                                o_jts['vis_color'][i], o_jts['feedback'][i],
                                                                o_jts['jt_lim'][i]))

    def run_sim(self):
        contactgroup = ode.JointGroup()


        cur_time = 0
        c = Clock()
        c.clock.nsecs = cur_time
        self.pub_clock.publish(c)
        # Simulation loop...
        dt = 0.0002
        cost=self.cost
        

        h_g = self.h_g  
        self.draw.pub_body((h_g[0],h_g[1],h_g[2]), (0, 0, 0,1), (0.35, 0.35, 0.35), 
                           (0, 204/255.0, 0, 1), 101, self.draw.Marker.CUBE)
        self.draw.pub_body((h_g[0],h_g[1],h_g[2]+0.4), (0, 0, 0,1), (0.25, 0.25, 0.25), 
                           (1, 1, 1, 1), 102, self.draw.Marker.TEXT_VIEW_FACING, 'h_goal')

        while not rospy.is_shutdown():
            c.clock.secs = int(cur_time)
            c.clock.secs = int(math.pow(10,9)*(cur_time-math.floor(cur_time)))
            self.pub_clock.publish(c)
            self.draw.pub_body((h_g[0],h_g[1],h_g[2]), (0, 0, 0,1), (0.35, 0.35, 0.35), 
                               (0, 204/255.0, 0, 1), 101, self.draw.Marker.CUBE)
            self.draw.pub_body((h_g[0],h_g[1],h_g[2]+0.4), (0, 0, 0,1), (0.25, 0.25, 0.25), 
                               (1, 1, 1, 1), 102, self.draw.Marker.TEXT_VIEW_FACING, 'h_goal')

            self.draw_bodies()

            q, q_dot = self.get_joint_angles()
            g = np.matrix(np.zeros(q.shape))

############################should eventually be put into contact_controllers.py#####################

            h_cur = self.calc_ee_position(q)
            diff_h = (h_cur-h_g)[0:2]
            f_max = 2
            f_thresh = 0.5
            J = self.calc_J(q, h_cur)
            gr = 2*(diff_h[0,0])*J[0,:] + 2*(diff_h[1,0])*J[1,:]
            total_force = np.matrix(np.zeros(3)).T
            for force in self.contact_force:
                total_force = total_force + force
            cost = self.calc_cost(h_g, h_cur, f_max, np.linalg.norm(total_force[0:2]), 0.1, f_thresh)
            
            if np.linalg.norm(total_force[0:2])<(f_max-f_thresh):
                jep_mag = np.linalg.norm(diff_h)*math.radians(self.step)
                delta_q = -gr / np.linalg.norm(gr)*jep_mag
                self.q_eq = q+delta_q.T
                self.delta = delta_q.T
                torque = self.control.torque_control(q, self.q_eq, q_dot, g)
                print 'not above threshold'

            elif np.linalg.norm(total_force[0:2])>f_max-f_thresh:
                self.q_eq = q
                if self.cost<cost:
                    self.q_eq = self.q_eq-self.delta
                else:
                    self.delta = np.matrix(np.random.uniform(-0.2, 0.2,(self.RoboSimKDL.num_joints,1)))
                    self.q_eq = self.q_eq+self.delta
                torque = self.control.torque_control(q, self.q_eq, q_dot, g)


#########################this wil be the final usage when Advait is publishing#################
            # self.lock.acquire()
            # torque = self.control.torque_control(q, self.q_eq, q_dot, g)                
            # self.lock.release()
###########################################################################################


#             elif np.linalg.norm(self.contact_force[0:2])>f_max-f_thresh:
#                 J = self.calc_J(q, h_cur)
#                 pull_dir = np.cross(self.contact_force.A1, np.array([0, 0, -1]))  #this is a hack right now, how would I decide the direction?
#                 print "pull_dir direction ", pull_dir
#                 if np.dot(h_cur.A1, self.contact_pt)>0:
#                     scalar = 60
#                 else:
#                     scalar = -60
#                 pull_vector = scalar*np.matrix([pull_dir[0], pull_dir[1], pull_dir[2], 0, 0, 0]).reshape(6,1)

# #                pull_vector = 30*np.matrix([-h_cur[0,0], -h_cur[1,0], h_cur[2,0], 0, 0, 0]).reshape(6,1)
# #                pull_vector = 10*np.matrix([-self.contact_force[0,0], -self.contact_force[1,0], 0, 0, 0, 0]).reshape(6,1)
#                 torque = J.T*pull_vector
#                 print 'pull vector: ', pull_vector

            # J = self.calc_J(q, h_cur)
            # pull_vector = 500*np.matrix([0, -1, 0, 0, 0, 0]).reshape(6,1)
            # torque = J.T*pull_vector

                
            print 'contact force is :', self.contact_force#np.linalg.norm(total_force[0:2])
            if self.contact_force != []:
                x = self.contact_force[0][0]
                y = self.contact_force[0][1]
                print 'angle :', math.degrees(math.atan2(y,x))
            self.dist_old = diff_h
            self.cost = cost

            self.apply_friction()######This is to apply friction to movable objects ############

            for i in xrange(self.RoboSimKDL.num_joints):
                self.b_joints[i].addTorque(torque[i])
            self.in_contact = False 

            n = 3
            self.sc=SkinContact()
            for i in range(n):
                # collision step
                self.contact_force = []
                self.contact_pt = []
                self.link_name = []
                self.space.collide((self.world, contactgroup), self.near_callback)
                # Next simulation step
                self.world.step(dt/n)
                contactgroup.empty()
            
            cur_time = cur_time + dt
            print "current time in seconds :", cur_time
            self.sc.header.stamp = rospy.Time.now()
            self.sc.header.frame_id = 'world_frame'
            self.skin_sim.skin_callback(self.contact_force, self.contact_pt, self.link_name, self.sc)
            self.skin_sim.publish(self.sc)
            del self.sc
######################NEEEED TO VERIFY DIRECTION OF CYLINDERS INERTIA, WHAT HAPPENS WHEN ROTATE?###############

#start_wrench = jt_o.getFeedback ()


# wrenches = {'jt1': wrench_jt1, 'jt_o': wrench_jt_o, 'time': times}

# file_output = open('high_vel_wrenches.pkl', 'wb')

# cPickle.dump(wrenches, file_output)

# file_output.close()
# print "closed pkl file"

if __name__ == "__main__":
    robo_sim = RobotSimulator()
    robo_sim.run_config()
    robo_sim.run_sim()
