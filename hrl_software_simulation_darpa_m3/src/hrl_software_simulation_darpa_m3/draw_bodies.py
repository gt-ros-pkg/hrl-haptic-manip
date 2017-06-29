#!/usr/bin/python

import numpy as np
import roslib
roslib.load_manifest('hrl_software_simulation_darpa_m3')
import rospy

import hrl_common_code_darpa_m3.visualization.draw_scene as ds

from hrl_haptic_manipulation_in_clutter_msgs.msg import BodyDraw
from hrl_haptic_manipulation_in_clutter_msgs.msg import JtsDraw
from hrl_msgs.msg import FloatArrayBare
from hrl_lib.transforms import *
import hrl_lib.viz as hv
import threading

from visualization_msgs.msg import Marker


class DrawAll:
    def __init__(self):
        self.lock = threading.RLock()
        rospy.init_node('draw_all_bodies')

        # this needs to be the first param that is read for
        # synchronization with obstacles.py
        while not rospy.is_shutdown():
            try:
                self.moveable_dimen = rospy.get_param('m3/software_testbed/movable_dimen')
                break
            except KeyError:
                rospy.sleep(0.1)
                continue
            
        self.fixed_dimen = rospy.get_param('m3/software_testbed/fixed_dimen')
        self.compliant_dimen = rospy.get_param('m3/software_testbed/compliant_dimen')
        self.num_fixed = rospy.get_param('m3/software_testbed/num_fixed')
        self.num_moveable = rospy.get_param('m3/software_testbed/num_movable')
        self.num_compliant = rospy.get_param('m3/software_testbed/num_compliant')
        self.num_tot = rospy.get_param('m3/software_testbed/num_total')

        self.fixed_ctype = rospy.get_param('m3/software_testbed/fixed_ctype')
        self.movable_ctype = rospy.get_param('m3/software_testbed/movable_ctype')
        
        self.goal = np.matrix(rospy.get_param('m3/software_testbed/goal')).T

        try:
            self.movable_stiffness = rospy.get_param('m3/software_testbed/compliant_stiffness')
        except KeyError:
            self.movable_stiffness = [-1]*int(self.num_compliant)

        # all params from the robot_config uploading node (e.g.
        # three_link_planar_param_upload.py) should accessed after
        # this loop.
        while not rospy.is_shutdown():
            try:
                self.num_links = rospy.get_param('/m3/software_testbed/linkage/num_links')
                break
            except KeyError:
                rospy.sleep(0.1)
                continue

        self.links_dim = rospy.get_param('/m3/software_testbed/linkage/dimensions')
        self.links_shape = rospy.get_param('/m3/software_testbed/linkage/shapes')

        self.joints_pos = []
        self.active_jts = None
        self.constr_jts = None
        self.links_pos = []
        self.links_rot = []
        self.obst_pos = []
        self.obst_rot = []

        # Changed frames for a mobile base.
        #self.draw_obstacles = ds.SceneDraw("sim/viz/obstacles", "/torso_lift_link")
        ## self.draw_links = ds.SceneDraw("sim/viz/linkage", "/torso_lift_link")
        self.draw_obstacles = ds.SceneDraw("sim/viz/obstacles", "/world")
        self.draw_links = ds.SceneDraw("sim/viz/linkage", "/world")
        
        self.color_active = [0, 1, 0, 1]
        self.color_limit = [1, 0, 0, 1]
        self.color_passive = [0, 0, 1, 1]
        self.color_links =rospy.get_param('m3/software_testbed/linkage/colors')
        rospy.Subscriber("/sim_arm/bodies_visualization", BodyDraw, self.bodies_callback)

        self.goal_marker_pub = rospy.Publisher('/epc_skin/viz/goal', Marker)

        
    def bodies_callback(self, msg):
        with self.lock:
            self.links_pos = msg.link_loc
            self.links_rot = msg.link_rot
            self.obst_pos = msg.obst_loc
            self.obst_rot = msg.obst_rot

    def draw_bodies(self):
        obst_counter = 0
        link_counter = 0
        color_active = self.color_active
        color_limit = self.color_limit
        color_passive = self.color_passive
        color_links = self.color_links
        jt_size = 0.05

        self.lock.acquire()
        for i in xrange(len(self.links_pos)):
            link_rot_mat = self.draw_links.get_rot_mat(self.links_rot[i].data).T
            if self.links_shape[i] == "cube":
                shape = self.draw_links.Marker.CUBE
                dims = self.links_dim[i]
            elif self.links_shape[i] == "capsule":
                shape = self.draw_links.Marker.CYLINDER
                dims = self.links_dim[i]
                #dims = [self.links_dim[i][0], self.links_dim[i][1], self.links_dim[i][2]]
                dim_jt = [self.links_dim[i][0], self.links_dim[i][0], self.links_dim[i][0]]
                jt_pos_loc = link_rot_mat*np.array([0, 0, self.links_dim[i][2]/2.0]).reshape(3,1)
                jt_pos_plus = np.array(self.links_pos[i].data).reshape(3,1) + jt_pos_loc
                jt_pos_neg = np.array(self.links_pos[i].data).reshape(3,1) - jt_pos_loc                
                self.draw_links.pub_body(jt_pos_plus, matrix_to_quaternion(link_rot_mat), dim_jt, color_links[i], link_counter, self.draw_links.Marker.SPHERE)
                self.draw_links.pub_body(jt_pos_neg, matrix_to_quaternion(link_rot_mat), dim_jt, color_links[i], link_counter+1, self.draw_links.Marker.SPHERE)
                link_counter=link_counter+2

            self.draw_links.pub_body(self.links_pos[i].data, matrix_to_quaternion(self.draw_links.get_rot_mat(self.links_rot[i].data).T),
                               dims, color_links[i], link_counter, shape)
            link_counter = link_counter + 1

        if self.obst_pos != []:
            for i in xrange(self.num_moveable):
                color = [0.6,0.6,0,0.7]
                self.draw_obstacles.pub_body(self.obst_pos[i].data, 
                                   matrix_to_quaternion(self.draw_obstacles.get_rot_mat(self.obst_rot[i].data).T),
                                   [self.moveable_dimen[i][0]*2, self.moveable_dimen[i][1]*2, self.moveable_dimen[i][2]], 
                                   color, 
                                   obst_counter, 
                                   self.draw_obstacles.Marker.CYLINDER)
                obst_counter = obst_counter + 1

            for i in xrange(self.num_compliant):
                color = [0, 100/255.0, 0, 0.9]
                self.draw_obstacles.pub_body(self.obst_pos[self.num_moveable+i].data, 
                                   matrix_to_quaternion(self.draw_obstacles.get_rot_mat(self.obst_rot[self.num_moveable+i].data).T),
                                   [self.compliant_dimen[i][0]*2, self.compliant_dimen[i][1]*2, self.compliant_dimen[i][2]], 
                                   color, 
                                   obst_counter, 
                                   self.draw_obstacles.Marker.CYLINDER)
                obst_counter = obst_counter + 1


            for i in xrange(self.num_fixed):
                color2 = [0.3, 0.,0.,0.7]
                    
                if self.fixed_ctype[i] == 'wall':
                    
                    obst_rot_mat = self.draw_links.get_rot_mat(self.obst_rot[i].data).T
                    ## dim_jt       = [self.fixed_dimen[i][1]/2.0, self.fixed_dimen[i][1]/2.0, self.fixed_dimen[i][2]]

                    ## #jt_pos_loc = link_rot_mat*np.array([0, 0, self.links_dim[i][2]/2.0]).reshape(3,1)
                    ## #jt_pos_plus = np.array(self.links_pos[i].data).reshape(3,1) + jt_pos_loc
                    ## #jt_pos_neg = np.array(self.links_pos[i].data).reshape(3,1) - jt_pos_loc                                    

                    ## jt_pos_loc  = obst_rot_mat*np.array([0, 0, self.fixed_dimen[i][2]/2.0]).reshape(3,1)                    
                    ## jt_pos_plus = self.obst_pos[self.num_moveable+self.num_compliant+i].data + jt_pos_loc
                    ## jt_pos_neg  = self.obst_pos[self.num_moveable+self.num_compliant+i].data - jt_pos_loc                    

                    ## self.draw_links.pub_body(jt_pos_plus, matrix_to_quaternion(obst_rot_mat), dim_jt, color_links[i], link_counter, self.draw_links.Marker.SPHERE)
                    ## self.draw_links.pub_body(jt_pos_neg, matrix_to_quaternion(obst_rot_mat), dim_jt, color_links[i], link_counter+1, self.draw_links.Marker.SPHERE)
                    ## link_counter=link_counter+2

                    #print self.num_moveable+self.num_compliant+i
                    #print self.obst_rot[self.num_moveable+self.num_compliant+i].data
                    
                    self.draw_obstacles.pub_body(self.obst_pos[self.num_moveable+self.num_compliant+i].data, 
                                                 matrix_to_quaternion(self.draw_obstacles.get_rot_mat(self.obst_rot[self.num_moveable+self.num_compliant+i].data).T),
                                                 [self.fixed_dimen[i][0], self.fixed_dimen[i][1], self.fixed_dimen[i][2]], 
                                                 color2, 
                                                 obst_counter, 
                                                 self.draw_obstacles.Marker.CUBE)
                else:                    
                    self.draw_obstacles.pub_body(self.obst_pos[self.num_moveable+self.num_compliant+i].data, 
                                                 matrix_to_quaternion(self.draw_obstacles.get_rot_mat(self.obst_rot[self.num_moveable+self.num_compliant+i].data).T),
                                                 [self.fixed_dimen[i][0]*2, self.fixed_dimen[i][1]*2, self.fixed_dimen[i][2]], 
                                                 color2, 
                                                 obst_counter, 
                                                 self.draw_obstacles.Marker.CYLINDER)
                obst_counter = obst_counter + 1
        #self.num_fixed = rospy.get_param('m3/software_testbed/num_fixed')

        self.lock.release()

    def publish_goal_marker(self, goal_pos):
        o = np.matrix([0.,0.,0.,1.]).T
        g_m = hv.single_marker(goal_pos, o, 'sphere',
                               '/world', color=(0., 1., 1., 1.),
                               scale = (0.02, 0.02, 0.02),
                               m_id = 23, duration=0)
        g_m.header.stamp = rospy.Time.now()
        self.goal_marker_pub.publish(g_m)    
        #self.draw.pub.publish(g_m)



if __name__ == '__main__':
    draw_stuff = DrawAll()
    rospy.loginfo('Started visualizing bodies!')
    while not rospy.is_shutdown():
        draw_stuff.draw_bodies()
        draw_stuff.publish_goal_marker(draw_stuff.goal)
        rospy.sleep(0.1)

                        

                        
