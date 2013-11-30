#!/usr/bin/env python
import numpy as np
import math

height = 0.0

l1   = 0.18493-0.085
l2   = 0.085
l3_y = 0.03175 
l3_z = 0.09
l4_z = 0.27795-0.09
l4_x = 0.00635
l5   = 0.065
l6   = 0.27853-0.065
l7   = 0.04
tip  = 0.1
ee_location = np.matrix([l4_x, -l1-l2-l3_y, height-l3_z-l4_z-l5-l6-l7-tip]).T


#7.5, 6.5
#upper_arm = 0.25
#lower_arm  = 0.25
#ee_length = 0.10
"""
l0       = 0.18493
l1       = 0.0
l2       = 0.27795
l3       = 0.00635
l4       = 0.27853
l5       = 0.04318
l6       = 0.279-0.065
lee      = 0.04
ee_location = np.matrix([l3., -l0-l1, height-l2-l4-l5]).T
"""

arm_thck = 0.07

#bod_shapes = ['cube', 'cube', 'cube', 'cube', 'cube', 'cube', 'cube']  
#bod_shapes = ['sphere', 'sphere', 'capsule', 'capsule', 'sphere', 'sphere', 'capsule']
bod_shapes = ['capsule', 'capsule', 'capsule', 'capsule', 'capsule', 'capsule', 'capsule']

# if capsule, the value order is [radius*2, none, length]
# if sphere, the value order is [none,none,radius*2].
bod_dimensions = [[arm_thck, arm_thck, l2], 
                  [arm_thck, arm_thck, np.sqrt(l3_y*l3_y+l3_z*l3_z).tolist()],  
                  [arm_thck, arm_thck, l4_z], 
                  #[arm_thck, arm_thck, np.sqrt(l4_z*l4_z+l4_x*l4_x)], 
                  [arm_thck, arm_thck, l5],  
                  [arm_thck, arm_thck, l6],
                  [arm_thck, arm_thck, l7],
                  [arm_thck, arm_thck, tip]] 

# body block rotation Rx, Ry, Rz
bod_rotation   = [np.radians([90.,0.,0.]).tolist(),
                  np.radians([-math.atan2(l3_y,l3_z)*180.0/np.pi,0.,0.]).tolist(),
                  np.radians([0.,0.,0.]).tolist(),
                  np.radians([0.,0.,0.]).tolist(),
                  np.radians([0.,0.,0.]).tolist(),
                  np.radians([0.,0.,0.]).tolist(),
                  np.radians([0.,0.,0.]).tolist()]

bod_com_position = [[0.,   -l1-l2/2.0,      height], 
                    [0.,   -l1-l2-l3_y/2.0, height-l3_z/2.0], 
                    [0.,   -l1-l2-l3_y,     height-l3_z-l4_z/2.0], 
                    [l4_x, -l1-l2-l3_y,     height-l3_z-l4_z-l5/2.0],
                    [l4_x, -l1-l2-l3_y,     height-l3_z-l4_z-l5-l6/2.0],
                    [l4_x, -l1-l2-l3_y,     height-l3_z-l4_z-l5-l6-l7/2.0],
                    [l4_x, -l1-l2-l3_y,     height-l3_z-l4_z-l5-l6-l7-tip/2.0]]
                    
bod_color = [[0.4, 0.4, 0.4, 1], [0.8, 0.8, 0.8, 1], [0.33, 0.3, 0.33, 1], 
             [0.4, 0.4, 0.4, 1], [0.8, 0.8, 0.8, 1], [0.33, 0.33, 0.33, 1], 
             [0.05, 0.05, 0.05, 1]]
bod_num_links = 7
bod_mass = [2.0, 0.636, 2.3, 0.339, 1.18, 0.32, 0.32]

bod_names = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7']
bodies ={'shapes':bod_shapes, 'dim':bod_dimensions, 'num_links':bod_num_links,
         'com_pos':bod_com_position, 'mass':bod_mass, 'name':bod_names, 'color':bod_color,
         'rotation':bod_rotation }

b_jt_axis = [[0.,-1.,0.], [-1.,0.,0.], [0., 0., -1.], [0.,-1.,0.], [0., 0., -1.], [0., 1., 0.], [1., 0., 0.]]
b_jt_anchor = [[0., -l1,         height], 
               [0., -l1-l2,      height],
               [0., -l1-l2-l3_y, height-l3_z],
               [0., -l1-l2-l3_y, height-l3_z-l4_z],
               [0., -l1-l2-l3_y, height-l3_z-l4_z-l5],
               [0., -l1-l2-l3_y, height-l3_z-l4_z-l5-l6],
               [0., -l1-l2-l3_y, height-l3_z-l4_z-l5-l6-l7]]

b_num_jts = 7
b_jt_limits_max = np.radians([ 120., 122., 77.5, 144., 122.,  45., 45.]).tolist()  #45.
b_jt_limits_min = np.radians([ -47.61,  -20., -77.5,   -5.0, -80., -45., -45.]).tolist()  #-45.
b_jt_attach = [[0, -1], [1, 0], [2,1], [3,2], [4,3], [5,4], [6,5]]
b_jt_start = np.radians([0., 0., 0., 0., 0., 0., 0.]).tolist() #0.
#b_jt_type = ['universal','universal','universal','hinge','universal','universal','universal']
b_jt_type = ['hinge','hinge','hinge','hinge','hinge','hinge','hinge']

# 
b_jt_kp = [25., 25., 15., 15., 4., 2., 1. ]
b_jt_kd = [2.5, 2.5, 1., 2.5, 0.1, 0.1, 0.1 ]

b_jts = {'anchor':b_jt_anchor, 'axis':b_jt_axis, 'jt_lim_max':b_jt_limits_max,
         'jt_lim_min':b_jt_limits_min, 'jt_init':b_jt_start, 'jt_attach':b_jt_attach, 'num_jts':b_num_jts,
         'jt_stiffness':b_jt_kp, 'jt_damping':b_jt_kd, 'jt_type':b_jt_type}

