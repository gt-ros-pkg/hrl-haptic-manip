#!/usr/bin/env python
import numpy as np
import math

bod_planar = 0

height = 0.0

l1 = 0.185-0.085
l2 = 0.085
l3 = 0.09
l4 = 0.278-0.09
l4_offset = 0.00635
l5 = 0.065
l6 = 0.279-0.065
lee = 0.1
lee_block = 0.04

upper_arm = 0.25
lower_arm  = 0.25
ee_length = 0.10
ee_location = np.matrix([0., -l1-l2, height-l3-l4-l5-l6-lee_block]).T
arm_thck = 0.07

bod_shapes = ['cube', 'cube', 'cube', 'cube', 'cube', 'cube', 'cube']  
#bod_shapes = ['sphere', 'capsule', 'capsule', 'capsule', 'sphere', 'sphere', 'capsule']

# if capsule, the value order is [radius*2, none, length]
# if sphere, the value order is [none,none,radius*2].
bod_dimensions = [[arm_thck, arm_thck, l2], [arm_thck, arm_thck, l3], 
                  [arm_thck, arm_thck, l4],  [arm_thck, arm_thck, l5], 
                  [arm_thck, arm_thck, l6],  [arm_thck, arm_thck, lee_block],
                  [arm_thck, arm_thck, lee]] 

bod_com_position = [[0., -l1-l2/2.0, height], 
                    [0., -l1-l2, height-l3/2.0], 
                    [0., -l1-l2, height-l3-l4/2.0], 
                    [0., -l1-l2, height-l3-l4-l5/2.0],
                    [0., -l1-l2, height-l3-l4-l5-l6/2.0],
                    [0., -l1-l2, height-l3-l4-l5-l6-lee_block/2.0],
                    [0., -l1-l2, height-l3-l4-l5-l6-lee/2.0]]
                    
bod_color = [[0.4, 0.4, 0.4, 1], [0.8, 0.8, 0.8, 1], [0.33, 0.3, 0.33, 1], 
             [0.4, 0.4, 0.4, 1], [0.8, 0.8, 0.8, 1], [0.33, 0.33, 0.33, 1], 
             [0.05, 0.05, 0.05, 1]]
bod_num_links = 7
bod_mass = [2.0, 0.636, 2.3, 0.339, 1.18, 0.32, 0.32]

# if we do stuff other than cubes, we'll need to be really careful with rotations,
# it was a pain before - 17 May 2011, marc
#bod_rotation = [[0, 0, -1, 0, 1, 0, 1, 0, 0], [0, 0, -1, 0, 1, 0, 1, 0, 0], [0, 0, -1, 0, 1, 0, 1, 0, 0]]
bod_names = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7']
bodies ={'planar':bod_planar,
         'shapes':bod_shapes, 'dim':bod_dimensions, 'num_links':bod_num_links,
         'com_pos':bod_com_position, 'mass':bod_mass, 'name':bod_names, 'color':bod_color}
         #'rotation':bod_rotation, 'color':bod_color, 'density':bod_density, }

#When we want to make very general, we'll need this again - marc 17 May 2011
#b_jt_type = ['revolute', 'revolute', 'revolute']
#b_jt_vis_size = [[0.07, 0.07, 0.07], [0.07, 0.07, 0.07], [0.07, 0.07, 0.07]]
#b_jt_vis_color = [[1, 0, 0, 1], [1, 0, 0, 1], [1, 0, 0, 1]]
#b_jt_feedback = [0, 0, 0]

b_jt_axis = [[0.,-1.0,0.],[-1.,0.,0.], [0., 0., -1.], [0.,-1.,0.], [0., 0., -1.], [0., 1., 0.], [1., 0., 0.]]
b_jt_anchor = [[0., -l1, height], [0., -l1-l2, height],
               [0., -l1-l2, height-l3],
               [0., -l1-l2, height-l3-l4],
               [0., -l1-l2, height-l3-l4-l5],
               [0., -l1-l2, height-l3-l4-l5-l6],
               [0., -l1-l2, height-l3-l4-l5-l6]]
b_num_jts = 7
b_jt_limits_max = np.radians([ 120., 122., 77.5, 144., 122.,  45., 45.]).tolist()  #45.
b_jt_limits_min = np.radians([ -47.61,  -20., -77.5,   -5.0, -80., -45., -45.]).tolist()  #-45.
b_jt_attach = [[0, -1], [1, 0], [2,1], [3,2], [4,3], [5,4], [6,5]]
b_jt_start = np.radians([0., 0., 0., 0., 0., 0., 0.]).tolist() #0.

# 
b_jt_kp = [25., 25., 15., 15., 4., 2., 1. ]
b_jt_kd = [2.5, 2.5, 1., 2.5, 0.1, 0.1, 0.1 ]

b_jts = {'anchor':b_jt_anchor, 'axis':b_jt_axis, 'jt_lim_max':b_jt_limits_max,
         'jt_lim_min':b_jt_limits_min, 'jt_init':b_jt_start, 'jt_attach':b_jt_attach, 'num_jts':b_num_jts,
         'jt_stiffness':b_jt_kp, 'jt_damping':b_jt_kd}
         #'bodies':b_jt_bodies, 'jt_type':b_jt_type, 
         #'vis_size':b_jt_vis_size, 'vis_color':b_jt_vis_color, 'feedback':b_jt_feedback

"""
if __name__ == '__main__':
    import roslib; roslib.load_manifest('darpa_m3')
    import rospy

    # uploading to the param server so that the ODE C++ software
    # simulation creates a robot with the dimensions etc. specified in
    # this file.
    rospy.init_node('arm_config_upload')
    rospy.set_param('m3/software_testbed/joints/axes', b_jts['axis'])
    rospy.set_param('m3/software_testbed/joints/anchor', b_jts['anchor'])
    rospy.set_param('m3/software_testbed/joints/max', b_jts['jt_lim_max'])
    rospy.set_param('m3/software_testbed/joints/min', b_jts['jt_lim_min'])
    rospy.set_param('m3/software_testbed/joints/attach', b_jts['jt_attach'])
    rospy.set_param('m3/software_testbed/joints/init_angle', b_jts['jt_init'])
    rospy.set_param('m3/software_testbed/joints/num_joints', b_jts['num_jts'])
    rospy.set_param('m3/software_testbed/linkage/dimensions', bodies['dim'])
    rospy.set_param('m3/software_testbed/linkage/colors', bodies['color'])
    rospy.set_param('m3/software_testbed/linkage/mass', bodies['mass'])
    rospy.set_param('m3/software_testbed/linkage/positions', bodies['com_pos'])
    rospy.set_param('m3/software_testbed/linkage/num_links', bodies['num_links'])
    rospy.set_param('m3/software_testbed/linkage/shapes', bodies['shapes'])
    rospy.set_param('m3/software_testbed/linkage/link_names', bodies['name'])
"""


