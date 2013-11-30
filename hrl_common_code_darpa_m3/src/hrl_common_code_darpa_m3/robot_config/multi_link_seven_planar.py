#!/usr/bin/env python
import numpy as np
import math
from multi_link_common import *

#height is probably 0 from multi_link_common.py
#total mass and total length are also defined in multi_link_common.py

num_links = 7.0
link_length = total_length/num_links
link_mass = total_mass/num_links

ee_location = np.matrix([0., -link_length*7.0, height]).T
#bod_shapes = ['cube', 'cube', 'cube', 'cube', 'cube', 'cube', 'cube']  
bod_shapes = ['capsule', 'capsule', 'capsule', 'capsule', 'capsule', 'capsule', 'capsule'] 

bod_dimensions = [[0.03, 0.03, link_length]]*7

bod_com_position = [[0., -link_length/2., height], 
                    [0., -3.0/2.0*link_length, height], 
                    [0., -5.0/2.0*link_length, height],
                    [0., -7.0/2.0*link_length, height], 
                    [0., -9.0/2.0*link_length, height], 
                    [0., -11.0/2.0*link_length, height],
                    [0., -13.0/2.0*link_length, height]]

bod_color = [[0.4, 0.4, 0.4, 1], [0.8, 0.8, 0.8, 1], [0.33, 0.33, 0.33, 1], [0.5, 0.5, 0.5, 1], [0.1, 0.1, 0.1, 1], [0.7, 0.7, 0.7, 1], [0.2, 0.2, 0.2, 1]]
bod_num_links = 7
bod_mass = [link_mass]*bod_num_links

bod_names = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7']
bodies ={'shapes':bod_shapes, 'dim':bod_dimensions, 'num_links':bod_num_links,
         'com_pos':bod_com_position, 'mass':bod_mass, 'name':bod_names, 'color':bod_color}

b_jt_axis = [[0.,0.,1.],[0.,0.,1.], [0.,0.,1.], [0.,0.,1.],[0.,0.,1.], [0.,0.,1.], [0.,0.,1.]]
b_jt_anchor = [[0., 0., height], 
               [0., -link_length, height], 
               [0., -2*link_length, height],
               [0., -3*link_length, height],
               [0., -4*link_length, height],
               [0., -5*link_length, height],
               [0., -6*link_length, height]]

b_jt_kp = [25., 10., 8., 5., 3., 1.5, 1.]
b_jt_kd = [1.8, 1., 1., 1.2, 0.8, .5, 0.2]
b_jt_limits_max = np.radians([180, 120, 120, 120, 120, 120, 120]).tolist()
b_jt_limits_min = np.radians([-180, -120, -120, -120, -120, -120, -120]).tolist()
b_jt_axis = [[0.,0.,1.],[0.,0.,1.], [0.,0.,1.], [0.,0.,1.],[0.,0.,1.], [0.,0.,1.], [0.,0.,1.]]
b_jt_attach = [[0, -1], [1, 0], [2,1], [3,2], [4,3], [5,4], [6,5]]
b_jt_start = [-1.99, 0.417, 0.689, 0.892, 0.986, 0.656, 0.501] #(gives ee pos of [0, -0.2, 0]

b_jts = {'anchor':b_jt_anchor, 'axis':b_jt_axis, 'jt_lim_max':b_jt_limits_max,
         'jt_lim_min':b_jt_limits_min, 'jt_init':b_jt_start, 'jt_attach':b_jt_attach,
         'jt_stiffness':b_jt_kp, 'jt_damping':b_jt_kd}





