#!/usr/bin/env python
import numpy as np
import math
from multi_link_common import *

#height is probably 0 from multi_link_common.py
#total mass and total length are also defined in multi_link_common.py

num_links = 1.0
link_length = 0.1345
link_mass = 0.416

ee_location = np.matrix([0., -link_length, height]).T
#bod_shapes = ['cube', 'cube', 'cube', 'cube']]  
bod_shapes = ['capsule'] 

bod_dimensions = [[0.03, 0.03, link_length]]*int(num_links)

bod_com_position = [[0., -0.078, height]]


bod_color = [[0.4, 0.4, 0.4, 1]]
bod_num_links = 1
bod_mass = [link_mass]*bod_num_links

bod_names = ['link1']
bod_rotation   = [np.radians([-90.,0.,0.]).tolist()]

bodies ={'shapes':bod_shapes, 'dim':bod_dimensions, 'num_links':bod_num_links,
         'com_pos':bod_com_position, 'mass':bod_mass, 'name':bod_names, 'color':bod_color,
         'rotation':bod_rotation}



b_jt_axis = [[0.,0.,1.]]
b_jt_anchor = [[0., 0., height]]
b_jt_type = ['hinge']


b_jt_kp = [1.032]
#b_jt_kp = [2.032]
b_jt_kd = [0.0112]
b_jt_limits_max = np.radians([180]).tolist()
b_jt_limits_min = np.radians([-180]).tolist()
b_jt_axis = [[0.,0.,1.]]
b_jt_attach = [[0, -1]]

b_jt_start = [0.0] #(gives ee pos of [0, -0.2, 0]


b_jts = {'anchor':b_jt_anchor, 'axis':b_jt_axis, 'jt_lim_max':b_jt_limits_max,
         'jt_lim_min':b_jt_limits_min, 'jt_init':b_jt_start, 'jt_attach':b_jt_attach,
         'jt_stiffness':b_jt_kp, 'jt_damping':b_jt_kd, 'jt_type':b_jt_type}


