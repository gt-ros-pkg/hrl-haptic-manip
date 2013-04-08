import numpy as np
import math

from three_link_planar_common import *


b_jt_limits_max = np.radians([150, 162, 159]).tolist()
b_jt_limits_min = np.radians([-150, -162, -159]).tolist()

b_jts = {'anchor':b_jt_anchor, 'axis':b_jt_axis, 'jt_lim_max':b_jt_limits_max,
         'jt_lim_min':b_jt_limits_min, 'jt_init':b_jt_start, 'jt_attach':b_jt_attach,
         'jt_stiffness':b_jt_kp, 'jt_damping':b_jt_kd}

bod_shapes = ['capsule', 'capsule', 'capsule'] 

dia = 0.03
bod_dimensions = [[dia, dia, torso_half_width], [dia, dia, upper_arm_length], 
                  [dia, dia, forearm_length-dia/2]]

bod_com_position = [[0., -torso_half_width/2., height], 
                    [0., -torso_half_width-upper_arm_length/2., height], 
                    [0., -torso_half_width-upper_arm_length-forearm_length/2.+dia/4, height]]

bodies ={'shapes':bod_shapes, 'dim':bod_dimensions, 'num_links':bod_num_links,
         'com_pos':bod_com_position, 'mass':bod_mass, 'name':bod_names, 'color':bod_color}

