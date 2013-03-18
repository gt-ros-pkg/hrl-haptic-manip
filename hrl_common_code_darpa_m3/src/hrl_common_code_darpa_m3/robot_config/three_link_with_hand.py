
import numpy as np
import math

height = 0.0
upper_arm_length = 0.334
forearm_length = 0.288
hand_length = 0.12

ee_location = np.matrix([0., -upper_arm_length-forearm_length-hand_length, height]).T

bod_color = [[0.4, 0.4, 0.4, 1], [0.8, 0.8, 0.8, 1], [0.33, 0.33, 0.33, 1]]
bod_num_links = 3

bod_mass = [2.3, 1.32, 0.7]

bod_names = ['link1', 'link2', 'link3']

b_jt_axis = [[0.,0.,1.],[0.,0.,1.], [0.,0.,1.]]
b_jt_anchor = [[0., 0., height], [0., -upper_arm_length, height], 
               [0., -upper_arm_length-forearm_length, height]]
b_jt_kp = [20., 15., 10.]
b_jt_kd = [3., 2., 1.]
b_jt_limits_max = np.radians([162, 159, 90]).tolist()
b_jt_limits_min = np.radians([-63, 0, -45]).tolist()
b_jt_axis = [[0.,0.,1.],[0.,0.,1.], [0.,0.,1.]]
b_jt_attach = [[0, -1], [1, 0], [2,1]]

b_jt_start = np.radians([-30.0, 150, 15]).tolist()
#b_jt_start = np.radians([0.0, 10, 0]).tolist()

b_jts = {'anchor':b_jt_anchor, 'axis':b_jt_axis, 'jt_lim_max':b_jt_limits_max,
         'jt_lim_min':b_jt_limits_min, 'jt_init':b_jt_start, 'jt_attach':b_jt_attach,
         'jt_stiffness':b_jt_kp, 'jt_damping':b_jt_kd}



bod_shapes = ['capsule', 'capsule', 'capsule'] 

dia = 0.03
bod_dimensions = [[dia, dia, upper_arm_length], [dia, dia, forearm_length], 
                  [dia, dia, hand_length-dia/2]]

bod_com_position = [[0., -upper_arm_length/2., height], 
                    [0., -upper_arm_length-forearm_length/2., height], 
                    [0., -upper_arm_length-forearm_length-hand_length/2.+dia/4, height]]

bodies ={'shapes':bod_shapes, 'dim':bod_dimensions, 'num_links':bod_num_links,
         'com_pos':bod_com_position, 'mass':bod_mass, 'name':bod_names, 'color':bod_color}

