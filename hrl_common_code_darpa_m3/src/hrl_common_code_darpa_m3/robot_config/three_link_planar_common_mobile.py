
import numpy as np
import math

height = 0.0
torso_half_width = 0.196
upper_arm_length = 0.334
forearm_length = 0.288

ee_location = np.matrix([0., -torso_half_width-upper_arm_length-forearm_length, height]).T

bod_color = [[0.0, 0.0, 0.0, 1], [0.1, 0.1, 0.1, 1], [0.4, 0.4, 0.4, 1], [0.8, 0.8, 0.8, 1], [0.33, 0.33, 0.33, 1]]
bod_num_links = 5

bod_mass = [100.0, 100.0, 11.34/4.0, 2.3, 1.32]

#bod_names = ['base1', 'base2', 'link1', 'link2', 'link3']
bod_names = ['base_link', 'base_link2', 'torso_lift_link', 'elbow_link', 'wrist_link']
    
bod_rotation   = [np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist()]

b_jt_axis = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.],[0.,0.,1.], [0.,0.,1.]]
b_jt_anchor = [[0., 0., height], [0., 0., height],
               [0., 0., height], [0., -torso_half_width, height], 
               [0., -torso_half_width-upper_arm_length, height]]
b_jt_kp = [100., 100., 30., 20., 15.]
b_jt_kd = [100., 100., 15., 10., 8.]
b_jt_limits_max = np.radians([5, 5, 150, 162, 159]).tolist()
b_jt_limits_min = np.radians([-5, -5, -150, -63, 0]).tolist()
b_jt_attach = [[0, -1], [1, 0], [2,1], [3,2], [4,3]]

b_jt_start = np.radians([0., 0., -60., 45., 135.]).tolist()
#b_jt_start = np.radians([0.0, 0, 0]).tolist()
b_jt_type = ['slider', 'slider', 'hinge','hinge','hinge']

b_jts = {'anchor':b_jt_anchor, 'axis':b_jt_axis, 'jt_lim_max':b_jt_limits_max,
         'jt_lim_min':b_jt_limits_min, 'jt_init':b_jt_start, 'jt_attach':b_jt_attach,
         'jt_stiffness':b_jt_kp, 'jt_damping':b_jt_kd, 'jt_type':b_jt_type}

