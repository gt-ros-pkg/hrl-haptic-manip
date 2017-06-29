
import numpy as np
import math

height = 0.0
torso_half_width = 0.196
upper_arm_length = 0.334
forearm_length = 0.288

ee_location = np.matrix([0., -torso_half_width-upper_arm_length-forearm_length, height]).T

bod_color = [[0.4, 0.4, 0.4, 1], [0.8, 0.8, 0.8, 1], [0.33, 0.33, 0.33, 1]]
bod_num_links = 3

bod_mass = [11.34/4.0, 2.3, 1.32]

bod_names = ['torso_lift_link', 'elbow_link', 'wrist_link']

bod_rotation   = [np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist()]

b_jt_axis = [[0.,0.,1.],[0.,0.,1.], [0.,0.,1.]]
b_jt_anchor = [[0., 0., height], [0., -torso_half_width, height], 
               [0., -torso_half_width-upper_arm_length, height]]
b_jt_kp = [30., 30., 20.]
b_jt_kd = [20., 20., 10.]
b_jt_limits_max = np.radians([150, 162, 159]).tolist()
b_jt_limits_min = np.radians([-150, -63, 0]).tolist()
b_jt_axis = [[0.,0.,1.],[0.,0.,1.], [0.,0.,1.]]
b_jt_attach = [[0, -1], [1, 0], [2,1]]

b_jt_start = np.radians([-60., 45., 135.]).tolist()
#b_jt_start = np.radians([0.0, 0, 0]).tolist()
b_jt_type = ['hinge','hinge','hinge']

b_base_jt_kp = [10., 10., 10.]
b_base_jt_kd = [1., 1., 1.]
b_base_jt_limits_max = [10., 10., 10.]
b_base_jt_limits_min = [-10., -10., -10.]


b_jts = {'anchor':b_jt_anchor, 'axis':b_jt_axis, 'jt_lim_max':b_jt_limits_max,
         'jt_lim_min':b_jt_limits_min, 'jt_init':b_jt_start, 'jt_attach':b_jt_attach,
         'jt_stiffness':b_jt_kp, 'jt_damping':b_jt_kd, 'jt_type':b_jt_type,
         'base_jt_stiffness':b_base_jt_kp, 'base_jt_damping':b_base_jt_kd,
         'base_jt_lim_max':b_base_jt_limits_max, 'base_jt_lim_min':b_base_jt_limits_min}

