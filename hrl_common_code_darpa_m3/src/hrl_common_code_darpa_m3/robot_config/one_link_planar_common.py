
import numpy as np
import math

height = 0.0

ee_location = np.matrix([0., 0.13545, height]).T

bod_color = [[0.4, 0.4, 0.4, 1]]
bod_num_links = 1

bod_mass = [0.416]

bod_names = ['link1']

b_jt_axis = [[0.,0.,1.]]
b_jt_anchor = [[0., 0., height]]
b_jt_kp = [1.032]
b_jt_kd = [0.0112]
b_jt_limits_max = np.radians([68]).tolist()
b_jt_limits_min = np.radians([-68]).tolist()
b_jt_axis = [[0.,0.,1.]]
b_jt_attach = [[0, -1]]

b_jt_start = [0.0] #np.radians([-60., 45., 135.]).tolist()
#b_jt_start = np.radians([0.0, 0, 0]).tolist()

b_jts = {'anchor':b_jt_anchor, 'axis':b_jt_axis, 'jt_lim_max':b_jt_limits_max,
         'jt_lim_min':b_jt_limits_min, 'jt_init':b_jt_start, 'jt_attach':b_jt_attach,
         'jt_stiffness':b_jt_kp, 'jt_damping':b_jt_kd}

