
import numpy as np
import hrl_lib.util as ut

import matplotlib_util.util as mpu
import matplotlib.pyplot as pp


d = ut.load_pickle('c.pkl')

pos_list = d['pos']
norm_list = d['norm']


pos_xy = np.column_stack(pos_list)
norm_vec = np.matrix(np.column_stack(norm_list))

mpu.figure()

pp.plot(pos_xy[0,:], pos_xy[1,:], 'o')
mpu.plot_quiver_yxv(pos_xy[1,:], pos_xy[0,:], norm_vec, scale=10)

pp.axis('equal')
pp.show()




