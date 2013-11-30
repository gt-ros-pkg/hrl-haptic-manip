import numpy as np
import math

h_goal = np.matrix([-1.41013431, 4.41590087, 1.]).T
q_goal = np.matrix([1.57, 1.57/2.0]).T
linkage_offset_from_ground = np.matrix([0., 0., 1.]).T
ee_location = np.matrix([5., 0., 1.]).T
mu = np.matrix([0, 80000, 0, 50, 0, 50, 0, 50]).T

bod_shapes = ['cylinder', 'cylinder', 'cylinder']
bod_dimensions = [[0.2, 0.2, 2], [0.2, 0.2, 2], [0.2, 0.2, 1]]
bod_com_position = [[1.0, 0, 1],[3.0, 0, 1], [4.5, 0, 1.]]
bod_color = [[1, 1, 0, 1], [0, 0, 1, 1], [1, 0.5, 0,1]]
bod_mass = [None, None, None]
bod_density =[20, 20, 20]
bod_rotation = [[0, 0, -1, 0, 1, 0, 1, 0, 0], [0, 0, -1, 0, 1, 0, 1, 0, 0], [0, 0, -1, 0, 1, 0, 1, 0, 0]]
bod_inertia = [None, None, None]
bod_names = ['link1', 'link2', 'link3']
bodies ={'shapes':bod_shapes, 'dim':bod_dimensions, 'com_pos':bod_com_position, 
       'color':bod_color, 'mass':bod_mass, 'density':bod_density, 
       'rotation':bod_rotation, 'inertia':bod_inertia, 'name':bod_names}


b_geom_bodies_ind = [0, 1, 2, None]
b_geom_name = ['link', 'link', 'link', 'ground']
b_geom_dim = [None, None, None, [20, 20, 0.1]]
b_geom_shape = [None, None,None, 'cube']
b_geom_pos = [None, None, None, [0, 0, -0.05]]
b_geometries = {'body':b_geom_bodies_ind, 'name':b_geom_name, 'dim':b_geom_dim,
              'shape':b_geom_shape, 'pos':b_geom_pos}


b_jt_q_ind = [True, True, True]
b_jt_bodies = [[0,-1],[1,0], [2,1]]
b_jt_type = ['revolute', 'revolute', 'revolute']
b_jt_anchor = [[0, 0, 1], [2, 0, 1], [4, 0, 1]]
b_jt_axis = [[0,0,1],[0,0,1], [0,0,1]]
b_jt_vis_size = [[0.25]*3, [0.25]*3, [0.25]*3]
b_jt_vis_color = [[1, 0, 0, 1], [1, 0, 0, 1], [1, 0, 0, 1]]
b_jt_feedback = [0, 0, 0]
b_jt_lim = [[0, math.pi], [0, math.pi], [-math.pi/2, math.pi/2]]

b_jts = {'bodies':b_jt_bodies, 'jt_type':b_jt_type, 'anchor':b_jt_anchor, 'axis':b_jt_axis,
         'vis_size':b_jt_vis_size, 'vis_color':b_jt_vis_color, 'feedback':b_jt_feedback, 'jt_lim':b_jt_lim}


num_obst = 5
obs_shapes = ['cylinder', 'cylinder', 'cylinder', 'cylinder', 'cylinder', 'cylinder', 'cylinder', 'cylinder',
              'cube', 'cube']
#obs_size[[0.5, 0.5, 2],[0.4, 0.4, 1
obs_dimensions = [[0.4, 0.4, 0.1], [0.4, 0.4, 1.9], [0.4, 0.4, 0.1], [0.4, 0.4, 1.9],
                  [0.4, 0.4, 0.1], [0.4, 0.4, 1.9], [0.4, 0.4, 0.1], [0.4, 0.4, 1.9],
                  [0.2, 4, 0.1], [0.2, 4,1.9]]

# obs_com_position
# obs_com_position = [[3.2, 3.2, 0.05], [3.2, 3.2, 1.05], [2, 3.4, 0.05], [2, 3.4, 0.8], 
#                     [0.5, 3.5, 0.05], [0.5, 3.5, 0.8], [0.5, 2.2, 0.05], [0.5, 2.2, 0.8]]
obs_com_position = [[4, 3.2, 0.05], [4, 3.2, 1.05], [2, 3.4, 0.05], [2, 3.4, 1.05], 
                    [0.5, 3.8, 0.05], [0.5, 3.8, 1.05], [0, 3.5, 0.05], [0, 3.5, 1.05],
                    [-1.5, 2, 0.05], [-1.5, 2, 1.05]]

obs_color = [[1, 1, 1, 1]]*num_obst*2
obs_mass = [None]*num_obst*2
obs_density =[10]*num_obst*2
obs_rotation = [None]*num_obst*2
obs_inertia = [None]*num_obst*2
obs_names = [None, 'obstacle1', None, 'obstacle2', None, 'obstacle3', None, 'obstacle4', None, 'obstacle5', None, 'obstacle6', None, 'obstacle7', None, 'obstacle8']
obstacles ={'shapes':obs_shapes, 'dim':obs_dimensions, 'com_pos':obs_com_position, 
       'color':obs_color, 'mass':obs_mass, 'density':obs_density, 
       'rotation':obs_rotation, 'inertia':obs_inertia, 'name':obs_names}


o_geom_bodies_ind = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
o_geom_name = [None, 'obstacle']*num_obst
o_geom_dim = [None]*num_obst*2
o_geom_shape = [None]*num_obst*2
o_geom_pos = [None]*num_obst*2
o_geometries = {'body':o_geom_bodies_ind, 'name':o_geom_name, 'dim':o_geom_dim,
              'shape':o_geom_shape, 'pos':o_geom_pos}

o_jt_q_ind = [False]*num_obst
o_jt_bodies = [[0,1], [1, -1], [2,3], [3,-1], [4,5], [5,-1], [6,7], [7,-1],
               [8,9], [9, -1]]
#o_jt_type = [ 'fixed', 'prismatic', 'fixed', 'prismatic', 'fixed', 'prismatic', 
o_jt_type = [ 'fixed', 'fixed', 'fixed', 'prismatic', 'fixed', 'prismatic', 
              'fixed', 'prismatic', 'fixed', 'fixed']
o_jt_anchor = [None]*num_obst*2
o_jt_axis = [None, [1,0,0]]*num_obst
o_jt_vis_size = [[0.25]*3]*num_obst*2
o_jt_vis_color = [[1, 0, 0, 1]]*num_obst*2
o_jt_feedback = [1, 0]*num_obst
o_jt_lim = [None]*num_obst*2

o_jts = {'bodies':o_jt_bodies, 'jt_type':o_jt_type, 'anchor':o_jt_anchor, 'axis':o_jt_axis,
         'vis_size':o_jt_vis_size, 'vis_color':o_jt_vis_color, 'feedback':o_jt_feedback, 'jt_lim':o_jt_lim}