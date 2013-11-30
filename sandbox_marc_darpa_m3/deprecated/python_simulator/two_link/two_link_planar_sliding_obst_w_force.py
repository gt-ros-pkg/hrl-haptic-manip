import numpy as np

q_eq = np.matrix([1.57, 1.57/2.0]).T

shapes = ['cylinder', 'cylinder', 'cube', 'cube']
dimensions = [[0.2, 0.2, 3], [0.2, 0.2, 2],  [0.5, 0.5, 1.9], [0.5, 0.5, 0.1]]
com_position = [[1.5, 0, 1],[4, 0, 1],
                [3.2, 3.2, 1.05], [3.2, 3.2, 0.05]]
#                [2, 2, 1.55], [2, 2, 0.05]]

color = [[1, 1, 0, 1], [0, 0, 1, 1],  [1, 1, 1, 1], [1, 1,1,1]]
mass = [None, None, None, None]
density =[20, 20, 10, 10]
rotation = [[0, 0, -1, 0, 1, 0, 1, 0, 0], [0, 0, -1, 0, 1, 0, 1, 0, 0], None, None]
inertia = [None, None, None, None]
bodies ={'shapes':shapes, 'dim':dimensions, 'com_pos':com_position, 
       'color':color, 'mass':mass, 'density':density, 
       'rotation':rotation, 'inertia':inertia}


geom_bodies_ind = [0, 1, 2, 3, None]
geom_name = ['link1', 'link2', 'obstacle', 'obstacle','ground']
geom_dim = [None, None, None, None, [20, 20, 0.1]]
geom_shape = [None, None, None, None, 'cube']
geom_pos = [None, None, None, None, [0, 0, -0.05]]
geometries = {'body':geom_bodies_ind, 'name':geom_name, 'dim':geom_dim,
              'shape':geom_shape, 'pos':geom_pos}

jt_q_ind = [True, True, False, False]
jt_bodies = [[0,-1],[1,0], [3, -1], [2,3]]#, [2, -1]]  # [2,-1] obstacle and ground 
jt_type = ['revolute', 'revolute', 'prismatic', 'fixed']#, 'prismatic']  #'fixed'
jt_anchor = [[0, 0, 1], [3, 0, 1], com_position[2], com_position[3]]#, com_position[2]]  #[
jt_axis = [[0,0,1],[0,0,1], [1,0,0], None]#, [0,1,0]]
jt_vis_size = [[0.25, 0.25, 0.25], [0.25, 0.25, 0.25], [0.25, 0.25, 0.25], [0.25, 0.25, 0.25]]
jt_vis_color = [[1, 0, 0, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1,0,0,1]]
jt_feedback = [0, 0, 0, 1]

jts = {'bodies':jt_bodies, 'jt_type':jt_type, 'anchor':jt_anchor, 'axis':jt_axis,
          'vis_size':jt_vis_size, 'vis_color':jt_vis_color, 'feedback':jt_feedback}
