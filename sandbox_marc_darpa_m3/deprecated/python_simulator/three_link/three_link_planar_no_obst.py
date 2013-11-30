import numpy as np

q_eq = np.matrix([1.57, 0, 1.3]).T


shapes = ['cylinder', 'cylinder', 'cylinder']
dimensions = [[0.2, 0.2, 3], [0.2, 0.2, 2], [0.2, 0.2, 1.8]]
com_position = [[1.5, 0, 1],[4, 0, 1], [5.9, 0, 1]]
color = [[1, 1, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
mass = [None, None, None]
density =[20, 20, 20]
rotation = [[0, 0, -1, 0, 1, 0, 1, 0, 0], 
            [0, 0, -1, 0, 1, 0, 1, 0, 0],[0, 0, -1, 0, 1, 0, 1, 0, 0]]
inertia = [None, None, None]
bodies ={'shapes':shapes, 'dim':dimensions, 'com_pos':com_position, 
       'color':color, 'mass':mass, 'density':density, 
       'rotation':rotation, 'inertia':inertia}


geom_bodies_ind = [0, 1, 2, None]
geom_name = ['link1', 'link2', 'link3', 'ground']
geom_dim = [None, None, [20, 20, 0.1]]
geom_shape = [None, None, 'cube']
geom_pos = [None, None, [0, 0, -0.05]]
geometries = {'body':geom_bodies_ind, 'name':geom_name, 'dim':geom_dim,
              'shape':geom_shape, 'pos':geom_pos}

jt_bodies = [[0,-1],[1,0]]
jt_type = ['revolute', 'revolute']
jt_anchor = [[0, 0, 1], [3, 0, 1]]
jt_axis = [[0,0,1],[0,0,1]]
jt_vis_size = [[0.25, 0.25, 0.25], [0.25, 0.25, 0.25]]
jt_vis_color = [[1, 0, 0, 1], [1, 0, 0, 1]]
jt_feedback = [0, 0]

jts = {'bodies':jt_bodies, 'jt_type':jt_type, 'anchor':jt_anchor, 'axis':jt_axis,
          'vis_size':jt_vis_size, 'vis_color':jt_vis_color, 'feedback':jt_feedback}
