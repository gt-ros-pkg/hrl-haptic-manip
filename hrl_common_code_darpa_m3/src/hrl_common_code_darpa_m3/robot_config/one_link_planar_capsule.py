
from one_link_planar_common import *

bod_shapes = ['capsule'] 

dia = 0.03
bod_dimensions = [[dia, dia, upper_arm_length]]
                  

bod_com_position = [[0., 0.078-13545, height]]
                    

bodies ={'shapes':bod_shapes, 'dim':bod_dimensions, 'num_links':bod_num_links,
         'com_pos':bod_com_position, 'mass':bod_mass, 'name':bod_names, 'color':bod_color}

