
from three_link_planar_common import *

bod_shapes = ['cube', 'cube', 'cube']  

bod_dimensions = [[0.03, 0.03, torso_half_width], [0.03, 0.03, upper_arm_length], 
                  [0.03, 0.03, forearm_length]]

bod_com_position = [[0., -torso_half_width/2., height], 
                    [0., -torso_half_width-upper_arm_length/2., height], 
                    [0., -torso_half_width-upper_arm_length-forearm_length/2., height]]

bodies ={'shapes':bod_shapes, 'dim':bod_dimensions, 'num_links':bod_num_links,
         'com_pos':bod_com_position, 'mass':bod_mass, 'name':bod_names, 'color':bod_color}


