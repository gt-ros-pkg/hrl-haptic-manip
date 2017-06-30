
from three_link_planar_common_mobile import *

bod_shapes = ['cube', 'cube', 'capsule', 'capsule', 'capsule'] 

dia = 0.03
bod_dimensions = [[dia, dia, dia], [dia, dia, dia],
                  [dia, dia, torso_half_width], [dia, dia, upper_arm_length], 
                  [dia, dia, forearm_length-dia/2]]

bod_com_position = [[0.,0.,0.], [0.,0.,0.],
                    [0., -torso_half_width/2., height], 
                    [0., -torso_half_width-upper_arm_length/2., height], 
                    [0., -torso_half_width-upper_arm_length-forearm_length/2.+dia/4, height]]

bod_rotation   = [np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist(),
                  np.radians([-90.,0.,0.]).tolist()]

bodies ={'shapes':bod_shapes, 'dim':bod_dimensions, 'num_links':bod_num_links,
         'com_pos':bod_com_position, 'mass':bod_mass, 'name':bod_names, 'color':bod_color,
         'rotation':bod_rotation }

