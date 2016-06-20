#####################################################################
#header.frame_id  The frame in which all data in the messgae is defined (typically torso_lift_link).
#                 Data must be transformed to this frame before populating this message.
#
#sensor_type  #'force' or 'distance' depending on the type of sensor
#
#link_names   The names of the link to which each taxel is rigidly fixed (the Jacobian of elements distal to this link do not impact taxel movement)
#
#centers_x _y _z  The position in x,y,and z components of the center of the taxel in the header frame
#normals_x _y _z  The unit normal direction in x,y,z components of each taxel in the header frame
#values_x _y _z  The calibrated sensor values (force or distance) at each taxel, in components, in header frame
#
#####################################################################
Header header

string sensor_type # 'force' or 'proximity'. If nothing is specified the haptic state node assumes this is force

string[] link_names # If you have a single taxel array for multiple bodies, specify the link name/frame here

float64[] centers_x
float64[] centers_y
float64[] centers_z

float64[] normals_x
float64[] normals_y
float64[] normals_z

float64[] values_x
float64[] values_y
float64[] values_z

string[] contact_type # Rigid, moveable, compliant, etc.
