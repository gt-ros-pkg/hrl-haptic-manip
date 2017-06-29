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

float64[] contact_cost # Contact cost value between 0 and 100.  Current options: 'rigid' is 100, 'moveable'and 'compliant' are below than 100.
