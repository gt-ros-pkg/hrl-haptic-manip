import numpy as np
    
# http://en.wikipedia.org/wiki/Dodecahedron
# http://en.wikipedia.org/wiki/Icosahedron 
# http://www.cs.umbc.edu/~squire/reference/polyhedra.shtml

cube_faces = np.matrix([[ 1.0,  0.0,  0.0],
                        [-1.0,  0.0,  0.0],
                        [ 0.0,  1.0,  0.0],
                        [ 0.0, -1.0,  0.0],
                        [ 0.0,  0.0,  1.0],
                        [ 0.0,  0.0, -1.0]])

octahedron_faces = np.matrix([[ 1.0,  1.0,  1.0],
                              [ 1.0,  1.0, -1.0],
                              [ 1.0, -1.0,  1.0],
                              [ 1.0, -1.0, -1.0],
                              [-1.0,  1.0,  1.0],
                              [-1.0,  1.0, -1.0],
                              [-1.0, -1.0,  1.0],
                              [-1.0, -1.0, -1.0]])

v_length = np.linalg.norm(octahedron_faces[0, :])
octahedron_faces = octahedron_faces/v_length

# 12 constraint vectors, one for each flat face, should
# define a regular dodecahedron which is a regular
# polytope in 3D.  We want to define it such that it
# inscribes a sphere of the desired limit.
#
# Since we actually want the vectors that go to the center
# of the faces. We can use the vectors that define the
# vertices of an icosahedron, which is the dual of the
# dodecahedron and has 12 vertices (20 faces. From
# Wikipedia:
#
# The following Cartesian coordinates define the 12
# vertices of an icosahedron with edge-length 2, centered
# at the origin:
# (   0, +/-1, +/-p)
# (+/-1, +/-p,    0)
# (+/-p,    0, +/-1)
# where p = (1+sqrt(5))/2 is the golden ratio.

gld = (1.0 + np.sqrt(5.0))/2.0

dodecahedron_faces = np.matrix([[ 0.0,  1.0,  gld],
                                [ 0.0,  1.0, -gld],
                                [ 0.0, -1.0,  gld],
                                [ 0.0, -1.0, -gld],
                                [ 1.0,  gld,  0.0],
                                [ 1.0, -gld,  0.0],
                                [-1.0,  gld,  0.0],
                                [-1.0, -gld,  0.0],
                                [ 1.0,  0.0,  gld],
                                [ 1.0,  0.0, -gld],
                                [-1.0,  0.0,  gld],
                                [-1.0,  0.0, -gld]])

v_length = np.linalg.norm(dodecahedron_faces[0, :])
dodecahedron_faces = dodecahedron_faces/v_length

# We could instead use the 20 vertices of a dodecahedron, since they
# would correspond to the 20 faces of an icosahedron search volume.
#
#The following Cartesian coordinates define the vertices of a
#dodecahedron centered at the origin:
#
#(        +/-1,        +/-1,        +/-1)
#(           0,  +/-(1/gld),      +/-gld)
#(  +/-(1/gld),      +/-gld,           0)
#(      +/-gld,           0,  +/-(1/gld))
#
# where gld = (1+sqrt(5))/2 is the golden ratio

icosahedron_faces = np.matrix([[      1.0,      1.0,     1.0],
                               [      1.0,      1.0,    -1.0],
                               [      1.0,     -1.0,     1.0],
                               [      1.0,     -1.0,    -1.0],
                               [     -1.0,      1.0,     1.0],
                               [     -1.0,      1.0,    -1.0],
                               [     -1.0,     -1.0,     1.0],
                               [     -1.0,     -1.0,    -1.0],
                               [      0.0,  1.0/gld,     gld],
                               [      0.0,  1.0/gld,    -gld],
                               [      0.0, -1.0/gld,     gld],
                               [      0.0, -1.0/gld,    -gld],
                               [  1.0/gld,      gld,      0.0],
                               [  1.0/gld,     -gld,      0.0],
                               [ -1.0/gld,      gld,      0.0],
                               [ -1.0/gld,     -gld,      0.0],
                               [      gld,      0.0,  1.0/gld],
                               [      gld,      0.0, -1.0/gld],
                               [     -gld,      0.0,  1.0/gld],
                               [     -gld,      0.0, -1.0/gld]])
                               
v_length = np.linalg.norm(icosahedron_faces[0, :])
icosahedron_faces = icosahedron_faces/v_length

