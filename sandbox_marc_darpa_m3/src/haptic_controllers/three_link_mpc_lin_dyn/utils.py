import numpy as np

def I_to_matrix(I):
    return np.matrix([[I[0], -I[3], -I[5]],[-I[3], I[1], -I[4]],[-I[5], -I[4], I[2]]])


def matrix_to_I(mat):
    return [mat[0,0], -mat[0,1], -mat[0,2], mat[1,1], -mat[1,2], mat[2,2]]


def skew(d):
    return np.matrix([[0., -d[2], d[1]], [d[2], 0., -d[0]], [-d[1], d[0], 0.]])
