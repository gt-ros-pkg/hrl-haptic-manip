import numpy as np
import dMdq_func


def c_mat(parms, q, dq):

    dof = len(q)
    dMdq_ls = []

    for i in xrange(dof):
        dMdq_ls.append(dMdq_func.dMdq(parms, q, i))

    coriolis = [[sum([1/2.*(dMdq_ls[k][dof*i+j] + dMdq_ls[j][dof*i+k] - dMdq_ls[i][dof*j+k])*dq[k] for k in xrange(dof) ]) for j in xrange(dof)] for i in xrange(dof)]

    return coriolis

