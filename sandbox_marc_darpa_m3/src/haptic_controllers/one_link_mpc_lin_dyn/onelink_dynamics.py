from math import sin, cos 



def tau( parms, q, dq, ddq ) :

    tau_out = [0]*1

    ivar_0 = 0.334*dq[0]
    ivar_1 = 0.334*ddq[0]
    ivar_2 = parms[2]*ddq[0] - parms[4]*((dq[0])*(dq[0])) - ivar_1*parms[8] + 9.81*parms[7]
    ivar_3 = parms[4]*ddq[0] + dq[0]*(parms[2]*dq[0] - ivar_0*parms[8]) - 9.81*parms[6]
    ivar_4 = parms[5]*ddq[0] + dq[0]*ivar_0*parms[7] + ivar_1*parms[6]
    ivar_5 = -ddq[0]*parms[7] - dq[0]*(dq[0]*parms[6] + ivar_0*parms[9])
    ivar_6 = ddq[0]*parms[6] - ((dq[0])*(dq[0]))*parms[7] + ivar_1*parms[9]
    ivar_7 = 9.81*parms[9]

    tau_out[0] = ivar_4 + 0.334*ivar_6

    return tau_out









def regressor( q, dq, ddq ) :

    regressor_out = [0]*10

    ivar_0 = 0.334*dq[0]
    ivar_1 = 0.334*ddq[0]
    ivar_2 = ((dq[0])*(dq[0]))
    ivar_3 = -((dq[0])*(dq[0]))
    ivar_4 = -((dq[0])*(dq[0]))
    ivar_5 = dq[0]*ivar_0
    ivar_6 = -((dq[0])*(dq[0]))
    ivar_7 = -dq[0]*ivar_0
    ivar_8 = -dq[0]*ivar_0

    regressor_out[0] = 0
    regressor_out[1] = 0
    regressor_out[2] = 0
    regressor_out[3] = 0
    regressor_out[4] = 0
    regressor_out[5] = ddq[0]
    regressor_out[6] = 0.334*ddq[0] + ivar_1
    regressor_out[7] = ivar_5 + 0.334*ivar_6
    regressor_out[8] = 0
    regressor_out[9] = 0.334*ivar_1

    return regressor_out









def M( parms, q ) :

    M_out = [0]*1

    ivar_0 = parms[2] - 0.334*parms[8]
    ivar_1 = parms[5] + 0.334*parms[6]
    ivar_2 = parms[6] + 0.334*parms[9]

    M_out[0] = ivar_1 + 0.334*ivar_2

    return M_out









def c( parms, q, dq ) :

    c_out = [0]*1

    ivar_0 = 0.334*dq[0]
    ivar_1 = -parms[4]*((dq[0])*(dq[0]))
    ivar_2 = dq[0]*(parms[2]*dq[0] - ivar_0*parms[8])
    ivar_3 = dq[0]*ivar_0*parms[7]
    ivar_4 = -dq[0]*(dq[0]*parms[6] + ivar_0*parms[9])
    ivar_5 = -((dq[0])*(dq[0]))*parms[7]

    c_out[0] = ivar_3 + 0.334*ivar_5

    return c_out









def g( parms, q ) :

    g_out = [0]*1

    ivar_0 = 9.81*parms[7]
    ivar_1 = -9.81*parms[6]
    ivar_2 = 9.81*parms[9]

    g_out[0] = 0

    return g_out









#dynparms = [L_1xx, L_1xy, L_1xz, L_1yy, L_1yz, L_1zz, l_1x, l_1y, l_1z, m_1]
