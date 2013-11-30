from math import sin, cos
import numpy as np



def tau( parms, q, dq, ddq ) :

    tau_out = [0]*3

    cse2 = 0.196*ddq[0]
    x0 = 0.196*dq[0]
    x1 = sin(q[1])
    x2 = x0*x1
    x3 = -x2
    x5 = cos(q[1])
    cse1 = 0.334*((x1)*(x1)) + 0.334*((x5)*(x5))
    x9 = cse1*ddq[0] + cse2*x5 + 0.334*ddq[1] + dq[1]*x3
    x10 = ddq[0] + ddq[1]
    x11 = dq[0] + dq[1]
    cse3 = -x11
    x12 = -cse3*parms[17] + parms[19]*x3
    x14 = cos(q[2])
    x15 = sin(q[2])
    x16 = -x15
    x17 = x0*x5
    x18 = cse2*x1 - 0.334*dq[0]*dq[1] + dq[1]*(cse1*dq[0] + x17)
    cse0 = 0.288*((x14)*(x14)) + 0.288*((x15)*(x15))
    x22 = cse1*dq[0] + 0.334*dq[1] + x17
    x23 = x14*x2 + x15*x22
    x24 = -x23
    x25 = cse0*x10 + 0.288*ddq[2] + dq[2]*x24 + x14*x9 + x16*x18
    x26 = ddq[2] + x10
    x27 = dq[2] + x11
    x28 = parms[27]*x27 + parms[29]*x24
    x29 = -x27
    x30 = parms[26]*x26 + parms[29]*x25 + x28*x29
    x31 = -parms[27]
    x32 = x14*x22 + x15*x3
    x33 = -cse0*cse3 + 0.288*dq[2] + x32
    x34 = parms[26]*x27 + parms[29]*x33
    x35 = 0.288*cse3*dq[2] + dq[2]*(-cse0*cse3 + x32) + x14*x18 + x15*x9
    x36 = parms[29]*x35 + x26*x31 + x29*x34
    x37 = cse3*x12 + parms[16]*x10 + parms[19]*x9 + x14*x30 + x15*x36
    x38 = -cse3*parms[16] + parms[19]*x22
    x39 = parms[25]*x26 + parms[26]*x25 + x23*x34 + x28*x33 + x31*x35
    x40 = -parms[17]
    x41 = parms[15]*x10 + cse0*x30 + parms[16]*x9 + x12*x22 + x18*x40 + x2*x38 + x39

    tau_out[0] = parms[5]*ddq[0] + cse1*x37 + 2.0*cse2*parms[6] + 0.196*cse2*parms[9] - 0.196*((dq[0])*(dq[0]))*parms[7] + dq[0]*parms[7]*x0 + 0.196*x1*(cse3*x38 + parms[19]*x18 + x10*x40 + x14*x36 + x16*x30) + 0.196*x37*x5 + x41
    tau_out[1] = 0.334*x37 + x41
    tau_out[2] = 0.288*x30 + x39

    return tau_out





def regressor( q, dq, ddq ) :

    regressor_out = [0]*90

    x0 = 0.196*ddq[0]
    x1 = 0.196*dq[0]
    x2 = ddq[0] + ddq[1]
    x3 = sin(q[1])
    cse2 = 0.196*x3
    x4 = x1*x3
    x5 = -x4
    x7 = cos(q[1])
    cse3 = 0.196*x7
    cse1 = 0.334*((x3)*(x3)) + 0.334*((x7)*(x7))
    x10 = cse1*ddq[0] + 0.334*ddq[1] + dq[1]*x5 + x0*x7
    x11 = dq[0] + dq[1]
    x12 = x10 + x11*x4
    x13 = -((x11)*(x11))
    x14 = x1*x7
    x15 = -0.334*dq[0]*dq[1] + dq[1]*(cse1*dq[0] + x14) + x0*x3
    x16 = -x15
    x17 = cse1*dq[0] + 0.334*dq[1] + x14
    x18 = x11*x17
    x19 = x16 + x18
    x20 = ddq[2] + x2
    x21 = cos(q[2])
    x22 = dq[2] + x11
    x23 = -((x22)*(x22))
    x24 = sin(q[2])
    x25 = x23*x24
    x26 = x20*x21 + x25
    x27 = x21*x23
    cse0 = 0.288*((x21)*(x21)) + 0.288*((x24)*(x24))
    x31 = x17*x24 + x21*x4
    x32 = cse0*x2 + 0.288*ddq[2] - dq[2]*x31 + x10*x21 + x16*x24 + x22*x31
    x33 = cse0*x20 + x32
    x34 = -x20
    x35 = x24*x34 + x27
    x36 = x17*x21 + x24*x5
    x37 = x22*(cse0*x11 + 0.288*dq[2] + x36)
    x38 = -0.288*dq[2]*x11 + dq[2]*(cse0*x11 + x36) + x10*x24 + x15*x21
    x39 = x37 - x38
    x40 = cse0*x23 + x39
    x41 = -x37 + x38
    x42 = x21*x32 + x24*x41
    x43 = cse0*x32

    regressor_out[0] = 0
    regressor_out[1] = 0
    regressor_out[2] = 0
    regressor_out[3] = 0
    regressor_out[4] = 0
    regressor_out[5] = ddq[0]
    regressor_out[6] = 2.0*x0
    regressor_out[7] = dq[0]*(-0.196*dq[0] + x1)
    regressor_out[8] = 0
    regressor_out[9] = 0.196*x0
    regressor_out[10] = 0
    regressor_out[11] = 0
    regressor_out[12] = 0
    regressor_out[13] = 0
    regressor_out[14] = 0
    regressor_out[15] = x2
    regressor_out[16] = cse1*x2 + cse2*x13 + cse3*x2 + x12
    regressor_out[17] = cse1*x13 + cse3*x13 + x19 - 0.196*x2*x3
    regressor_out[18] = 0
    regressor_out[19] = cse1*x12 + cse2*(x15 - x18) + cse3*x12
    regressor_out[20] = 0
    regressor_out[21] = 0
    regressor_out[22] = 0
    regressor_out[23] = 0
    regressor_out[24] = 0
    regressor_out[25] = x20
    regressor_out[26] = cse1*x26 + cse3*x26 - 0.196*x3*(x20*x24 - x27) + x33
    regressor_out[27] = cse1*x35 + cse2*(x21*x34 - x25) + cse3*x35 + x40
    regressor_out[28] = 0
    regressor_out[29] = cse1*x42 + cse2*(x21*x41 - x24*x32) + cse3*x42 + x43
    regressor_out[30] = 0
    regressor_out[31] = 0
    regressor_out[32] = 0
    regressor_out[33] = 0
    regressor_out[34] = 0
    regressor_out[35] = 0
    regressor_out[36] = 0
    regressor_out[37] = 0
    regressor_out[38] = 0
    regressor_out[39] = 0
    regressor_out[40] = 0
    regressor_out[41] = 0
    regressor_out[42] = 0
    regressor_out[43] = 0
    regressor_out[44] = 0
    regressor_out[45] = x2
    regressor_out[46] = x12 + 0.334*x2
    regressor_out[47] = 0.334*x13 + x19
    regressor_out[48] = 0
    regressor_out[49] = 0.334*x12
    regressor_out[50] = 0
    regressor_out[51] = 0
    regressor_out[52] = 0
    regressor_out[53] = 0
    regressor_out[54] = 0
    regressor_out[55] = x20
    regressor_out[56] = 0.334*x26 + x33
    regressor_out[57] = 0.334*x35 + x40
    regressor_out[58] = 0
    regressor_out[59] = 0.334*x42 + x43
    regressor_out[60] = 0
    regressor_out[61] = 0
    regressor_out[62] = 0
    regressor_out[63] = 0
    regressor_out[64] = 0
    regressor_out[65] = 0
    regressor_out[66] = 0
    regressor_out[67] = 0
    regressor_out[68] = 0
    regressor_out[69] = 0
    regressor_out[70] = 0
    regressor_out[71] = 0
    regressor_out[72] = 0
    regressor_out[73] = 0
    regressor_out[74] = 0
    regressor_out[75] = 0
    regressor_out[76] = 0
    regressor_out[77] = 0
    regressor_out[78] = 0
    regressor_out[79] = 0
    regressor_out[80] = 0
    regressor_out[81] = 0
    regressor_out[82] = 0
    regressor_out[83] = 0
    regressor_out[84] = 0
    regressor_out[85] = x20
    regressor_out[86] = 0.288*x20 + x32
    regressor_out[87] = 0.288*x23 + x39
    regressor_out[88] = 0
    regressor_out[89] = 0.288*x32

    return regressor_out





def M( parms, q ) :

    M_out = [0]*9

    x0 = sin(q[1])
    cse0 = 0.196*x0
    x1 = sin(q[2])
    cse1 = 0.334*x1
    x2 = cos(q[1])
    cse2 = 0.196*x2
    x3 = ((x2)*(x2))
    x4 = ((x0)*(x0))
    x5 = cse2 + 0.334*x3 + 0.334*x4
    x7 = cos(q[2])
    cse3 = 0.334*x7
    x8 = cse0*x7 + x1*x5
    x9 = -parms[27]
    x10 = parms[29]*x8 + x9
    x11 = -parms[17]
    x12 = -x1
    x13 = ((x7)*(x7))
    x14 = ((x1)*(x1))
    x15 = 0.288*x13 + 0.288*x14
    x16 = cse0*x12 + x15 + x5*x7
    x17 = parms[26] + parms[29]*x16
    x18 = parms[16] + parms[19]*x5 + x1*x10 + x17*x7
    x19 = 0.288*x17
    x20 = x13 + x14
    x21 = parms[25] + parms[26]*x16 + x8*x9
    x22 = parms[15] + cse0*x11 + parms[16]*x5 + x19*x20 + x21
    x23 = 0.334*x18
    x24 = x22 + x23
    x25 = x19 + x21
    x27 = cse3 + x15
    x28 = parms[25] + cse1*x9 + parms[26]*x27
    x29 = parms[26] + parms[29]*x27
    x30 = 0.288*x29
    x31 = x28 + x30

    M_out[0] = parms[5] + cse0*(cse0*parms[19] + x10*x7 + x11 + x12*x17) + cse2*x18 + 0.392*parms[6] + 0.038416*parms[9] + x22 + x23*(x3 + x4)
    M_out[1] = x24
    M_out[2] = x25
    M_out[3] = x24
    M_out[4] = parms[15] + cse1*(cse1*parms[29] + x9) + cse3*x29 + 0.668*parms[16] + 0.111556*parms[19] + x20*x30 + x28
    M_out[5] = x31
    M_out[6] = x25
    M_out[7] = x31
    M_out[8] = parms[25] + 0.576*parms[26] + 0.082944*parms[29]

    return M_out





def c( parms, q, dq ) :

    c_out = [0]*3

    cse1 = 0.334*dq[0]
    x0 = dq[0] + dq[1]
    cse2 = -x0
    x1 = 0.196*dq[0]
    x2 = sin(q[1])
    x3 = x1*x2
    x4 = -x3
    x5 = -cse2*parms[17] + parms[19]*x4
    x7 = dq[1]*x4
    x8 = cos(q[2])
    x9 = dq[2] + x0
    x10 = cos(q[1])
    x11 = x1*x10
    cse0 = ((x10)*(x10)) + ((x2)*(x2))
    x15 = cse0*cse1 + 0.334*dq[1] + x11
    x16 = sin(q[2])
    x17 = x15*x16 + x3*x8
    x18 = -x17
    x19 = parms[27]*x9 + parms[29]*x18
    x20 = -x9
    x21 = dq[1]*(cse0*cse1 - 0.334*dq[0] + x11)
    x22 = -x16
    x23 = dq[2]*x18 + x21*x22 + x7*x8
    x24 = parms[29]*x23 + x19*x20
    x25 = x15*x8 + x16*x4
    x26 = ((x8)*(x8))
    x27 = ((x16)*(x16))
    x28 = 0.288*cse2*dq[2] + dq[2]*(-cse2*(0.288*x26 + 0.288*x27) + x25) + x16*x7 + x21*x8
    x29 = x26 + x27
    x30 = -0.288*cse2*x29 + 0.288*dq[2] + x25
    x31 = parms[26]*x9 + parms[29]*x30
    x32 = parms[29]*x28 + x20*x31
    x33 = cse2*x5 + parms[19]*x7 + x16*x32 + x24*x8
    x34 = 0.334*x33
    x35 = parms[26]*x23 - parms[27]*x28 + x17*x31 + x19*x30
    x36 = 0.288*x24
    x37 = -cse2*parms[16] + parms[19]*x15
    x38 = parms[16]*x7 - parms[17]*x21 + x15*x5 + x29*x36 + x3*x37 + x35

    c_out[0] = cse0*x34 - 0.196*((dq[0])*(dq[0]))*parms[7] + dq[0]*parms[7]*x1 + 0.196*x10*x33 + 0.196*x2*(cse2*x37 + parms[19]*x21 + x22*x24 + x32*x8) + x38
    c_out[1] = x34 + x38
    c_out[2] = x35 + x36

    return c_out





def g( parms, q ) :

    g_out = [0]*3


    g_out[0] = 0
    g_out[1] = 0
    g_out[2] = 0

    return g_out









from coriolis import *




#dynparms = [L_1xx, L_1xy, L_1xz, L_1yy, L_1yz, L_1zz, l_1x, l_1y, l_1z, m_1, L_2xx, L_2xy, L_2xz, L_2yy, L_2yz, L_2zz, l_2x, l_2y, l_2z, m_2, L_3xx, L_3xy, L_3xz, L_3yy, L_3yz, L_3zz, l_3x, l_3y, l_3z, m_3]
