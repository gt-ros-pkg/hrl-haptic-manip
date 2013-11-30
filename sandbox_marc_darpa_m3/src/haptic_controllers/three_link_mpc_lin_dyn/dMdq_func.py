from math import sin, cos
def dMdq(parms, q, jt_num):

    if jt_num == 0:
    
        dMdq0_out = [0]*9
    
    
        dMdq0_out[0] = 0
        dMdq0_out[1] = 0
        dMdq0_out[2] = 0
        dMdq0_out[3] = 0
        dMdq0_out[4] = 0
        dMdq0_out[5] = 0
        dMdq0_out[6] = 0
        dMdq0_out[7] = 0
        dMdq0_out[8] = 0
    
        return dMdq0_out
    
    if jt_num == 1:
    
        dMdq1_out = [0]*9
    
        cse0 = sin(q[1])
        cse1 = cos(q[1])
        cse4 = 0.196*cse0
        cse2 = 0.196*cse1
        x1 = sin(q[2])
        cse5 = 0.196*cse1
        dx2 = -cse0
        cse3 = 0.196*dx2
        x3 = ((cse1)*(cse1))
        dx3 = 2.0*cse1*dx2
        x4 = ((cse0)*(cse0))
        dx4 = 2.0*cse0*cse1
        x5 = cse5 + 0.334*x3 + 0.334*x4
        dx5 = cse3 + 0.334*dx3 + 0.334*dx4
        x7 = cos(q[2])
        dx8 = cse2*x7 + dx5*x1
        x9 = -parms[27]
        x10 = parms[29]*(cse4*x7 + x1*x5) + x9
        dx10 = dx8*parms[29]
        x11 = -parms[17]
        x12 = -x1
        x13 = ((x7)*(x7))
        x14 = ((x1)*(x1))
        dx16 = cse2*x12 + dx5*x7
        x17 = parms[26] + parms[29]*(cse4*x12 + 0.288*x13 + 0.288*x14 + x5*x7)
        dx17 = dx16*parms[29]
        x18 = parms[16] + parms[19]*x5 + x1*x10 + x17*x7
        dx18 = dx10*x1 + dx17*x7 + dx5*parms[19]
        dx19 = 0.288*dx17
        dx21 = dx16*parms[26] + dx8*x9
        dx22 = cse2*x11 + dx19*(x13 + x14) + dx21 + dx5*parms[16]
        x23 = 0.334*x18
        dx23 = 0.334*dx18
        dx24 = dx22 + dx23
        dx25 = dx19 + dx21
    
        dMdq1_out[0] = cse2*cse4*parms[19] + cse2*(cse4*parms[19] + x10*x7 + x11 + x12*x17) + cse3*x18 + cse4*dx10*x7 + cse4*dx17*x12 + cse5*dx18 + dx22 + dx23*(x3 + x4) + dx3*x23 + dx4*x23
        dMdq1_out[1] = dx24
        dMdq1_out[2] = dx25
        dMdq1_out[3] = dx24
        dMdq1_out[4] = 0
        dMdq1_out[5] = 0
        dMdq1_out[6] = dx25
        dMdq1_out[7] = 0
        dMdq1_out[8] = 0
    
        return dMdq1_out
    
    if jt_num == 2:
    
        dMdq2_out = [0]*9
    
        cse0 = sin(q[2])
        cse1 = cos(q[2])
        x0 = sin(q[1])
        cse5 = 0.196*x0
        cse6 = 0.334*cse0
        cse2 = 0.334*cse1
        x2 = cos(q[1])
        cse7 = 0.196*x2
        x3 = ((x2)*(x2))
        x4 = ((x0)*(x0))
        x5 = cse7 + 0.334*x3 + 0.334*x4
        cse8 = 0.334*cse1
        dx7 = -cse0
        cse4 = 0.334*dx7
        dx8 = cse1*x5 + cse5*dx7
        x9 = -parms[27]
        x10 = parms[29]*(cse0*x5 + cse1*cse5) + x9
        dx10 = dx8*parms[29]
        x12 = -cse0
        dx12 = -cse1
        x13 = ((cse1)*(cse1))
        dx13 = 2.0*cse1*dx7
        x14 = ((cse0)*(cse0))
        dx14 = 2.0*cse0*cse1
        x15 = 0.288*x13 + 0.288*x14
        dx15 = 0.288*dx13 + 0.288*dx14
        dx16 = cse5*dx12 + dx15 + dx7*x5
        x17 = parms[26] + parms[29]*(cse1*x5 + cse5*x12 + x15)
        dx17 = dx16*parms[29]
        dx18 = cse0*dx10 + cse1*dx17 + cse1*x10 + dx7*x17
        dx19 = 0.288*dx17
        x20 = x13 + x14
        cse3 = 0.288*dx13 + 0.288*dx14
        dx21 = dx16*parms[26] + dx8*x9
        dx22 = cse3*x17 + dx19*x20 + dx21
        dx23 = 0.334*dx18
        dx24 = dx22 + dx23
        dx25 = dx19 + dx21
        dx27 = cse4 + dx15
        dx28 = cse2*x9 + dx27*parms[26]
        x29 = parms[26] + parms[29]*(cse8 + x15)
        dx29 = dx27*parms[29]
        dx30 = 0.288*dx29
        dx31 = dx28 + dx30
    
        dMdq2_out[0] = cse1*cse5*dx10 + cse5*dx12*x17 + cse5*dx17*x12 + cse5*dx7*x10 + cse7*dx18 + dx22 + dx23*(x3 + x4)
        dMdq2_out[1] = dx24
        dMdq2_out[2] = dx25
        dMdq2_out[3] = dx24
        dMdq2_out[4] = cse2*cse6*parms[29] + cse2*(cse6*parms[29] + x9) + cse3*x29 + cse4*x29 + cse8*dx29 + dx28 + dx30*x20
        dMdq2_out[5] = dx31
        dMdq2_out[6] = dx25
        dMdq2_out[7] = dx31
        dMdq2_out[8] = 0
    
        return dMdq2_out
    
