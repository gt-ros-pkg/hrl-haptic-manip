#
#
# Copyright (c) 2013, Georgia Tech Research Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# \authors: Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)
# \adviser: Charles Kemp (Healthcare Robotics Lab, Georgia Tech.)

from dynamics_utils import *
from darci_left_dynamics import *
from math import pi

# # m1 = 0.
# # m2 = 0.
# # m3 = 0.
# # m4 = 0.
# # m5 = 0.
# # m6 = 0.
# # m7 = 0.

m1 = 1.9872
m2 = 0.5575
m3 = 2.220
m4 = 0.22
m5 = 1.7
m6 = 0.212
m7 = 0.084
#m8 = 0.838


r1 = [0, -0.0094, 0.0240]  #[2.088e-05, -0.00936788, -0.02395458]
r2 = [-0.0264, -0.0431, -0.0008]  #[0.02635544, -0.04306494, -0.00080377]
r3 = [0.0080, 0., -0.0877]  #[0.00802582, 1.816e-05, -0.08769598]
r4 = [0.0, 0.02573092, 0.00080947]
r5 = [0.00193729, 0.00046171, -0.13853286]
r6 = [8.7719999999999994e-05, -0.0018379500000000001, -0.0018201000000000001]
r7 = [3.0e-05, -0.01092, 0.00292]
#r8 = [0.00286004, 0.00206754, 0.06242752]

#r1 = [0., 0., 0]
#r2 = [0.0]*3
#r3 = [0]*3
# r4 = [0]*3
# r5 = [0]*3
# r6 = [0]*3
# r7 = [0]*3

#

# %        Ixx     Iyy      Izz    Ixy     Iyz     Ixz
# I1 = [0, 0, 0.0, 0, 0, 0]
# I2 = [0, 0, 0.0, 0, 0, 0]
# I3 = [0, 0, 0.0, 0, 0, 0]
# I4 = [0, 0, 0.0, 0, 0, 0]
# I5 = [0, 0, 0.0, 0, 0, 0]
# I6 = [0, 0, 0.0, 0, 0, 0]
# I7 = [0, 0, 0.0, 0, 0, 0]


I1 = [0.00681123,  0.004616, 0.003267,  2.79e-06,   0.0001518,  -9.60e-07]
I2 = [0.00195358,  0.00121948, 0.0021736, -0.00077554, 4.74e-06, 4.39e-06]
I3 = [0.02949661, 0.02983326, 0.00244235, 8.636e-05, -0.00013024, -0.0024288]
I4 = [0.00062344, 0.00042457, 0.00038623, 2.0e-08, 1.768e-05, 0.0]
I5 = [0.03130809, 0.03135827, 0.00120798, -2.89e-06, -3.896e-05, -0.00089153]
I6 = [0.00011597, 0.00011378, 7.497e-05, -5.0e-08, -1.2e-07, 4.0e-08]
I7 = [9.428e-05, 6.133e-05, 5.054e-05, 2.0e-08, -7.4e-07, 0.0]
#I8 = [0.00484971, 0.00507558, 0.00080672, -3.507e-05, 9.2089e-05, 7.626e-05]

L1 = I_to_matrix(I1)-m1*skew(r1)*skew(r1)
L2 = I_to_matrix(I2)-m2*skew(r2)*skew(r2)
L3 = I_to_matrix(I3)-m3*skew(r3)*skew(r3)
L4 = I_to_matrix(I4)-m4*skew(r4)*skew(r4)
L5 = I_to_matrix(I5)-m5*skew(r5)*skew(r5)
L6 = I_to_matrix(I6)-m6*skew(r6)*skew(r6)
L7 = I_to_matrix(I7)-m7*skew(r7)*skew(r7)
#L8 = I_to_matrix(I8)-m4*skew(r8)*skew(r8)


params = [L1[0,0], -L1[0,1], -L1[0,2], L1[1,1], -L1[1,2], L1[2,2], r1[0]*m1, r1[1]*m1, r1[2]*m1, m1, 0., 0.,
          L2[0,0], -L2[0,1], -L2[0,2], L2[1,1], -L2[1,2], L2[2,2], r2[0]*m2, r2[1]*m2, r2[2]*m2, m2, 0., 0.,
          L3[0,0], -L3[0,1], -L3[0,2], L3[1,1], -L3[1,2], L3[2,2], r3[0]*m3, r3[1]*m3, r3[2]*m3, m3, 0., 0.,
          L4[0,0], -L4[0,1], -L4[0,2], L4[1,1], -L4[1,2], L4[2,2], r4[0]*m4, r4[1]*m4, r4[2]*m4, m4, 0., 0.,
          L5[0,0], -L5[0,1], -L5[0,2], L5[1,1], -L5[1,2], L5[2,2], r5[0]*m5, r5[1]*m5, r5[2]*m5, m5, 0., 0.,
          L6[0,0], -L6[0,1], -L6[0,2], L6[1,1], -L6[1,2], L6[2,2], r6[0]*m6, r6[1]*m6, r6[2]*m6, m6, 0., 0.,
          L7[0,0], -L7[0,1], -L7[0,2], L7[1,1], -L7[1,2], L7[2,2], r7[0]*m7, r7[1]*m7, r7[2]*m7, m7, 0., 0.]
#         L8[0,0], -L8[0,1], -L8[0,2], L8[1,1], -L8[1,2], L8[2,2], r8[0]*m8, r8[1]*m8, r8[2]*m8, m8, 0., 0.]




# params = [0.00813144619200000, 
#           2.79000000000000e-06,
#           -9.60000000000000e-07,
#           0.00576062720000000,
#           0.000600112320000000,
#           2.00344258899200,
#           0,
#           -0.0186796800000000,
#           -0.0476928000000000,
#           1.98720000000000,
#           0,
#           0,
#           3.00298955437500,
#           1.49859011420000,
#           -1.00000738440000,
#           -1.49839160800000,
#           1.00002396260000,
#           4.50359777277500,
#           0.0147180000000000,
#           -0.0240282500000000,
#           -0.000446000000000000,
#           0.557500000000000,
#           0,
#           0,
#           1.04657127380000,
#           8.63600000000000e-05,
#           -0.00398635200000000,
#           0.0470500038000000,
#           -0.000130240000000000,
#           2.00258443000000,
#           2.51776000000000,
#           11,
#           -0.194694000000000,
#           18.2200000000000,
#           0,
#           0,
#           0.000769241806860000,
#           2.00000000000000e-08,
#           0.500000000000000,
#           0.000424714153170000,
#           1.30977502813000e-05,
#           2.50053188765369,
#           -1.50000000000000,
#           -0.994339197600000,
#           9.99982191660000,
#           0.219999999999999,
#           0,
#           0,
#           0.0639337530090000,
#           -1.36940751797000e-06,
#           -0.00134777315139000,
#           0.0639899508670000,
#           -0.000147695411544000,
#           1.00121472265674,
#           0.503293393000000,
#           1.50078490700000,
#           3.76449413800000,
#           -20.3000000000000,
#           0,
#           0,
#           0.000117388454733000,
#           -8.41796944880000e-08,
#           6.15225553600000e-09,
#           0.000114483937267000,
#           5.89193592540000e-07,
#           1.00007568778006,
#           1.85966400000000e-05,
#           -0.000389645400000000,
#           0.999614138800000,
#           0.212000000000000,
#           0,
#           0,
#           0.000105012915200000,
#           -7.51840000000000e-09,
#           7.35840000000000e-09,
#           6.20462932000000e-05,
#           -3.41845760000000e-06,
#           6.05567732000000e-05,
#           2.52000000000000e-06,
#           -0.000917280000000000,
#           0.000245280000000000,
#           0.0840000000000000,
#           0,
#           0]


# params = [0.0068162330380577181,
#           0.00074668216894845335,
#           -8.1091729460405966e-05,
#           0.0086409408,
#           0.00041274813569905713,
#           0.0036378725182511667,
#           0.00013402380401816418,
#           -0.023227657890460128,
#           -0.063513809818296019,
#           1.0608702727591373,
#           -0.00031061125910230355,
#           -0.00074501042036397523,
#           0.0039626826601765579,
#           -0.0021148287000000003,
#           0.001,
#           0.0014394376854486167,
#           0.00033915809407014105,
#           0.003300190483991773,
#           0.022077,
#           -0.012541985951837793,
#           -0.00065428944449951603,
#           0.66672954143513086,
#           -0.00055447913088879479,
#           0.00033233597774516804,
#           0.060168070331395269,
#           -0.001,
#           -0.0043151377887145469,
#           0.027830157714098202,
#           -0.00017369845860442575,
#           0.0033217551342830964,
#           0.017238264572578831,
#           0.001,
#           -0.13544794943264266,
#           2.8099480910537924,
#           -6.865194312474386e-05,
#           0.00076178418441210109,
#           0.001033447564389513,
#           -0.0002436939893720673,
#           0.00014067670186764558,
#           0.000637071229755,
#           -0.00063057971385915206,
#           0.00054739147219960086,
#           -0.001,
#           0.0051052586052059215,
#           -0.00011719248194136056,
#           0.19430597198873939,
#           0.001,
#           0.00014248155730415982,
#           0.088004959132000099,
#           0.00090546746939283284,
#           -0.0018522597798119561,
#           0.090974920783641763,
#           -0.0002096372846517263,
#           0.0012358727496404691,
#           0.0026241950643258161,
#           0.0008248085953042047,
#           -0.14592651620289268,
#           2.55,
#           -0.00070780574756992983,
#           -0.00090843135436767075,
#           0.00014054192140454513,
#           -0.001,
#           0.00038930833013966267,
#           9.0776381270780003e-05,
#           -0.00033899046778744859,
#           0.0006582469826628707,
#           0.00080438552516085547,
#           -0.00028939502641969241,
#           -0.00020305231959386616,
#           0.16993449170648317,
#           -0.00031304747672090125,
#           -0.00055846234732002465,
#           0.00011230375554546601,
#           0.00041518605890675844,
#           0.00010583423655039864,
#           0.00029788778537683674,
#           0.00062623970165317144,
#           0.00059800258631844067,
#           -0.00041769064981410917,
#           -0.001127198505072027,
#           0.0003456402776569888,
#           0.078708561072762626,
#           0.00092668567298268576,
#           0.001]



# print "length of params: ", len(params)

# print np.where(np.array(params) != 0)

# print np.where(np.array(params) == 0)


# print "length is :", len(np.where(np.array(params) != 0)[0])
# print "length is :", len(np.where(np.array(params) == 0)[0])

# for parm in params:
#     print parm, ","

# print "params are :\n", 
# for i in xrange(len(params)):
#     print "for index "+str(i)+" :", params[i]

# np.set_printoptions(suppress=True)
# np.set_printoptions(precision=5)

# # qz = [0]*8
# # qn =  [0, pi/2, 0, 0, 0, 0, 0, 0]

# qz = [0]*7
# qn =  [0, pi/2, 0, 0, 0, 0, 0]
# q1 = [0.1]*7
# q2 = [0.3]*7
# #q_0 = [-1.0180830044852893, 1.1561538739460362, 0.6302994150119539, 0.5696089229412958]


# #qn = get this from matlab output.txt file
# #qr = 
# qd_0 = [0.0]*8
# qd_1 = [0.1]*8
# # params = [0.0, 0.0, 0.0, 0.0, 0.0, 0.040429892533237866, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.02632313894809937, 0.19616785648240612, -0.018129439683029904, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.015005065455025102, 0.09284360520374914, -0.013373003155801729, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.011661573324999053, 0.03332035575288851, -0.00039455771471209877, 0.0, 0]

# # print 'M for qz = \n', np.matrix(M(params, qz)).reshape(8,8), '\n'
# # print 'M for qn = \n', np.matrix(M(params, qn)).reshape(8,8), '\n'

# # print np.linalg.eig(np.matrix(M(params, qz)).reshape(8,8))

# # print "condition is :", np.linalg.cond(np.matrix(M(params, qz)).reshape(8,8)[0:7,0:7])



# print 'M for qz = \n', (np.matrix(M(params, qz)).reshape(7,7)).tolist(), '\n'
# print "inv for M :\n", np.linalg.inv(np.matrix(M(params, qz)).reshape(7,7)).tolist(), '\n'


# print 'M for qn = \n', (np.matrix(M(params, qn)).reshape(7,7)).tolist(), '\n'
# print 'M for q1 = \n', np.matrix(M(params, q1)).reshape(7,7), '\n'
# print 'M for q2 = \n', np.matrix(M(params, q2)).reshape(7,7), '\n'

# #print np.linalg.eig(np.matrix(M(params, qz)).reshape(7,7))
# print "condition is :", np.linalg.cond(np.matrix(M(params, qz)).reshape(7,7))

# print "gravity load at end was :\n", g(params, [ 0.0554,    0.4075,   0.4307,    1.0797,    0.2743,   -0.4502,   -0.0434])

# import scipy.io as io
# M_qz = np.matrix(M(params, qz)).reshape(7,7)
# M_qn = np.matrix(M(params, qn)).reshape(7,7)
# M_q1 = np.matrix(M(params, q1)).reshape(7,7)
# M_q2 = np.matrix(M(params, q2)).reshape(7,7)


# d = {'M_qz':M_qz, 'M_qn':M_qn, 'M_q1':M_q1, 'M_q2':M_q2}
# io.savemat('/home/mkillpack/Dropbox/rvctools/compare_sympy.mat', d)
