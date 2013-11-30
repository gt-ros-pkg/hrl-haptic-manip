%MDL_THREELINK Create model of a simple 3-link mechanism
%
%      mdl_threelink
%
% Script creates the workspace variable tl which describes the 
% kinematic and dynamic characteristics of a simple planar 3-link mechanism.
%
% Also defines the vector:
%   qz   corresponds to the zero joint angle configuration.
%
% Notes::
% - It is a planar mechanism operating in the XY (horizontal) plane and is 
%   therefore not affected by gravity.
%
% References::
%  - Based on 3 link simulated robot in our IJRR paper
%

a1 = 0.196;
a2 = 0.334;
a3 = 0.288;
%   theta d a alpha

L(1) = Link([ 0     0   a1  0], 'standard');
L(2) = Link([ 0     0   a2  0], 'standard');
L(3) = Link([ 0     0   a3  0], 'standard');

L(1).m = 11.34/4.0;
L(1).r = [-0.196/2.0 0 0];
L(1).I = [4.779405042171985e-08, 0.0, 0.0;
	   0.0, 1.712201983919259e-06, 0.0;
	   0.0, 0.0, 1.712201983919259e-06];
L(1).G = 0;
L(1).Jm = 0;
L(1).B = 0;
L(2).m = 2.3;
L(2).r = [-0.334/2.0 0 0];
L(2).I = [6.401485905541325e-08, 0.0, 0.0;
	   0.0, 6.049372207177322e-06, 0.0;
	   0.0, 0.0, 6.049372207177322e-06];
L(2).G = 0;
L(2).Jm = 0;
L(2).B = 0;
L(3).m = 1.32;
L(3).r = [-0.288/2.0 0 0];
L(3).I = [3.033588625150192e-08, 0.0, 0.0;
	   0.0, 1.9743854394051904e-06, 0.0;
	   0.0, 0.0, 1.9743854394051904e-06]
L(3).G = 0;
L(3).Jm = 0;
L(3).B = 0;

threelink = SerialLink(L, 'name', 'three link');
qz = [0 0 0];
qn = [pi/6, -pi/6, pi/4];
