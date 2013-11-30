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

percentage = 0.7

L(1).m = 11.34/4.0*percentage;
L(1).r = [-0.196/2.0 0 0];
L(1).I = [0.00031303125, 0.0, 0.0;
	   0.0, 0.011214214375, 0.0;
	   0.0, 0.0, 0.011214214375];
L(1).G = 0;
L(1).Jm = 0;
L(1).B = 0;
L(2).m = 2.3*percentage;
L(2).r = [-0.334/2.0 0 0];
L(2).I = [0.00025582627118644067, 0.0, 0.0;
	   0.0, 0.024175454849340867, 0.0;
	   0.0, 0.0, 0.024175454849340867];
L(2).G = 0;
L(2).Jm = 0;
L(2).B = 0;
L(3).m = 1.32*percentage;
L(3).r = [-0.288/2.0 0 0];
L(3).I = [0.00014657142857142855, 0.0, 0.0;
	   0.0, 0.010524754285714285, 0.0;
	   0.0, 0.0, 0.010524754285714285]
L(3).G = 0;
L(3).Jm = 0;
L(3).B = 0;

threelink = SerialLink(L, 'name', 'three link', ...
		       'comment', 'from Spong, Hutchinson, Vidyasagar');
qz = [0 0 0];
qn = [pi/6, -pi/6, pi/4];
