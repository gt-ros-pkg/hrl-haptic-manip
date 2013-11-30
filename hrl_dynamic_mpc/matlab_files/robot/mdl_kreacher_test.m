%MDL_KREACHER Create model of a 4 link mechanism
%
%      mdl_kreacher_test
%
% Script creates the workspace variable tl which describes the 
% kinematic and dynamic characteristics of a simple planar 4-link mechanism.
%
% Also defines the vector:
%   qz   corresponds to the zero joint angle configuration.
%
% Notes::
% - It is a planar mechanism operating in the XY (horizontal) plane and is 
%   therefore not affected by gravity.
%
% References::
%  - Based on 1 link simulated robot to test dynamic model
%

a1 = 0.1349;

%   theta d a alpha
L(1) = Link([ 0     0   a1  0], 'standard');
L(2) = Link([ 0     0   a1  0], 'standard');
L(3) = Link([ 0     0   a1  0], 'standard');
L(4) = Link([ 0     0   0.165 0], 'standard');

L(1).m = 0.416;
L(1).r = [0.078-a1 0. 0.];
L(1).I = [0., 0.0, 0.0;
 	      0.0, 0.0, 0.0;
 	      0.0, 0.0, 0.00287];
L(1).G = 0;
L(1).Jm = 0;
L(1).B = 0;

L(2).m = 0.416;
L(2).r = [0.078-a1 0. 0.];
L(2).I = [0., 0.0, 0.0;
 	      0.0, 0.0, 0.0;
 	      0.0, 0.0, 0.00287];
L(2).G = 0;
L(2).Jm = 0;
L(2).B = 0;

L(3).m = 0.416;
L(3).r = [0.078-a1 0. 0.];
L(3).I = [0., 0.0, 0.0;
 	      0.0, 0.0, 0.0;
 	      0.0, 0.0, 0.00287];
L(3).G = 0;
L(3).Jm = 0;
L(3).B = 0;

L(4).m = 0.208;
L(4).r = [-0.165/2. 0. 0.];
L(4).I = [0., 0.0, 0.0;
 	      0.0, 0.0, 0.0;
 	      0.0, 0.0, 0.001];
L(4).G = 0;
L(4).Jm = 0;
L(4).B = 0;


% L(2).m = 2.3;
% L(2).r = [-0.334 0 0];
% L(2).I = [0.00025582627118644067, 0.0, 0.0;
% 	   0.0, 0.024175454849340867, 0.0;
% 	   0.0, 0.0, 0.024175454849340867];
% L(2).G = 0;
% L(2).Jm = 0;
% L(2).B = 0;
% L(3).m = 1.32;
% L(3).r = [-0.144 0 0];
% L(3).I = [0.00014657142857142855, 0.0, 0.0;
% 	   0.0, 0.010524754285714285, 0.0;
% 	   0.0, 0.0, 0.010524754285714285]
% L(3).G = 0;
% L(3).Jm = 0;
% L(3).B = 0;

kreacher = SerialLink(L, 'name', 'kreacher', ...
		       'comment', 'full 4 dof planar kreacher');
qz = [0 0 0 0];
qn = [pi/6 pi/6 pi/6 pi/6];
