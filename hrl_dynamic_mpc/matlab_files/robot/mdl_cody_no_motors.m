%MDL_CODY Create model of Cody manipulator from Meka
%
%      mdl_cody_no_motors

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.

clear L
%             th    d       a         alpha   sigma  offset
% L(1) = Link([ 0     0.18493 0         pi/2    0        -pi/2], 'modified');
% L(2) = Link([ 0 	0.      0	      pi/2    0         pi/2], 'modified');
% L(3) = Link([ 0     0.27795	0.03175   pi/2    0         pi/2], 'modified');
% L(4) = Link([ 0     0       -0.00635  pi/2    0         0   ], 'modified');
% L(5) = Link([ 0     0.27853 0         -pi/2   0         0   ], 'modified');
% L(6) = Link([ 0     0       0         pi/2    0         pi/2], 'modified');
% L(7) = Link([ 0     0       0         pi/2    0         pi/2], 'modified');

L(1) = Link([ 0     0.18493 0         pi/2    0         pi/2], 'standard');
L(2) = Link([ 0 	0.      0.03175   pi/2    0         pi/2], 'standard');
L(3) = Link([ 0     0.27795	0.00635  -pi/2    0        -pi/2], 'standard');
L(4) = Link([ 0     0       0         pi/2    0         0   ], 'standard');
L(5) = Link([ 0     0.27853 0        -pi/2   0         0   ], 'standard');
L(6) = Link([ 0     0       0        -pi/2    0         pi/2], 'standard');
L(7) = Link([ 0     0       -0.1         0       0         0   ], 'standard');

L(1).m = 2.00359932;
L(2).m = 0.63620172;
L(3).m = 2.30170001;
L(4).m = 0.3388;
L(5).m = 1.18399998;
L(6).m = 0.32;
L(7).m = 0.10;

L(1).r = [ 0.00810609 -0.00286468 -0.00949388 ];
L(2).r = [-0.03412920 -0.02250353 -0.00087239];
L(3).r = [0.00480613 -0.00319633 -0.08909757];
L(4).r = [0.0 0.03509681 0.00593399];
L(5).r = [0.00023599 -0.00159016 -0.12832980];
L(6).r = [0 0 0.05];
L(7).r = [0.05 0 0];

% Ixx Iyy Izz Ixy Iyz Ixz
L(1).I = [0.00465797, 0.00305583, 0.00321248, -0.00050431, -0.00026800, 0.00007789];
L(2).I = [0.00124015, 0.00128069, 0.00096573, 0.00020556, -0.00001031, -0.00001630];
L(3).I = [0.01564688, 0.01578789, 0.00233888, -0.00000074, -0.00060252, -0.00052476];
L(4).I = [0.00066328, 0.00057379, 0.00040850, 0.00000000, -0.00005956, 0.00000000];
L(5).I = [0.00589156, 0.00568001, 0.00076099, 0.00000135, -0.00003509, -0.00000270];
L(6).I = [0 0 0 0 0 0];
L(7).I = [0 0 0 0 0 0];

L(1).Jm =  0.;
L(2).Jm =  0.;
L(3).Jm =  0.;
L(4).Jm =  0.;
L(5).Jm =  0.;
L(6).Jm =  0.;
L(7).Jm =  0.;

L(1).G =  0.;
L(2).G =  0.;
L(3).G =  0.;
L(4).G =  0.;
L(5).G =  0.;
L(6).G =  0.;
L(7).G =  0.;

% viscous friction (motor referenced)
L(1).B =   0.;
L(2).B =   0.;
L(3).B =   0.;
L(4).B =   0.;
L(5).B =   0.;
L(6).B =   0.;
L(7).B =   0.;

cody = SerialLink(L, 'name', 'Cody', ...
    'manufacturer', 'Meka', 'comment', 'params from Meka');

cody.base = [-1 0 0 0;
             0 0 -1 0;
             0 -1 0 0;
             0 0 0 1];

%
% some useful poses
%
qz = [0 0 0 0 0 0 0]; % zero angles, L shaped pose
qn = [pi/2 0 0 0 0 0 0]; % zero angles, L shaped pose
%qr = [0 pi/2 -pi/2 0 0 0]; % ready pose, arm up
%qs = [0 0 -pi/2 0 0 0];
%qn=[0 pi/4 pi 0 pi/4  0];


clear L
