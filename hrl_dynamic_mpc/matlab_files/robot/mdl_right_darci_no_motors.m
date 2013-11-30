% #                                                                                                                                                                      
% # Copyright (c) 2013, Georgia Tech Research Corporation                                                                                                                
% # All rights reserved.                                                                                                                                                 
% #                                                                                                                                                                      
% # Redistribution and use in source and binary forms, with or without                                                                                                   
% # modification, are permitted provided that the following conditions are met:                                                                                          
% #     * Redistributions of source code must retain the above copyright                                                                                                 
% #       notice, this list of conditions and the following disclaimer.                                                                                                  
% #     * Redistributions in binary form must reproduce the above copyright                                                                                              
% #       notice, this list of conditions and the following disclaimer in the                                                                                            
% #       documentation and/or other materials provided with the distribution.                                                                                           
% #     * Neither the name of the Georgia Tech Research Corporation nor the                                                                                              
% #       names of its contributors may be used to endorse or promote products                                                                                           
% #       derived from this software without specific prior written permission.                                                                                          
% #                                                                                                                                                                      
% # THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND                                                                                         
% # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                                                                                        
% # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                                                                                               
% # DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,                                                                                       
% # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT                                                                                         
% # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,                                                                                          
% # OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF                                                                                            
% # LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE                                                                                      
% # OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF                                                                                            
% # ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                                                                                           
% #                                                                                                                                                                      
% 
% # \\ author Marc Killpack (Healthcare Robotics Lab, Georgia Tech.)                                                                                                     
% # \\ adviser Charlie Kemp (Healthcare Robotics Lab, Georgia Tech.) 


%MDL_DARCI Create model of Cody manipulator from Meka
%
%      mdl_right_darci_no_motors

clear L
% These are the DH Parameters, in modified form
%             th    d       a         alpha   sigma  offset
L(1) = Link([ 0     0.18465 0         pi/2    0        -pi/2], 'modified');
L(2) = Link([ 0 	0.      0	      pi/2    0         pi/2], 'modified');
L(3) = Link([ 0     0.27857	0.03175   pi/2    0         pi/2], 'modified');
L(4) = Link([ 0     0       -0.00502  pi/2    0         0   ], 'modified');
L(5) = Link([ 0     0.27747 0         -pi/2   0         0   ], 'modified');
L(6) = Link([ 0     0       0         pi/2    0         pi/2], 'modified');
L(7) = Link([ 0     0       0         pi/2    0         pi/2], 'modified');


% These are the same DH Parameters, in standard form
% L(1) = Link([ 0    0.18465 0         pi/2    0         pi/2], 'standard');
% L(2) = Link([ 0 	0.      0.03175   pi/2    0         pi/2], 'standard');
% L(3) = Link([ 0     0.27857	0.00502  -pi/2    0        -pi/2], 'standard');
% L(4) = Link([ 0     0       0         pi/2    0         0   ], 'standard');
% L(5) = Link([ 0     0.27747 0        -pi/2   0         0   ], 'standard');
% L(6) = Link([ 0     0       0        -pi/2    0         pi/2], 'standard');
% L(7) = Link([ 0     0       -0.04414         0       0         0   ], 'standard');

% mass of each link
L(1).m = 1.9872;
L(2).m = 0.5575;
L(3).m = 2.220;
L(4).m = 0.22;
L(5).m = 1.7;
L(6).m = 0.212;
L(7).m = 0.084;

% center of mass location with respect to modified DH params 
L(1).r = [0, -0.0094, -0.0240];
L(2).r = [0.0264, -0.0431, -0.0008];
L(3).r = [0.0080, 0., -0.0877];
L(4).r = [0.0, 0.02573092, -0.00080947];
L(5).r = [0.00193729, 0.00046171, -0.13853286];
L(6).r = [8.7719999999999994e-05, -0.0018379500000000001, -0.0018201000000000001];
L(7).r = [3.0e-05, -0.01092, 0.00292];


% rotational inertia with respect to center of gravity (I think?), see
% toolbox documentation for ordering - Ixx, Iyy and Izz, etc.
L(1).I = [0.00681123,  0.004616, 0.003267,  2.79e-06,   0.0001518,  -9.60e-07];
L(2).I = [0.00195358,  0.00121948, 0.0021736, -0.00077554, 4.74e-06, 4.39e-06];
L(3).I = [0.02949661, 0.02983326, 0.00244235, 8.636e-05, -0.00013024, -0.0024288];
L(4).I = [0.00062344, 0.00042457, 0.00038623, 2.0e-08, 1.768e-05, 0.0];
L(5).I = [0.03130809, 0.03135827, 0.00120798, -2.89e-06, -3.896e-05, -0.00089153];
L(6).I = [0.00011597, 0.00011378, 7.497e-05, -5.0e-08, -1.2e-07, 4.0e-08];
L(7).I = [9.428e-05, 6.133e-05, 5.054e-05, 2.0e-08, -7.4e-07, 0.0];


% motor inertia
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


% making robot arm object
darci_right = SerialLink(L, 'name', 'darci right arm', ...
    'manufacturer', 'Meka', 'comment', 'params from Meka');

% specifying direction of gravity
darci_right.gravity = [0, 0, 0]';

% specifying end effector frame for forward kinematics
darci_right.tool = [1, 0, 0, 0;
                    0, 0, -1, -0.153;
                    0, 1, 0, 0;
                    0, 0, 0, 1];

% also possible to define base frame
% darci.base = [1 0 0 0;
%               0 0 -1 0;
%               0 -1 0 0;
%               0 0 0 1];



% some useful poses
%f
qz = [0 0 0 0 0 0 0]; % zero angles
qn = [0 0 0 pi/2 0 0 0]; % elbow up
qd1 = ones(1,7)*0.1
qd2 = ones(1,7)*0.3

clear L
