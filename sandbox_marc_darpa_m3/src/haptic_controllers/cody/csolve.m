% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(delta_x_d - J*(x_1 - x_0), I) + quad_form(x_1 - x_0, KP_t_KP))
%   subject to
%     x_1 == x_0 + B*u_0
%     q_min <= x_1
%     x_1 <= q_max
%     f_min <= n_K_ci_J_ci*(x_1 - x_0)
%     n_K_ci_J_ci*(x_1 - x_0) <= f_max
%     u_min <= u_0
%     u_0 <= u_max
%
% with variables
%      u_0   7 x 1
%      x_1   7 x 1
%
% and parameters
%        B   7 x 7
%        I   3 x 3    PSD
%        J   3 x 7
%  KP_t_KP   7 x 7    PSD
% delta_x_d   3 x 1
%    f_max 100 x 1
%    f_min 100 x 1
% n_K_ci_J_ci 100 x 7
%    q_max   1 x 1    positive
%    q_min   1 x 1    positive
%    u_max   7 x 1
%    u_min   7 x 1
%      x_0   7 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.B, ..., params.x_0, then run
%   [vars, status] = csolve(params, settings)


% Produced by CVXGEN, 2012-03-08 11:13:00 -0800.
% CVXGEN is Copyright (C) 2006-2011 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2011 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
