% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(kappa*(sum(pos(abs(2*mass*qd_1) - tau_max_delta_t)) + sum(pos(abs(2*mass*qd_2) - tau_max_delta_t)) + sum(pos(abs(2*mass*qd_3) - tau_max_delta_t))) + alpha*posture_weight*quad_form(delta_q_des - (q_6 - q_0), eye(7)) + xyz_weight*zeta*quad_form(delta_x_des - J*(q_6 - q_0), eye(3)) + mu*(quad_form(u_0, eye(7)) + quad_form(u_1, eye(7)) + quad_form(u_2, eye(7))) + beta*sum(pos(n_K_J_all*(q_6 - q_0) - delta_f_max)))
%   subject to
%     qd_1 == A_tl*qd_0 + A_tr*q_0 + B_t1*(q_des_cur_0 + u_0) + B_t2*tau_cont_sum_0 + B_t3*q_0
%     qd_2 == A_tl*qd_1 + A_tr*q_1 + B_t1*(q_des_cur_1 + u_1) + B_t2*tau_cont_sum_0 + B_t3*q_0
%     qd_3 == A_tl*qd_2 + A_tr*q_2 + B_t1*(q_des_cur_2 + u_2) + B_t2*tau_cont_sum_0 + B_t3*q_0
%     q_1 == A_bl*qd_0 + A_br*q_0 + B_b1*(q_des_cur_0 + u_0) + B_b2*tau_cont_sum_0 + B_b3*q_0
%     q_2 == A_bl*qd_1 + A_br*q_1 + B_b1*(q_des_cur_1 + u_1) + B_b2*tau_cont_sum_0 + B_b3*q_0
%     q_3 == A_bl*qd_2 + A_br*q_2 + B_b1*(q_des_cur_2 + u_2) + B_b2*tau_cont_sum_0 + B_b3*q_0
%     qd_4 == A_tl*qd_3 + A_tr*q_3 + B_t1*q_des_cur_3 + B_t2*tau_cont_sum_0 + B_t3*q_0
%     qd_5 == A_tl*qd_4 + A_tr*q_4 + B_t1*q_des_cur_3 + B_t2*tau_cont_sum_0 + B_t3*q_0
%     qd_6 == A_tl*qd_5 + A_tr*q_5 + B_t1*q_des_cur_3 + B_t2*tau_cont_sum_0 + B_t3*q_0
%     q_4 == A_bl*qd_3 + A_br*q_3 + B_b1*q_des_cur_3 + B_b2*tau_cont_sum_0 + B_b3*q_0
%     q_5 == A_bl*qd_4 + A_br*q_4 + B_b1*q_des_cur_3 + B_b2*tau_cont_sum_0 + B_b3*q_0
%     q_6 == A_bl*qd_5 + A_br*q_5 + B_b1*q_des_cur_3 + B_b2*tau_cont_sum_0 + B_b3*q_0
%     q_des_cur_1 == q_des_cur_0 + u_0
%     q_des_cur_2 == q_des_cur_1 + u_1
%     q_des_cur_3 == q_des_cur_2 + u_2
%     q_min <= q_1
%     q_min <= q_2
%     q_min <= q_3
%     q_min <= q_4
%     q_min <= q_5
%     q_min <= q_6
%     q_1 <= q_max
%     q_2 <= q_max
%     q_3 <= q_max
%     q_4 <= q_max
%     q_5 <= q_max
%     q_6 <= q_max
%     torque_min <= Kp*(q_des_cur_0 + u_0 - q_0) - Kd*qd_0
%     torque_min <= Kp*(q_des_cur_1 + u_1 - q_1) - Kd*qd_1
%     torque_min <= Kp*(q_des_cur_2 + u_2 - q_2) - Kd*qd_2
%     Kp*(q_des_cur_0 + u_0 - q_0) - Kd*qd_0 <= torque_max
%     Kp*(q_des_cur_1 + u_1 - q_1) - Kd*qd_1 <= torque_max
%     Kp*(q_des_cur_2 + u_2 - q_2) - Kd*qd_2 <= torque_max
%
% with variables
%      q_1   7 x 1
%      q_2   7 x 1
%      q_3   7 x 1
%      q_4   7 x 1
%      q_5   7 x 1
%      q_6   7 x 1
% q_des_cur_1   7 x 1
% q_des_cur_2   7 x 1
% q_des_cur_3   7 x 1
%     qd_1   7 x 1
%     qd_2   7 x 1
%     qd_3   7 x 1
%     qd_4   7 x 1
%     qd_5   7 x 1
%     qd_6   7 x 1
%      u_0   7 x 1
%      u_1   7 x 1
%      u_2   7 x 1
%
% and parameters
%     A_bl   7 x 7
%     A_br   7 x 7
%     A_tl   7 x 7
%     A_tr   7 x 7
%     B_b1   7 x 7
%     B_b2   7 x 7
%     B_b3   7 x 7
%     B_t1   7 x 7
%     B_t2   7 x 7
%     B_t3   7 x 7
%        J   3 x 7
%       Kd   7 x 7    PSD
%       Kp   7 x 7    PSD
%    alpha   1 x 1    positive
%     beta   1 x 1    positive
% delta_f_max  10 x 1
% delta_q_des   7 x 1
% delta_x_des   3 x 1
%    kappa   1 x 1    positive
%     mass   7 x 7    PSD
%       mu   1 x 1    positive
% n_K_J_all  10 x 7
% posture_weight   1 x 1    positive
%      q_0   7 x 1
% q_des_cur_0   7 x 1
%    q_max   7 x 1
%    q_min   7 x 1
%     qd_0   7 x 1
% tau_cont_sum_0   7 x 1
% tau_max_delta_t   7 x 1
% torque_max   7 x 1    positive
% torque_min   7 x 1
% xyz_weight   1 x 1    positive
%     zeta   1 x 1    positive
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A_bl, ..., params.zeta, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2014-01-13 17:28:28 -0500.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
