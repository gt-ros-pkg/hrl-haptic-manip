clear all; clc;
%run startup_rvc;
sim_model = 'mpc_model_control_robot_with_forces';

%run ./robot/mdl_threelink_sympy_test;
run ./robot/mdl_threelink_ijrr_test;
robot = threelink;

model_robot = make_variable_robot(1, 1, 1);

%urlwrite('http://cvxgen.stanford.edu/download/cvxgen.m', 'cvxgen.m');
%cvxgen(785952849920);

Kp = diag([30, 20, 15]);
Kd = diag([20, 15, 8]);
delta_t = 0.01;
delta_t_impulse_vec = [0.4, 0.8, 1.2, 1.6, 2.0, 3.0, 4.0, 5.0, 6.0];
damping_vec = [10];
stiffness_vec = [40000, 20000, 10000, 5000];

for tt = 1:1:length(damping_vec)
    for m = 1:1:length(stiffness_vec)
        for d = 1:1:length(delta_t_impulse_vec)            
            if exist(strcat('./impact_search_t_impulse_', num2str(delta_t_impulse_vec(d)), '_damping_', num2str(damping_vec(tt)), '_stiffness_', num2str(stiffness_vec(m)), '.mat')) == 0
                q_0 = deg2rad([-65, 40, 135]);
                q_des_cur_0 = q_0;
                qd_0 = [0, 0, 0];
                num_joints = length(q_0);
                num_max_contacts = 15;
                
                x_g = [0.383, 0.475]';
                obst_pos = [0.4, -0.1, 0.0;
                    0.4, 0.3, 0.0;
                    0.2, 0.0, 0.0];
                [r_obst, c_obst] = size(obst_pos);
                pose = model_robot.fkine(q_0);
                x_cur = pose(1:2, 4);
                x_start = x_cur;
                delta_x_d = x_g - x_cur;
                
                
                %init some params outside of loop
                contact_stiffness = stiffness_vec(m);
                contact_damping = damping_vec(tt);
                params.Kp = Kp;
                params.Kd = Kd;
                
                
                waypoint = 0.015;
                params.delta_rate_f_max = 35.*ones(num_max_contacts,1);
                params.alpha = 239;
                params.mu = 15.;
                params.zeta = 0.743;
                params.beta = 255.;
                %params.gamma = 10.;
                params.u_max = [94., 94., 94.]'; %update this
                params.q_min = deg2rad([-150, -63, 0])';
                params.q_max = deg2rad([150, 162, 159])';
                %params.qd_max = [0.4, 0.4, 0.4]'; %update this
                params.delta_t = delta_t;
                params.f_max = 5.0;
                
                q_real = q_0;
                qd_real = qd_0;
                q_control = [];
                x = [];
                y = [];
                max_forces = [];
                
                delta_t_real = 0.01
                i = 0;
                %while norm(x_g - x_cur) > 0.001
                rms_vec = []
                %for i = 0:0.01:7.0
                flag_success = false;
                
                while i < 24.0 && flag_success == false
                    %for i = 0:0.01:0.01
                    dist = norm(cross([x_cur; 0]-[x_start; 0],[x_cur; 0]-[x_g; 0]))/norm(x_g-x_start);
                    rms_vec = [rms_vec dist];
                    delta_t = 0.01;
                    
                    i = i+delta_t
                    pose = model_robot.fkine(q_0);
                    x_cur = pose(1:2, 4) ;
                    
                    %delta_x_d = x_g - x_cur;
                    if norm(x_g-x_cur) > waypoint
                        delta_x_d = (x_g - x_cur)/norm(x_g-x_cur)*waypoint;
                    else
                        delta_x_d = x_g - x_cur;
                    end
                    
                    if norm(x_g-x_cur) < 0.01 && norm(qd_0) < 0.001
                        flag_success = true;
                    end
                    
                    
                    J_buf = model_robot.jacob0(q_0);
                    J = J_buf(1:length(delta_x_d), :);
                    normal = [delta_x_d(2), -delta_x_d(1)];
                    normal = normal/norm(normal);
                    params.vel_norm_J = normal*J;
                    
                    fmax = 5;
                    %delta_t_impulse = 0.15;  % started with 0.07 update this?
                    delta_t_impulse = delta_t_impulse_vec(d) %1.6  %.08 %0.05; %0.03;  % started with 0.07 update this?
                    %f_max_delta_t = fmax*delta_t_impulse*ones(3,1);
                    
                    %done
                    f_cur = zeros(num_max_contacts,1);
                    contact_J = zeros(num_max_contacts, 3, num_joints);
                    tau_cont_sum_0 = zeros(num_joints,1);
                    
                    % not done, START HERE TOMORROW MORNING
                    n_K_J_all = zeros(num_max_contacts, num_joints);
                    n_K_J_over = zeros(num_max_contacts, num_joints);
                    %J_T_K_J = zeros(num_max_contacts, num_joints, num_joints);
                    all_J_T_K_J = zeros(num_joints, num_joints);
                    all_J_T_Kd_J = zeros(num_joints, num_joints);
                    delta_f_max = ones(num_max_contacts, 1)*1000;
                    delta_f_des = zeros(num_max_contacts, 1);
                    
                    % this is a quick hack to allow me to run the trial
                    a1 = model_robot.links(1).a;
                    a2 = model_robot.links(2).a;
                    a3 = model_robot.links(3).a;
                    
                    p0 = [0, 0, 0]';
                    p1 = [a1*cos(q_0(1)) a1*sin(q_0(1)) 0]';
                    p2 = [a1*cos(q_0(1)) + a2*cos(q_0(1)+q_0(2));
                        a1*sin(q_0(1)) + a2*sin(q_0(1)+q_0(2));
                        0];
                    p3 = [a1*cos(q_0(1)) + a2*cos(q_0(1)+q_0(2)) + a3*cos(q_0(1)+q_0(2)+q_0(3));
                        a1*sin(q_0(1)) + a2*sin(q_0(1)+q_0(2)) + a3*sin(q_0(1)+q_0(2)+q_0(3));
                        0];
                    jt_pos = [p0'; p1'; p2'; p3'];
                    
                    
                    %%%%%%This is the old version of the impact model that Charlie didn't like%%%%%%%
                    %     tau_max_delta_t = [];
                    %     for kk = 1:1:3
                    %         max_dist = 0;
                    %         max_pos = [0, 0, 0];
                    %         for jj = 1:1:4
                    %             if jj > kk
                    %                 dist = norm(jt_pos(jj,:) - jt_pos(kk,:));
                    %                 if dist > max_dist
                    %                     max_dist = dist;
                    %                     max_pos = jt_pos(jj,:);
                    %                 end
                    %             end
                    %         end
                    %         f_dir = cross(max_pos-jt_pos(kk,:), [0, 0, 1]);
                    %         tau_impulse_max = norm(cross(-f_dir*5, jt_pos(kk,:) - max_pos));
                    %         tau_max_delta_t = [tau_max_delta_t; tau_impulse_max*delta_t_impulse];
                    %     end
                    
                    %                 end
                    %%%%%%This is the old version of the impact model that Charlie didn't like%%%%%%%
                    
                    %this is the much simpler version that requires delta_t to be much larger
                    tau_max_delta_t = ones(num_joints, 1) * (fmax*0.02)*delta_t_impulse;
                    
                    
                    num_cont = 0;
                    for kk = 1:1:r_obst
                        for mm = 1:1:num_joints
                            num_taxels = floor(model_robot.links(mm).a/0.01);
                            for ll = 1:1:num_taxels
                                if mm == 1
                                    x_test = model_robot.links(mm).a*ll/num_taxels*cos(q_0(1));
                                    y_test = model_robot.links(mm).a*ll/num_taxels*sin(q_0(1));
                                elseif mm == 2
                                    x_test = model_robot.links(mm-1).a*cos(q_0(1))+ ...
                                        model_robot.links(mm).a*ll/num_taxels*cos(q_0(1)+q_0(2));
                                    y_test = model_robot.links(mm-1).a*sin(q_0(1))+ ...
                                        model_robot.links(mm).a*ll/num_taxels*sin(q_0(1)+q_0(2));
                                elseif mm == 3
                                    x_test = model_robot.links(mm-2).a*cos(q_0(1)) + ...
                                        model_robot.links(mm-1).a*cos(q_0(1)+q_0(2)) + ...
                                        model_robot.links(mm).a*ll/num_taxels*cos(q_0(1)+q_0(2)+q_0(3));
                                    y_test = model_robot.links(mm-2).a*sin(q_0(1))+ ...
                                        model_robot.links(mm-1).a*sin(q_0(1)+q_0(2)) + ...
                                        model_robot.links(mm).a*ll/num_taxels*sin(q_0(1)+q_0(2)+q_0(3));
                                end
                                
                                if norm([x_test, y_test, 0.0] - obst_pos(kk,:)) < 0.03
                                    num_cont = num_cont + 1;
                                    J_cont = calc_jacobian(q_0, [x_test, y_test, 0.0]', ...
                                        [model_robot.links(1).a, model_robot.links(2).a], mm);
                                    contact_J(num_cont, :, :) = J_cont;
                                    cont_normal = ([x_test y_test 0]- obst_pos(kk,:))'/norm([x_test y_test 0]- obst_pos(kk,:));
                                    mat_normal = cont_normal*cont_normal';
                                    K_cont = contact_stiffness*mat_normal;
                                    Kd_cont = contact_damping*mat_normal;
                                    f_vec = K_cont*(cont_normal'*0.03 - ([x_test, y_test, 0]- obst_pos(kk,:)))' - Kd_cont*J_cont*qd_0';
                                    %f_vec= K_cont*([x_test, y_test, 0] - obst_pos(kk,:))'; %...
                                    %- Kd_cont*J_cont*qd_0;
                                    f_cur(num_cont) = norm(f_vec) ;
                                    
                                    %should there be a negative here on cont_normal or not?!!!!
                                    %                     if f_cur(num_cont) > 5
                                    %                         n_K_J_over(num_cont,:) = -cont_normal'*K_cont*J_cont;
                                    %                         if f_cur(num_cont) > 10
                                    %                             delta_f_des(num_cont) = -5.;
                                    %                         else
                                    %                             delta_f_des(num_cont) = 5. - f_cur(num_cont);
                                    %                         end
                                    %                     else
                                    %                         n_K_J_all(num_cont,:) = -cont_normal'*K_cont*J_cont;
                                    %                         delta_f_max(num_cont) = 5. - f_cur(num_cont);
                                    %                     end
                                    
                                    n_K_J_all(num_cont,:) = -cont_normal'*K_cont*J_cont;
                                    delta_f_max(num_cont) = 5. - f_cur(num_cont);
                                    
                                    
                                    tau_cont_sum_0 = tau_cont_sum_0 + J_cont'*f_vec;
                                    %J_T_K_J(num_cont, :, :) = J_cont'*K_cont*J_cont;
                                    all_J_T_K_J = all_J_T_K_J + J_cont'*K_cont*J_cont;
                                    all_J_T_Kd_J = all_J_T_Kd_J + J_cont'*Kd_cont*J_cont;
                                    %pause
                                end
                            end
                        end
                    end
                    
                    
                    mass = model_robot.inertia(q_0);
                    cor = model_robot.coriolis(q_0, qd_0);
                    A = [inv(mass)*(-Kd-cor) -inv(mass)*(Kp+all_J_T_K_J);
                        eye(num_joints), zeros(num_joints,num_joints)];
                    
                    %     B = [inv(mass)  inv(mass)*Kp;
                    %         zeros(num_joints, num_joints*2)];
                    
                    B_c = [inv(mass)*Kp inv(mass), inv(mass)*all_J_T_K_J;
                        zeros(num_joints,3*num_joints)];
                    C = eye(num_joints*2);
                    sys = ss(A,B_c,C,0);
                    sys_d = c2d(sys, delta_t_real, 'zoh');
                    A_tl = sys_d.a(1:num_joints,1:num_joints);
                    A_tr = sys_d.a(1:num_joints,num_joints+1:num_joints*2);
                    A_bl = sys_d.a(num_joints+1:num_joints*2,1:num_joints);
                    A_br = sys_d.a(num_joints+1:num_joints*2,num_joints+1:num_joints*2);
                    B_t1 = sys_d.b(1:num_joints, 1:num_joints);
                    B_t2= sys_d.b(1:num_joints, num_joints+1:2*num_joints);
                    B_t3= sys_d.b(1:num_joints, num_joints*2+1:3*num_joints);
                    B_b1 = sys_d.b(num_joints+1:num_joints*2, 1:num_joints);
                    B_b2 = sys_d.b(num_joints+1:num_joints*2, num_joints+1:2*num_joints);
                    B_b3 = sys_d.b(num_joints+1:num_joints*2, num_joints*2+1:3*num_joints);
                    
                    %     robot.plot(q_0)
                    max_forces = [max_forces; max(f_cur)];
                    
                    params.delta_x_d = delta_x_d;
                    params.q_0 = q_0';
                    params.qd_0 = qd_0';
                    params.A_tl = A_tl;
                    params.A_tr = A_tr;
                    params.A_bl = A_bl;
                    params.A_br = A_br;
                    params.B_t1 = B_t1;
                    params.B_t2 = B_t2;
                    params.B_t3 = B_t3;
                    params.B_b1 = B_b1;
                    params.B_b2 = B_b2;
                    params.B_b3 = B_b3;
                    params.J = J;
                    %params.f_cur = f_cur;
                    params.delta_f_max = delta_f_max/contact_stiffness;
                    params.delta_f_des = delta_f_des/contact_stiffness;
                    params.tau_cont_sum_0 = tau_cont_sum_0;
                    %params.mass_n_J_com = [m_n_J1; m_n_J2; m_n_J3];
                    params.mass = mass;
                    
                    %             m_n_J1
                    %             m_n_J2
                    %             m_n_J3
                    %             f_max_delta_t
                    %             m_n_J1*qd_0'
                    %             m_n_J2*qd_0'
                    %             m_n_J3*qd_0'
                    %             max(f_cur)
                    
                    %params.n_J_com_link1 = n_J1;
                    %params.n_J_com_link2 = n_J2;
                    %params.n_J_com_link3 = n_J3;
                    %params.mass = [robot.links(1).m; robot.links(2).m; robot.links(3).m];
                    
                    %params.f_max_delta_t = f_max_delta_t;
                    params.tau_max_delta_t = tau_max_delta_t;
                    
                    %     for jj = 1:1:num_max_contacts
                    %         %params.contact_J{jj} = reshape(contact_J(jj,:,:), 3, num_joints);
                    %         params.J_T_K_J{jj} = reshape(J_T_K_J(jj,:,:), num_joints, num_joints)';
                    %     end
                    params.all_J_T_K_J = all_J_T_K_J;
                    params.n_K_J_all = n_K_J_all/contact_stiffness;
                    params.n_K_J_over = n_K_J_over/contact_stiffness;
                    params.q_des_cur_0 = q_des_cur_0';
                    
                    [vars, status] = csolve_humanoid_param_var(params);
                    
                    q_init = q_0;
                    qd_init = qd_0;
                    
                    if status.converged == 1
                        q_des = q_des_cur_0 + vars.u_0';
                        q_des_cur_0 = vars.q_des_cur_1';
                    else
                        q_des = q_des_cur_0
                        q_des_cur_0 = q_des_cur_0  %this is obvious, but just to be explicit about what is happening
                    end
                    
                    %q_des = vars.u_1'
                    %state_zoh_buf = sys_d.a*[qd_int, q_int]' + sys_d.b*[zeros(num_joints,1); q_des'];
                    % assign all current values for params.
                    
                    out = sim(sim_model);
                    q = out.get('q');
                    t = out.get('t');
                    qd = out.get('qd');
                    qdd = out.get('qdd');
                    
                    %     robot.plot(q_0);
                    %     hold on
                    %     plot(obst_pos(:,1), obst_pos(:,2), 'ro');
                    %     hold on
                    %     plot(x_g(1), x_g(2), 'g*');
                    %
                    %robot.plot(q_0);
                    
                    q_0 = q(end,:);
                    qd_0 = qd(end,:);
                    q_real = [q_real; q_0];
                    qd_real = [qd_real; qd_0];
                    q_control = [q_control; vars.u_0'];
                    buf = model_robot.fkine(q_0);
                    x_buf = buf(1,4);
                    y_buf = buf(2,4);
                    x = [x, x_buf];
                    y = [y, y_buf];
                end
                
                
                
                control = []
                time = []
                
                for i = 1:1:length(q_control)
                    control = [control;q_control(i,:); q_control(i,:)] ;
                    time = [time; (i)*0.01; (i+1)*0.01;];
                end
                
                %             ind_try = 1:24:length(q_real);
                %             robot.plot(q_real(ind_try,:));
                %             hold on;
                %             plot(x,y, 'r--');
                %
                %             figure();
                %             plot(max_forces);
                %             t_force_08 = 0:0.01:(length(max_forces)-1)*0.01;
                %
                
                %save('mpc_matlab_controller_zero_weight_jerk_small_step_0.02_vel_of_2pi_smaller_u_lim.mat')
                
                x_vel=[];
                for i = 1:1:length(qd_real)
                    J = robot.jacob0(q_real(i,:));
                    x_vel = [x_vel; norm(J*qd_real(i,:)')'];
                end
                
                save(strcat('./impact_search_t_impulse_', num2str(delta_t_impulse_vec(d)), '_damping_', num2str(damping_vec(tt)), ...
                    '_stiffness_', num2str(stiffness_vec(m)), '.mat'), ...
                    'q_real', 'qd_real', 'x_vel', 'robot', 'max_forces')
            end
            
        end
    end
end
% better plots
% run_control_tests_with_force
% robot.plot(q_real(ind_try,:))
% xlim([-0.218, 0.618])
% ylim([-0.418, 0.818])
% hold on
% plot(x_g(1), x_g(2), 'g*', 'MarkerSize', 10);
% plot(obst_pos(:,1), obst_pos(:,2), 'ro', 'MarkerSize',15, 'MarkerFaceColor',[1 0 0]);
% robot.plot(q_real(ind_try,:))
% plot(x_g(1), x_g(2), 'g*', 'MarkerSize', 15);
% robot.plot(q_real(ind_try,:))
% plot(x,y, 'r--');
