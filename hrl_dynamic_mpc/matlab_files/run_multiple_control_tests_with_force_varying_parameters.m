clear all; clc;
%run startup_rvc;
sim_model = 'mpc_model_control_robot_with_forces';

run ./robot/mdl_threelink_ijrr_test;
model_robot = threelink;

q_reals = []
qd_reals = []
xs = []
ys = []
controls = []
cont_times = []

mass_vary = 0.5:0.1:1.5;
%mass_vary = ones(length(mass_vary), 1)
com_vary = 0.5:0.1:1.5;
I_vary = 0.5:0.1:1.5;
delta_t_std = 0.0:0.001:0.01;

result_rms = zeros(length(delta_t_std), length(mass_vary));
result_max_forces = zeros(length(delta_t_std), length(mass_vary));
result_mean_forces = zeros(length(delta_t_std), length(mass_vary));
result_median_forces = zeros(length(delta_t_std), length(mass_vary));
result_90th_forces = zeros(length(delta_t_std), length(mass_vary));
result_99th_forces = zeros(length(delta_t_std), length(mass_vary));
result_times = zeros(length(delta_t_std), length(mass_vary));
result_success = zeros(length(delta_t_std), length(mass_vary));

%this is what rotational inertia variation would look like
%result_success = zeros(length(delta_t_std), length(I_vary));
%for x_axis=1:1:length(I_vary)
%    for y_axis = 1:1:length(delta_t_std)

try
    load robustness_test_faster.mat;
catch
    warning('didnt get the file so starting over ...');
end

for x_axis=1:1:length(mass_vary)
    for y_axis = 1:1:length(delta_t_std)
        %try
        if result_times(y_axis, x_axis) == 0
            x_axis
            y_axis
            robot = make_variable_robot(mass_vary(x_axis), 1, 1);
            %urlwrite('http://cvxgen.stanford.edu/download/cvxgen.m', 'cvxgen.m');
            %cvxgen(785952849920);
            
            Kp = diag([30, 20, 15]);
            Kd = diag([20, 15, 8]);
            delta_t = 0.01;
            q_0 = deg2rad([-65, 40, 135]);  %
            %q_0 = [-0.6601    1.6366    0.5084]; % good start for unstable examples
            q_des_cur_0 = q_0;
            qd_0 =  [0, 0, 0]; %
            %qd_0 = [0.1248   -0.0478   -0.1537];  %this was good start for unstable examples
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
            delta_t_pred = 0.01
            contact_stiffness = 10000;
            contact_damping = 10;
            params.Kp = Kp;
            params.Kd = Kd;
            
            waypoint = 0.015
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
            params.delta_t = delta_t_pred;
            params.f_max = 5.0;
            
            q_real = q_0;
            qd_real = qd_0;
            q_control = [];
            x = [];
            y = [];
            max_forces = [];
            forces = [];
            
            i = 0;
            %while norm(x_g - x_cur) > 0.001
            rms_vec = []
            %for i = 0:0.01:12
            flag_success = false;
            while i < 24.0 && flag_success == false
                %for i = 0:0.01:0.01
                dist = norm(cross([x_cur; 0]-[x_start; 0],[x_cur; 0]-[x_g; 0]))/norm(x_g-x_start);
                rms_vec = [rms_vec dist];
                delta_t = delta_t_pred + delta_t_std(y_axis)*randn(1);
                if delta_t < 0
                    delta_t = 0;
                end
                i = i+delta_t
                
                
                pose = model_robot.fkine(q_0);
                x_cur = pose(1:2, 4) ;
                
                %delta_x_d = x_g - x_cur;
                if norm(x_g-x_cur) > waypoint
                    delta_x_d = (x_g - x_cur)/norm(x_g-x_cur)*waypoint;
                else
                    delta_x_d = x_g - x_cur;
                end
                
                norm(x_g-x_cur)
                
                if norm(x_g-x_cur) < 0.01 && norm(qd_0) < 0.001
                    flag_success = true;
                end
                
                mass = model_robot.inertia(q_0);
                cor = model_robot.coriolis(q_0, qd_0);
                J_buf = model_robot.jacob0(q_0);
                J = J_buf(1:length(delta_x_d), :);
                %             normal = [delta_x_d(2), -delta_x_d(1)];
                %             normal = normal/norm(normal);
                %             params.vel_norm_J = normal*J;
                
                fmax = 5;
                %delta_t_impulse = 0.15;  % started with 0.07 update this?
                delta_t_impulse = 5.0; %1.8; %10.; %1.8; %0.2;  %0.05; %0.03;  % started with 0.07 update this?
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
                %                 tau_max_delta_t = [];
                %                 for kk = 1:1:3
                %                     max_dist = 0;
                %                     max_pos = [0, 0, 0];
                %                     for jj = 1:1:4
                %                         if jj > kk
                %                             dist = norm(jt_pos(jj,:) - jt_pos(kk,:));
                %                             if dist > max_dist
                %                                 max_dist = dist;
                %                                 max_pos = jt_pos(jj,:);
                %                             end
                %                         end
                %                     end
                %                     f_dir = cross(max_pos-jt_pos(kk,:), [0, 0, 1]);
                %                     tau_impulse_max = norm(cross(-f_dir*f_max, jt_pos(kk,:) - max_pos));
                %                     tau_max_delta_t = [tau_max_delta_t; tau_impulse_max*delta_t_impulse];
                %                 end
                %%%%%%This is the old version of the impact model that Charlie didn't like%%%%%%%
                
                %this is the much simpler version that requires delta_t to
                %be much larger
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
                
                inv_mass = inv(mass);
                %inv_mass = eye(3)*100;
                
                A = [inv_mass*(-Kd-cor) -inv_mass*(Kp+all_J_T_K_J);
                    eye(num_joints), zeros(num_joints,num_joints)];
                
                %     B = [inv(mass)  inv(mass)*Kp;
                %         zeros(num_joints, num_joints*2)];
                
                B_c = [inv_mass*Kp inv(mass), inv_mass*all_J_T_K_J;
                    zeros(num_joints,3*num_joints)];
                C = eye(num_joints*2);
                sys = ss(A,B_c,C,0);
                sys_d = c2d(sys, delta_t_pred, 'zoh');
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
                
                max_forces = [max_forces; max(f_cur)];
                if max(f_cur) > 10
                    max(f_cur)
                end
                indices = find(f_cur);
                forces = [forces; f_cur(indices)];
                
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
                
                %                 robot.plot(q_0);
                %                 hold on
                %                 plot(obst_pos(:,1), obst_pos(:,2), 'ro');
                %                 hold on
                %                 plot(x_g(1), x_g(2), 'g*');
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
            
            %catch
            %    warning('it went unstable!!!');
            %end
            
            control = []
            time = []
            
            for pp = 1:1:length(q_control)
                control = [control;q_control(pp,:); q_control(pp,:)] ;
                time = [time; (pp)*0.01; (pp+1)*0.01;];
            end
            
            %         ind_try = 1:12:length(q_real);
            %         robot.plot(q_real(ind_try,:));
            %         hold on;
            %         plot(x,y, 'r--');
            %
            %         figure();
            %         plot(max_forces);
            
            
            if norm(x_g-x_cur)<=0.01
                result_success(y_axis, x_axis) = 1;
            else
                result_success(y_axis, x_axis) = -1;
            end
            
            result_max_forces(y_axis, x_axis) = max(max_forces);
            result_rms(y_axis, x_axis) = rms(rms_vec);
            q_reals = [q_reals; [nan, nan, nan]; q_real];
            qd_reals = [qd_reals; [nan, nan, nan]; qd_real];
            controls = [controls; [nan, nan, nan]; control];
            xs = [xs; nan; x'];
            ys = [ys; nan; y'];
            cont_times = [cont_times; nan; time];
            result_mean_forces(y_axis, x_axis) = mean(forces);
            result_median_forces(y_axis, x_axis) = median(forces);
            result_90th_forces(y_axis, x_axis) = prctile(forces, 90);
            result_99th_forces(y_axis, x_axis) = prctile(forces, 99);
            result_times(y_axis,x_axis) = time(end);
            
            save('./robustness_test_faster.mat', 'q_reals', 'qd_reals', 'controls', 'xs', 'ys', 'cont_times', ...
                'result_mean_forces', 'result_median_forces', 'result_90th_forces', 'result_99th_forces', ...
                'result_times', 'result_max_forces', 'result_rms', 'result_success')
        end        
    end
end







% figure
% x_all = [];
% y_all = [];
% for tt = 1:1:10*11
%     ind = [(tt-1)*603+2, tt*603]
%     x = [];
%     y = [];
%     for kk = 1:1:601
%         buf = model_robot.fkine(q_reals(kk+ind(1),:));
%         x_buf = buf(1,4);
%         y_buf = buf(2,4);
%         x = [x, x_buf];
%         y = [y, y_buf];
%     end
%     plot(x,y)
%     axis equal
%     x_all = [x_all; x];
%     y_all = [y_all; y];
%     pause
% end
%     
