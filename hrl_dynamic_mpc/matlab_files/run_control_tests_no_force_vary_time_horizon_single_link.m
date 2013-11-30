clear all; clc;
horizon_vec = [1, 5, 10, 20, 50]  %add 1 back

%run startup_rvc;
sim_model = 'mpc_model_control_robot';

%run ./robot/mdl_threelink_sympy_test;
run ./robot/mdl_onelink_test;
robot = onelink;

model_robot = onelink; %make_variable_robot(1, 1, 1);

%urlwrite('http://cvxgen.stanford.edu/download/cvxgen.m', 'cvxgen.m');
%cvxgen(785952849920);
%stiffness = 1.032
%kp = 3.19;
kd = 0.0532;
kp_vec = [3.19, 1.032];
delta_t_vec = [0.04, 0.02, 0.01];
u_max_vec = [1.28, 0.32, 0.08, 0.02];
t_impulse_vec = [0.16, 0.08, 0.04]; %[0.04, 0.08, 0.16];

for zz = 1:1:length(horizon_vec)
    horizon = horizon_vec(zz);
    for ll = 1:1:length(kp_vec)
        for pp = 1:1:length(delta_t_vec)
            for mm =1:1:length(u_max_vec)
                for dd = 1:1:length(t_impulse_vec)
                    path = strcat('./variable_control_horizon_tests/single_link/horizon_',num2str(horizon), ...
                        '_stiffness_', num2str(kp_vec(ll)), '_t_step_', num2str(delta_t_vec(pp)),'_u_max_',num2str(u_max_vec(mm)), ...
                        '_t_impulse_', num2str(t_impulse_vec(dd)), '.mat')
                    if exist(path) == 0
                        save(path);
                        %Kp = diag([1.032]);
                        %Kd = diag([0.0112]);
                        Kp = diag([kp_vec(ll)]);
                        Kd = diag([kd]);
                        delta_t = delta_t_vec(pp);
                        q_0 = deg2rad([0.]);
                        q_des_cur_0 = q_0;
                        qd_0 = [0];
                        num_joints = length(q_0);
                        
                        rad = 0.165
                        x_g = [rad*cos(pi/4.), rad*sin(pi/4.)]';
                        pose = model_robot.fkine(q_0);
                        x_cur = pose(1:2, 4);
                        x_start = x_cur;
                        delta_x_d = x_g - x_cur;
                        
                        solver = str2func(strcat('csolve_single_link_horiz_',sprintf('%02d',horizon)));
                        %init some params outside of loop
                        params.Kp = Kp;
                        params.Kd = Kd;
                        params.alpha = 239;
                        params.mu = 0; % 0.000001;
                        params.u_max = [u_max_vec(mm)]; %[10.0]; %[0.2]'; %update this
                        params.q_min = deg2rad([-80])';
                        params.q_max = deg2rad([80])';
                        %params.qd_max = [0.4]'; %update this
                        params.delta_t = delta_t;
                        
                        q_real = q_0;
                        qd_real = qd_0;
                        q_s = q_0;
                        q_control = [];
                        x = [];
                        y = [];
                        
                        delta_t_real = delta_t_vec(pp);
                        i = 0;
                        %while norm(x_g - x_cur) > 0.001
                        rms_vec = []
                        %for i = 0:0.01:7.0
                        flag_success = false;
                        
                        i = -delta_t
                        while i < 16. && flag_success == false
                            %for i = 0:delta_t:10.0  % was 1.5, also 6.0
                            %for i = 0:0.01:0.01
                            i = i+delta_t
                            dist = norm(cross([x_cur; 0]-[x_start; 0],[x_cur; 0]-[x_g; 0]))/norm(x_g-x_start);
                            rms_vec = [rms_vec dist];
                            
                            i
                            pose = model_robot.fkine(q_0);
                            x_cur = pose(1:2, 4) ;
                            
                            delta_x_d = x_g - x_cur;
                            %     if norm(x_g-x_cur) > 0.02
                            %         delta_x_d = (x_g - x_cur)/norm(x_g-x_cur)*0.02;
                            %     else
                            %         delta_x_d = x_g - x_cur;
                            %     end
                            
                            
                            J_buf = model_robot.jacob0(q_0);
                            J = J_buf(1:length(delta_x_d), :);
                            normal = [delta_x_d(2), -delta_x_d(1)];
                            normal = normal/norm(normal);
                            
                            delta_t_impulse = t_impulse_vec(dd); %0.07; %0.07; %0.03;  % started with 0.07 update this?
                            
                            % this is a quick hack to allow me to run the trial
                            a1 = model_robot.links(1).a;
                            
                            p0 = [0, 0, 0]';
                            p1 = [a1*cos(q_0(1)) a1*sin(q_0(1)) 0]';
                            jt_pos = [p0'; p1'];
                            
                            %                     tau_max_delta_t = [];
                            %                     for kk = 1:1:1
                            %                         max_dist = 0;
                            %                         max_pos = [0, 0, 0];
                            %                         for jj = 1:1:2
                            %                             if jj > kk
                            %                                 distance = norm(jt_pos(jj,:) - jt_pos(kk,:));
                            %                                 if distance > max_dist
                            %                                     max_dist = distance ;
                            %                                     max_pos = jt_pos(jj,:);
                            %                                 end
                            %                             end
                            %                         end
                            %                         f_dir = cross(max_pos-jt_pos(kk,:), [0, 0, 1])
                            %                         tau_impulse_max = norm(cross(-f_dir*5, jt_pos(kk,:) - max_pos));
                            %                         tau_max_delta_t = [tau_max_delta_t; tau_impulse_max*delta_t_impulse];
                            %                     end
                            
                            %this is the much simpler version that requires delta_t to
                            %be much larger
                            fmax = 5.;
                            tau_max_delta_t = ones(num_joints, 1)*(fmax*0.02)*delta_t_impulse;
                            
                            mass = model_robot.inertia(q_0);
                            cor = model_robot.coriolis(q_0, qd_0);
                            A = [inv(mass)*(-Kd-cor) -inv(mass)*(Kp);
                                eye(num_joints), zeros(num_joints,num_joints)];
                            
                            %     B = [inv(mass)  inv(mass)*Kp;
                            %         zeros(num_joints, num_joints*2)];
                            
                            B_c = [inv(mass)*Kp;
                                zeros(num_joints, num_joints)];
                            C = eye(num_joints*2);
                            sys = ss(A,B_c,C,0);
                            sys_d = c2d(sys, delta_t_real, 'zoh');
                            A_tl = sys_d.a(1:num_joints,1:num_joints);
                            A_tr = sys_d.a(1:num_joints,num_joints+1:num_joints*2);
                            A_bl = sys_d.a(num_joints+1:num_joints*2,1:num_joints);
                            A_br = sys_d.a(num_joints+1:num_joints*2,num_joints+1:num_joints*2);
                            B_t1 = sys_d.b(1:num_joints, 1:num_joints);
                            B_b1 = sys_d.b(num_joints+1:num_joints*2, 1:num_joints);
                            
                            %     robot.plot(q_0)
                            
                            params.delta_x_d = delta_x_d;
                            params.q_0 = q_0';
                            params.qd_0 = qd_0';
                            params.A_tl = A_tl;
                            params.A_tr = A_tr;
                            params.A_bl = A_bl;
                            params.A_br = A_br;
                            params.B_t1 = B_t1;
                            params.B_b1 = B_b1;
                            params.J = J;
                            %params.f_cur = f_cur;
                            params.mass = mass;
                            params.tau_max_delta_t = tau_max_delta_t;
                            params.q_des_cur_0 = q_des_cur_0';                 
                      
                            [vars, status] = solver(params); %csolve(params);
                            
                            q_init = q_0;
                            qd_init = qd_0;
                            
                            if status.converged == 1
                                q_des = q_des_cur_0 + vars.u_0';
                                q_des_cur_0 = vars.q_des_cur_1';
                            else
                                q_des = q_des_cur_0
                                q_des_cur_0 = q_des_cur_0  %this is obvious, but just to be explicit about what is happening
                            end
                            
                            
                            if norm(x_g-x_cur) < 0.01 && norm(qd_0) < 0.001
                                flag_success = true;
                            end
                            
                            %q_des = vars.u_1'
                            %state_zoh_buf = sys_d.a*[qd_int, q_int]' + sys_d.b*[zeros(num_joints,1); q_des'];
                            % assign all current values for params.
                            
                            out = sim(sim_model);
                            q = out.get('q');
                            t = out.get('t');
                            qd = out.get('qd');
                            qdd = out.get('qdd');
                            
                            %robot.plot(q_0);
                            %plot(x_g(1), x_g(2), 'g*');
                            %
                            %robot.plot(q_0);
                            
                            q_0 = q(end,:);
                            qd_0 = qd(end,:);
                            q_real = [q_real; q];
                            qd_real = [qd_real; qd];
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
                            time = [time; (i)*delta_t; (i+1)*delta_t;];
                        end
                        
                        x_vel=[];
                        for i = 1:1:length(qd_real)
                            J = robot.jacob0(q_real(i,:));
                            x_vel = [x_vel; norm(J*qd_real(i,:)')'];
                        end
                        
                        save(path)
                    end
                end
            end
        end
    end
end