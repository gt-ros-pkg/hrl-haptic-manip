clear all; clc;
%run startup_rvc;
sim_model = 'mpc_model_control_robot';

%run ./robot/mdl_threelink_sympy_test;
run ./robot/mdl_kreacher_test;
robot = kreacher;

model_robot = kreacher; %make_variable_robot(1, 1, 1);

horizon = 1;
kd = 0.0532;
kp_vec = [1.032, 3.19];
delta_t_vec = [0.04, 0.02, 0.01];
beta_vec = [0.001, 0.01, 1.0];
waypoint_vec = [0.001, 0.01, 0.1, 1];



for ll = 1:1:length(kp_vec)
    for pp = 1:1:length(delta_t_vec)
        for dd = 1:1:length(beta_vec)
            for kk = 1:1:length(waypoint_vec);
                %Kp = diag([1.032]);
                %Kd = diag([0.0112]);
                Kp = diag([kp_vec(ll), kp_vec(ll), kp_vec(ll), kp_vec(ll)]);
                Kd = diag([kd, kd, kd, kd]);
                KP_t_KP = Kp'*Kp;
                delta_t = delta_t_vec(pp);
                q_0 = deg2rad([0., 0, 0, 0]);
                q_des_cur_0 = q_0;
                qd_0 = [0, 0, 0, 0];
                num_joints = length(q_0);
                
                x_g = [0.3, 0.3]';
                pose = model_robot.fkine(q_0);
                x_cur = pose(1:2, 4);
                x_start = x_cur;
                delta_x_d = x_g - x_cur;
                
                %init some params outside of loop
                params.alpha = 1.0;
                params.beta = beta_vec(dd); % 0.000001;
                params.q_min = deg2rad(-80)*ones(4,1);
                params.q_max = deg2rad(80)*ones(4,1);
                
                q_real = q_0;
                qd_real = qd_0;
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
                    
                    pose = model_robot.fkine(q_0);
                    x_cur = pose(1:2, 4) ;
                    
                    delta_x_d = x_g - x_cur;
                    if norm(x_g-x_cur) > waypoint_vec(kk)
                        delta_x_d = (x_g - x_cur)/norm(x_g-x_cur)*waypoint_vec(kk);
                    else
                        delta_x_d = x_g - x_cur;
                    end
                    
                    J_buf = model_robot.jacob0(q_0);
                    J = J_buf(1:length(delta_x_d), :);
                    
                    %     robot.plot(q_0)
                    
                    params.delta_x_d = delta_x_d;
                    params.q_0 = q_0';
                    params.B = eye(4) %would normally at least vary according to contacts
                    params.J = J;
                    params.KP_t_KP = KP_t_KP;
                    
                    
                    params.u_max = degtorad(20) + params.q_max - q_0';
                    params.u_min = -q_0' + params.q_min - degtorad(20);
                    
                    [vars, status] = csolve_kreacher_QS_horiz_01(params); %csolve(params);
                    
                    q_init = q_0;
                    qd_init = qd_0;
                    
                    if status.converged == 1
                        q_des = q_des_cur_0 + vars.u_0';
                    else
                        q_des = q_des_cur_0
                        q_des_cur_0 = q_des_cur_0  %this is obvious, but just to be explicit about what is happening
                    end
                    
                    q_des_cur_0 = q_des;
                    
                    if norm(x_g-x_cur) < 0.001 && norm(vars.u_0) < 0.0001
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
                    
                    ub = q_0 > deg2rad(80);
                    lb = q_0 < deg2rad(-80);
                    q_0(ub) = deg2rad(80);
                    q_0(lb) = deg2rad(-80);
                    qd_0(ub) = 0;
                    qd_0(lb) = 0;
                    
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
                    time = [time; (i)*delta_t; (i+1)*delta_t;];
                end
                
                x_vel=[];
                for i = 1:1:length(qd_real)
                    J = robot.jacob0(q_real(i,:));
                    x_vel = [x_vel; norm(J*qd_real(i,:)')'];
                end
                
                path = strcat('./variable_control_horizon_tests/kreacher/QS_horizon_',num2str(horizon), ...
                    '_stiffness_', num2str(kp_vec(ll)), '_t_step_', num2str(delta_t_vec(pp)), ...
                    '_beta_', num2str(beta_vec(dd)), '_waypoint_mag_', num2str(waypoint_vec(kk)), '.mat')
                save(path)
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
