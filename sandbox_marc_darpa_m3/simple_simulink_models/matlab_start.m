load /home/mkillpack/svn/robot1/usr/marc/darpa_m3_simple_models/MPC1.mat
%load /home/mkillpack/svn/robot1/usr/marc/darpa_m3_simple_models/MPC2.mat
f_thresh = 1000
gamma = 7
t_s = 0.01
k1 = 10
k2 = 10
k_a = 10
b_a = 1
k_tot = k1+k2+k_a

%discrete gains
kp = 0.15013
ki = 30.025
kd = 0

%continuous gains
% kp = 10
% ki = 3
% kd = 1

num = [b_a k_a]
%num = [1]
den = [1 b_a k1+k2+k_a]

x_des = 1.0
x_in = x_des*(k1+k2+k_a)/(k_a)

t = 0:0.01:10
u = x_in*ones(length(t), 1)

sys = tf(num, den)

% figure()
% lsim(sys, u, t)
% [y t] = lsim(sys, u, t)
% figure()
% plot(t, k_tot*y)
% A = [0]
% B = [k_a/(k1+k2+k_a) 1/(k1+k2)]
% %B = k_a/(k1+k2+k_a)
% C = 1
% D = 0


A = [0 0; k1+k2 0]
B = [k_a/(k1+k2+k_a); 0]
%B = k_a/(k1+k2+k_a)
%C = [1 0; 0 1]
C = [1 0; 0 1]
D = 0
sys = ss(A, B, C, D, 0.01)
%step(sys)
%impulse(sys)
