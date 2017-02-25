clc;
clear;
close all;

%% init
M = 1.0;            %% Mass of cart
m = 0.1;            %% Mass of pole
l = 1;              %% length of pole

%% 
I = (m * l^2) / 12; %% momment of inertia
g = 9.8;            %% acceleration due to gravity


%% forming a system
% X_dot = AX + Bu  ... after linearing the mdoel around vertical position
% of pendulum
% X = {'x' 'x_dot' 'phi' 'phi_dot'};
p = I*(M+m)+M*m*l^2;
A = [0      1              0           0;
     0      0          (m^2*g*l^2)/p   0;
     0      0              0           1;
     0      0           m*g*l*(M+m)/p  0];
 
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
    
C = [1 0 0 0;
     0 0 1 0];
 
D = [0;
     0];

%% lqr gain
Q = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
R = 0.1;
K = lqr(A,B,Q,R);
 
sys = ss(A-B*K ,B ,C ,D);

%% open loop responce
t = 0:0.001:10;
u = zeros(size(t));
x_init =  [0; 0; 0 + pi /10 ; 0];
[y,t] = lsim(sys,u,t, x_init);
plot(t,y(:,2));


%% close loop control
% %% get the lqt
% t = 0:0.01:1;
% u = zeros(size(t));
% x_init =  [0; 0; pi; 0];
% sys_c = ;
% [y,t] = lsim(sys,u,t, x_init);
% plot(t,y);
