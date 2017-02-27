clear
clc
syms x theta x_dot theta_dot x_ddot theta_ddot f 'real';

% M = 1;
% m = 0.1;
% L = 1;
% g = 9.8;

syms M m L g 'real';

q = [x,theta];
q_dot = [x_dot, theta_dot];
q_ddot = [x_ddot, theta_ddot];

% % lagrangian
l = (1/2)*M*(x_dot^2) + (1/2)*m*((x_dot + L*cos(theta)*theta_dot)^2 + (-L*sin(theta)*theta_dot)^2) - m*g*L*cos(theta);

dl_dq = jacobian(l, q);
dl_dq_dot = jacobian(l, q_dot);
ddl_dtdq_dot = jacobian(dl_dq_dot,[q, q_dot]) * [q_dot, q_ddot]';


equ = [f, 0]' == ddl_dtdq_dot - dl_dq' ;

equ(1)

