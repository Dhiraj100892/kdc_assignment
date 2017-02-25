%% trying to write the dynamics of the 

%% 
clc
%clear
close all

%% init
global m_p m_c l g x_sym u_sym d_t A B
x_sym = sym('x_sym',[4,1]);
u_sym = sym('u_sym');

m_p = 0.1;
m_c = 1;
l = 1;
g = 9.8;

%%
x_init = [pi 0 0 0];     %% {theta, d_theta, pos, d_pos}
d_t = 0.001;
t_f = 2;

%% linear system A and B.. this is the function of states
[A,B] = cart_pole_linearize_system();

%% control param
Q = zeros(4);
Q(1,1) = 0.0;
Q_f = 10 * eye(4);
Q_f(2,2) = 0.5;
Q_f(3,3) = 0;
Q_f(4,4) = 0.1;
R =  0.05* d_t;
x_f = [0 0 0 0];

t = 0 : d_t: t_f;
x = zeros(length(t),4);
u = zeros(length(t)-1 ,1);
%current_u = load('u_3.mat');
%u = current_u.current_u;
x(1,:) = x_init;

%% for num of iteration
current_u = u;
cost = ilqr_cost(current_u, x_init, x_f, Q_f, Q, R, d_t);
disp(cost)
figure

%hold on
max_num_iter = 500;
for num_iter = 1:max_num_iter
    traj = ilqr_openloop(x_init,current_u, d_t);
    subplot(3,1,1);
    plot(t,traj(:,1))
    grid on
    
    subplot(3,1,2);
    plot(t,traj(:,2))
    grid on
    
    subplot(3,1,3);
    plot(t,traj(:,4))
    
    grid on
    pause(0.00001);

	du = ilqr_iteration(current_u, x_init, x_f, Q_f, Q, R, d_t);	
	current_u = current_u + 0.5 * du;
	
	newcost = ilqr_cost(current_u, x_init, x_f, Q_f, Q, R, d_t);
		
	disp(newcost)    
    
end
% new_u = current_u