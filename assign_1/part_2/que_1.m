%% init
clear
close all
% clc

%% desciption

%% var (ll-> lower limit ul-> upper limit)
global link_length des_pos actu_ang_ll actu_ang_ul obstacles vis_color fig d_fk sym_actu_ang

link_length = [1 1 1];
des_pos = [ 0.8398 2.2293 0.8730 0.707 -0.707 0 0]' ;
num_link = length(link_length);
actu_ang_ll = -pi * ones(num_link * 3 , 1); 
actu_ang_ul = pi * ones(num_link * 3 , 1);
obstacles = [0.6 0.2 0.6 0.2; 0.2 0.2 0.5 0.1; 0.2 0.8 0.5 0.1; 0.8 0.9 0.1 0.1];
vis_color = rand(num_link,3);

% init_actu_ang = -pi * ones(num_link*3,1) + 2*pi*rand(num_link*3,1);
init_actu_ang = zeros(num_link*3,1);

sym_actu_ang = sym('X',[num_link*3,1]);

d_fk = jacobian(forward_kine(sym_actu_ang),sym_actu_ang);

%d_fk
%dist(sym_actu_ang)
%% optimization
fig = figure()
options = optimset('Display','iter','MaxFunEvals',1000,'Algorithm','interior-point');
[answer,fval,exitflag]=fmincon(@dist,init_actu_ang, [], [], [], [], actu_ang_ll, actu_ang_ul, @sphere_intersec,options);

vis(answer)