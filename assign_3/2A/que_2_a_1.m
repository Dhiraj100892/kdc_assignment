%% trying to find the center of mass location of the artifact given the co-ordinates of point regidly attach to the body

%% concept applied
% location of center of mass will be point in a body whose motion is
% minimum wrt to the marker points

%% init
clc;
clear;
close all;

%% get the values of markers
global M
global NF
M = get_the_markers_trajectory('../data/d00060');
NF = 1000;
%% define the intial guess for location as weel as velocity
CM = zeros(6,1);

%% do optimization
options = optimset('MaxFunEvals',1000000);
[answer,fval,exitflag]=fminunc(@cm_criterion,CM,options);

%% result
CM_pos = answer(1:3)
CM_vel = answer(4:6)


%% answers
% CM_pos =
% 
%   -11.0000
%     0.0000
%    -0.0000
% 
% 
% CM_vel =
% 
%     0.0200
%    -0.0500
%     0.0100

CM_traj = zeros(NF,3);
CM_traj(1,:) = CM_pos;
for i = 2:NF
    CM_traj(i,:) = CM_traj(i-1,:) + CM_vel' * 0.01; 
end
save('CM_traj.dat', 'CM_traj','-ascii') 