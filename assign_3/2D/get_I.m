clear all
clc

%% load data
filename = '../2B/problem_2_1.dat';
w =load(filename,'-ascii');
filename = '../2C/problem_2_2.dat';
w_dot =load(filename,'-ascii');
%% make equations
% eq1 = I*angular_acc(2,:)' + cross(angular_velocity(2,:)', I*angular_velocity(2,:)')==zeros(3,1);
% eq2 = I*angular_acc(3,:)' + cross(angular_velocity(3,:)', I*angular_velocity(3,:)')==zeros(3,1);
choose_idx = 50:10:1000;
% A = [w_dot(choose_idx,1), (w_dot(choose_idx,2) - w(choose_idx,3).*w(choose_idx,1)), (w_dot(choose_idx,3) + w(choose_idx,3).*w(choose_idx,1)), - w(choose_idx,2).*w(choose_idx,3), (w(choose_idx,2).^2 - w(choose_idx,3).^2), w(choose_idx,2).*w(choose_idx,3);
%     w(choose_idx,1).*w(choose_idx,3), (w_dot(choose_idx,1) + w(choose_idx,2).*w(choose_idx,3)), (w(choose_idx,3).^2 - w(choose_idx,1).^2), w_dot(choose_idx,2), (w_dot(choose_idx,3) - w(choose_idx,2).*w(choose_idx,1)), -w(choose_idx,1).*w(choose_idx,3);
%     -w(choose_idx,1).*w(choose_idx,2), (w(choose_idx,1).^2 - w(choose_idx,2).^2), (w_dot(choose_idx,1) - w(choose_idx,2).*w(choose_idx,3)), w(choose_idx,1).*w(choose_idx,2), (w_dot(choose_idx,2) + w(choose_idx,1).*w(choose_idx,3)), w_dot(choose_idx,3)];

A = [w_dot(choose_idx,1), (w_dot(choose_idx,2) - w(choose_idx,3).*w(choose_idx,1)), (w_dot(choose_idx,3) + w(choose_idx,3).*w(choose_idx,1)), - w(choose_idx,2).*w(choose_idx,3), (w(choose_idx,2).^2 - w(choose_idx,3).^2), w(choose_idx,2).*w(choose_idx,3);
    w(choose_idx,1).*w(choose_idx,3), (w_dot(choose_idx,1) + w(choose_idx,2).*w(choose_idx,3)), (w(choose_idx,3).^2 - w(choose_idx,1).^2), w_dot(choose_idx,2), (w_dot(choose_idx,3) - w(choose_idx,2).*w(choose_idx,1)), -w(choose_idx,1).*w(choose_idx,3);
    -w(choose_idx,1).*w(choose_idx,2), (w(choose_idx,1).^2 ), (w_dot(choose_idx,1) - w(choose_idx,2).*w(choose_idx,3)), w(choose_idx,1).*w(choose_idx,2), (w_dot(choose_idx,2) + w(choose_idx,1).*w(choose_idx,3)), w_dot(choose_idx,3)];


tmp = A'*A;
[U,S,V] = svd(tmp);
I = [V(6,1:3);
    V(6,2), V(6,4), V(6,5);
    V(6,3), V(6,5), V(6,6)]
