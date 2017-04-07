filename = '../2A/problem_2_0.dat';
pose =load(filename,'-ascii');
T = length(pose);
% R = zeros(T,3,3);
dt = 0.01;
R_prev = quat2rotm(pose(1,4:7));
angular_velocity = zeros(T,3);
for i=2:T
    R = quat2rotm(pose(i,4:7));
    R_dot = (R - R_prev)/dt;
    % tmp = R_dot* inv(R);
    tmp = R'*R_dot;			%% guy in the lab
    angular_velocity(i,:) = [tmp(3,2), tmp(1,3),tmp(2,1)];
end