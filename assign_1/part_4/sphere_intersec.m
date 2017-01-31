function [ineq_violations,eq_violations] = sphere_intersec( actu_ang )
% function useful to find the is there intersection between the sphere with
% the robot arm config defines by the actu_ang


global link_length obstacles
num_link = length(link_length);
num_obstacles = size(obstacles,1);          %% obstacle is N x 4
pos = [0 0 0 1];
T = eye(4);
pos_each_joint = [0 0 0 1];
count = 1;

%% calculate each link position in world frame
for i = 1:num_link
    T = T * rotx( actu_ang( (i-1) * 3 + 1 ) ) * roty( actu_ang( (i-1) * 3 + 2 ) ) * rotz( actu_ang( (i-1) * 3 + 3 ) ) * trans(link_length(i));
    pos = T * [0; 0; 0; 1];
    pos_each_joint = [pos_each_joint; pos']; 
    P2 = pos(1:3);
    P1 = pos_each_joint(i,1:3)';
    %% check for each intersection
    for j = 1: num_obstacles
        eq_violations(count) = line_sphere_interection( P1, P2, obstacles(j,1:3)', obstacles(j,4) );
        count = count + 1;
    end
    
end

ineq_violations = [];


end

