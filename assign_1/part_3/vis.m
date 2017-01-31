function [ output_args ] = vis( actu_ang )
% useful for visulazing the result 

%% visulize the links
% get the position of each joint
global link_length vis_color des_pos obstacles 

num_link = length(link_length);
pos = [0 0 0 1];
T = eye(4);
pos_each_joint = [0 0 0 1];

%% calculate each link position in world frame
for i = 1:num_link
    T = T * rotx( actu_ang( (i-1) * 3 + 1 ) ) * roty( actu_ang( (i-1) * 3 + 2 ) ) * rotz( actu_ang( (i-1) * 3 + 3 ) ) * trans(link_length(i));
    pos = T * [0; 0; 0; 1];
    pos_each_joint = [pos_each_joint; pos']; 
end

%% draw each line
drawnow
clf
for i = 1:num_link
    plot3(pos_each_joint(i:i+1,1)',pos_each_joint(i:i+1,2)', pos_each_joint(i:i+1,3)','LineWidth',4,'color', vis_color(i,1:3) );
    hold on
    
end

max_length = sum(link_length);
xlim([-max_length, max_length])
ylim([-max_length, max_length])
zlim([-max_length, max_length])
grid on

%% visualize the sphere
[x,y,z] = sphere;

scale = mean(link_length) / 10.0;
% origin pos
surf(x * scale, y * scale, z * scale, 'FaceColor', 'red', 'EdgeColor', 'red')

% goal pos
surf(x * scale + des_pos(1), y * scale + des_pos(2) , z * scale + des_pos(3), 'FaceColor', 'green', 'EdgeColor','green')

% obstacles
num_obstacle = size(obstacles, 1);
for i = 1: num_obstacle
    surf(x * obstacles(i,4) + obstacles(i,1), y * obstacles(i,4) + obstacles(i,2), z * obstacles(i,4) + obstacles(i,3),'FaceColor','black','EdgeColor','black')
end

end

