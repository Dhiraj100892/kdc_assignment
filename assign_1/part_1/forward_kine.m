function [ pos ] = forward_kine( actu_ang )
%useful to calculate the pos of end effector given link length and
%actuation angle(order-> x,y,z)... (Nx3,1) .. in radian
% issues -> for link_len = [1 1] & act_ang = [0 0 deg2rad(90); 0 0
% deg2rad(90)]--> infinite qw becomes zero

%% intialization
global link_length;
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

%% take out the first 3 coordinates
pos_each_joint = pos_each_joint(1:num_link + 1, 1:3);
rot = T(1:3,1:3);
pos = pos_each_joint(num_link + 1, 1:3);

%% get the orientation in quaternion
% formula is copioed from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
qw = sqrt(1.0 + rot(1,1) + rot(2,2) + rot(3,3)) / 2.0;
qx = ( rot(3,2) - rot(2,3) ) / ( 4.0 * qw );
qy = ( rot(1,3) - rot(3,1) ) / ( 4.0 * qw );
qz = ( rot(2,1) - rot(1,2) ) / ( 4.0 * qw );

%% pos of endeffector
pos = [pos qw qx qy qz]';


end

