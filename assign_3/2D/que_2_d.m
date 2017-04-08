%% trying to find the orientation of object as a function of time

%% concept
% it is given that intially it is completely alligned with the lander
% this infomation is used in selecting the points that should consider ..
% from visualization of the marker position I come to conclusion that X -->
% 5-7, Y-->5-1, Z -->5-6 
%I further used the shape of artifact into consideration to get the 
% orthogonal vectors which will be helpful in defining the rotation matrix 

%% init
clc;
clear;
close all;


%% calculate the rotation matrix for all the time step
[data,names,units,freq] = mrdplot_convert('../data/d00060');
data = data(1:1000,:);
D = data(:,findMRDPLOTindex(names,'m0x') :findMRDPLOTindex(names,'m7z'));

quaternion_world = zeros(size(D,1),4);
ang_vel_world = zeros(size(D,1)-1,3);
ang_accel_world = zeros(size(D,1) - 2,3);

quaternion_lander = zeros(size(D,1),4);
ang_vel_lander = zeros(size(D,1)-1,3);
ang_accel_lander = zeros(size(D,1) - 2,3);

R = zeros(3);
R_lander = zeros(3);

R_prev = zeros(3);
R_prev_lander = zeros(3);

O = 1;           % origin
X = 5;           % O --> X    x-axis
Y = 3;           % O --> Y    y-axis
Z = 2;           % O --> Z    z-axis

%% rot_lander_world
rot_l_w = [0 -1 0; 1 0 0; 0 0 1];
for i = 1: size(D,1)
    R(1,:) = normr( D(i,3*(X-1)+1:3*X) - D(i,3*(O-1) + 1:3*O) ) ;
    R(2,:) = normr( D(i,3*(Y-1)+1:3*Y) - D(i,3*(O-1) + 1:3*O) ) ;
    R(3,:) = normr( D(i,3*(Z-1)+1:3*Z) - D(i,3*(O-1) + 1:3*O) ) ;
   
    R = R';
    %% find the quaternion wrt to assignment frame
    R_lander = R * rot_l_w;
    
     %% convert the roation matrix to quaternion
    quaternion_world(i,:) = rotm2quat(R); 
    
    %% take into account the suddent changes that are happening
    if i > 1
        if norm(quaternion_world(i,:) - quaternion_world(i-1,:)) > 1
            quaternion_world(i,:) = -quaternion_world(i,:);
        end
    end
    
    %% find the quaternion wrt to assignment frame
    quaternion_lander(i,:) = rotm2quat(R_lander);
    if i > 1
        if norm(quaternion_lander(i,:) - quaternion_lander(i-1,:)) > 1
            quaternion_lander(i,:) = -quaternion_lander(i,:);
        end
    end
    
    if (i > 1)
        
        %% wrt world
        ang_vel_world_m = (R - R_prev) * freq;
        ang_vel_world_m = transpose(R_prev) * ang_vel_world_m;

        ang_vel_world(i-1,1) = ang_vel_world_m(3,2,1);
        ang_vel_world(i-1,2) = ang_vel_world_m(1,3,1);
        ang_vel_world(i-1,3) = ang_vel_world_m(2,1,1);
        
        %% wrt lander
        ang_vel_lander_m = (R_lander - R_prev_lander) * freq;
        ang_vel_lander_m = transpose(R_prev_lander) * ang_vel_lander_m;

        ang_vel_lander(i-1,1) = ang_vel_lander_m(3,2,1);
        ang_vel_lander(i-1,2) = ang_vel_lander_m(1,3,1);
        ang_vel_lander(i-1,3) = ang_vel_lander_m(2,1,1);
    end
    
    %% assign prev_r
    R_prev = R;
    R_prev_lander = R_lander;
end

%% finite difference method
for i = 1: size(D,1) - 2
    ang_accel_world(i,:) = (ang_vel_world(i+1,:) - ang_vel_world(i,:)) * freq; 
    ang_accel_lander(i,:) = (ang_vel_lander(i+1,:) - ang_vel_lander(i,:)) * freq;
end

ang_accel_world(:,1) = smooth(ang_accel_world(:,1),10);
ang_accel_world(:,2) = smooth(ang_accel_world(:,2),10);
ang_accel_world(:,3) = smooth(ang_accel_world(:,3),10);

ang_accel_lander(:,1) = smooth(ang_accel_lander(:,1),10);
ang_accel_lander(:,2) = smooth(ang_accel_lander(:,2),10);
ang_accel_lander(:,3) = smooth(ang_accel_lander(:,3),10);

%% moment of inertia
%Solve for Inertia Tensor
A = [];
for i=1:50:1000
    inter_A = [ang_accel_lander(i,1) ang_accel_lander(i,2)-ang_vel_lander(i,1)*ang_vel_lander(i,3) ang_accel_lander(i,3)+ang_vel_lander(i,1)*ang_vel_lander(i,3) -ang_vel_lander(i,2)*ang_vel_lander(i,3) ang_vel_lander(i,2)^2-ang_vel_lander(i,3)^2 ang_vel_lander(i,2)*ang_vel_lander(i,3);
         ang_vel_lander(i,1)*ang_vel_lander(i,3) ang_accel_lander(i,1)+ang_vel_lander(i,2)*ang_vel_lander(i,3) ang_vel_lander(i,3)^2-ang_vel_lander(i,1)^2 ang_accel_lander(i,2) ang_accel_lander(i,3)-ang_vel_lander(i,1)*ang_vel_lander(i,2) -ang_vel_lander(i,1)*ang_vel_lander(i,3);
         -ang_vel_lander(i,1)*ang_vel_lander(i,2) ang_vel_lander(i,1)^2 ang_accel_lander(i,1)-ang_vel_lander(i,2)*ang_vel_lander(i,3) ang_vel_lander(i,1)*ang_vel_lander(i,2) ang_accel_lander(i,2)+ang_vel_lander(i,1)*ang_vel_lander(i,3) ang_accel_lander(i,3)];
    A = [A;inter_A];
end
[U,S,V] = svd(A);

%% Build Inertia Matrix
I = [V(1,6) V(2,6) V(3,6);
     V(2,6) V(4,6) V(5,6);
     V(3,6) V(5,6) V(6,6)];
 
 %% principle axis
 I_principle = eye(3) .* I;
 I_principle(1,1) = I(2,2);
 I_principle(2,2) = I(1,1);
