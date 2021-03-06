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

ang_vel_world = zeros(size(D,1)-1,3);
ang_accel_world = zeros(size(D,1) - 2,3);

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
%% output 
% ang_vel_lander
save('problem_2_2.dat','ang_accel_lander','-ascii');
