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
[data,names,units,freq] = mrdplot_convert('../data/d00121');
data = data(1:1000,:);
D = data(:,findMRDPLOTindex(names,'m0x') :findMRDPLOTindex(names,'m0x') + 23);

quaternion_world = zeros(size(D,1),4);
ang_vel_world = zeros(size(D,1)-1,3);
ang_accel_world = zeros(size(D,1) - 2,3);

quaternion_lander = zeros(size(D,1),4);
ang_vel_lander = zeros(size(D,1)-1,3);
ang_accel_lander = zeros(size(D,1) - 2,3);

R_world = zeros(3);
R_lander = zeros(3);

R_prev_world = zeros(3);
R_prev_lander = zeros(3);

O = 1;           % origin
X = 5;           % O --> X    x-axis
Y = 3;           % O --> Y    y-axis
Z = 2;           % O --> Z    z-axis

%% rot_lander_world
rot_l_w = [0 -1 0; 1 0 0; 0 0 1];
for i = 1: size(D,1)
    R_world(1,:) = normr( D(i,3*(X-1)+1:3*X) - D(i,3*(O-1) + 1:3*O) ) ;
    R_world(2,:) = normr( D(i,3*(Y-1)+1:3*Y) - D(i,3*(O-1) + 1:3*O) ) ;
    R_world(3,:) = normr( D(i,3*(Z-1)+1:3*Z) - D(i,3*(O-1) + 1:3*O) ) ;
   
    R_world = R_world';
    %% find the quaternion wrt to assignment frame
    R_lander = R_world * rot_l_w;
    
     %% convert the roation matrix to quaternion
    quaternion_world(i,:) = rotm2quat(R_world); 
    
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
        ang_vel_world_m = (R_world - R_prev_world) * freq;
        ang_vel_world_m = transpose(R_prev_world) * ang_vel_world_m;

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
    R_prev_world = R_world;
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
 
 %% predict the future trajectory
 
 %% lander
 fut_quaternion_lander(1,:) = quaternion_lander(end,:);
 fut_ang_vel_lander(1,:) = ang_vel_lander(end,:);
 fut_ang_accel_lander(1,:) = ang_accel_lander(end,:);
 
 %% world
 fut_quaternion_world(1,:) = quaternion_world(end,:);
 fut_ang_vel_world(1,:) = ang_vel_world(end,:);
 fut_ang_accel_world(1,:) = ang_accel_world(end,:);
 
 % iteraret 
 for i = 1: 1005
     
     %% for lander
     % predict r_dot and r(t+1)
     temp_ang_vel_skew_lander = [0 -fut_ang_vel_lander(i,3) fut_ang_vel_lander(i,2);
                                 fut_ang_vel_lander(i,3) 0 -fut_ang_vel_lander(i,1);
                                 -fut_ang_vel_lander(i,2) fut_ang_vel_lander(i,1) 0];
     R_lander_dot = R_lander * temp_ang_vel_skew_lander;
     R_lander = R_lander + R_lander_dot / freq;
     
     % quaternion
     fut_quaternion_lander(i+1,:) = rotm2quat(R_lander);
     if norm(fut_quaternion_lander(i+1,:) - fut_quaternion_lander(i,:)) > 1
         fut_quaternion_lander(i+1,:) = -fut_quaternion_lander(i+1,:);
     end
     % predict w(i+1)
     fut_ang_vel_lander(i+1,:) = fut_ang_vel_lander(i,:) + fut_ang_accel_lander(i,:) / freq;
     
     % predict w_dot(i+1)
     fut_ang_accel_lander(i+1,:) = -(cross(fut_ang_vel_lander(i+1,:)', I_principle*fut_ang_vel_lander(i+1,:)'))'/I_principle;
 
     %% for world
     % predict r_dot and r(t+1)
     temp_ang_vel_skew_world = [0 -fut_ang_vel_world(i,3) fut_ang_vel_world(i,2);
                                 fut_ang_vel_world(i,3) 0 -fut_ang_vel_world(i,1);
                                 -fut_ang_vel_world(i,2) fut_ang_vel_world(i,1) 0];
     R_world_dot = R_world * temp_ang_vel_skew_world;
     R_world = R_world + R_world_dot / freq;
     
     % quaternion
     fut_quaternion_world(i+1,:) = rotm2quat(R_world);
     if norm(fut_quaternion_world(i+1,:) - fut_quaternion_world(i,:)) > 1
         fut_quaternion_world(i+1,:) = -fut_quaternion_world(i+1,:);
     end
     % predict w(i+1)
     fut_ang_vel_world(i+1,:) = fut_ang_vel_world(i,:) + fut_ang_accel_world(i,:) / freq;
     
     % predict w_dot(i+1)
     fut_ang_accel_world(i+1,:) = -(cross(fut_ang_vel_world(i+1,:)', I_principle*fut_ang_vel_world(i+1,:)'))'/I_principle;
 
     %
 end

 %% cm trajectory
 cm_pos_world = [0 0 4.0];
 cm_vel_world = [0.05 0.02 0.01];
 cm_traj_world = zeros(2000,3);
 cm_traj_world(1,:) = cm_pos_world;
 for i = 2:2000
     cm_traj_world(i,:) = cm_traj_world(i-1,:) + cm_vel_world * 0.01; 
 end
 
 %% quaternion
 full_quaternion_world = [quaternion_world;fut_quaternion_world];
 full_quaternion_world = full_quaternion_world(1:2000,:);
 
 %% full_ang_vel
 full_ang_vel_world = [ang_vel_world; fut_ang_vel_world];
 full_ang_vel_world = full_ang_vel_world(1:2000,:);
 
 %% full_ang_accel
 full_ang_accel_world = [ang_accel_world; fut_ang_accel_world];
 full_ang_accel_world = full_ang_accel_world(1:2000,:);
 
 %% saving the data
 data_save = [cm_traj_world full_quaternion_world];
 save('problem_2_3.dat','data_save','-ascii');
 
