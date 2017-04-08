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
D = data(:,findMRDPLOTindex(names,'m0x') :findMRDPLOTindex(names,'m7z'));
quaternion_world = zeros(size(D,1),4);
quaternion_lander = zeros(size(D,1),4);

R = zeros(3);
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
        
    %% convert the roation matrix to quaternion
    quaternion_world(i,:) = rotm2quat(R); 
    
    %% take into account the suddent changes that are happening
    if i > 1
        if norm(quaternion_world(i,:) - quaternion_world(i-1,:)) > 1
            quaternion_world(i,:) = -quaternion_world(i,:);
        end
    end
    
    %% find the quaternion wrt to assignment frame
    R_lander = R * rot_l_w;
    quaternion_lander(i,:) = rotm2quat(R_lander);
    if i > 1
        if norm(quaternion_lander(i,:) - quaternion_lander(i-1,:)) > 1
            quaternion_lander(i,:) = -quaternion_lander(i,:);
        end
    end
end

