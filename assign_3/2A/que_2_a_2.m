%% trying to find the orientation of object as a function of time

%% concept
% it is given that intially it is completely alligned with the lander
% this infomation is used in selecting the points that should consider ..
% from visualization of the marker position I come to conclusion that X -->
% 5-7, Y-->5-1, Z -->5-6 
%I further used the shape of artifact into consideration to get the 
% orthogonal vectors which will be helpful in defining the rotation matrix 

clc;
clear;
close all;

%% concept (we r using CM location and its linear velocity from previous que)
% V_centroid = V_CM + W x r_(centroid_from_CM)   [W is angular velocity]
% 2nd method
%w = r_(centroid_from_CM) x (V_centroid - V_CM) / |r_(centroid_from_CM)|^2
[D,names,units,freq] = mrdplot_convert('../data/d00060');
marker = D( :, findMRDPLOTindex(names,'ml0x'):findMRDPLOTindex(names,'ml0x') + 23);
num_markers = 8;

%% find the rotation matrix
% calculate the rotation matrix for all the time ste

rotation = zeros(size(marker,1),3,3);
diff_rotation = zeros(size(marker,1)-1,3,3);
ang_vel = zeros(size(marker,1)-1,3);
R = zeros(3);
R_prev = zeros(3);
%%% for lander coordinate frame case

O = 5;           % origin
X = 7;           % O --> X    x-axis
Y = 1;           % O --> Y    y-axis
Z = 6;           % O --> Z    z-axis


%%% for global coordinate frame case

% O = 1;           % origin
% X = 5;           % O --> X    x-axis
% Y = 3;           % O --> Y    y-axis
% Z = 2;           % O --> Z    z-axis

quaternion = zeros(size(D,1),4);
for i = 1: size(marker,1)
    % over here I have a diubt .. wheather it should be column or row wise
    R(:,1) = normr( marker(i,3*(X-1)+1:3*X) - marker(i,3*(O-1)+1:3*O))';
    R(:,2) = normr( marker(i,3*(Y-1)+1:3*Y) - marker(i,3*(O-1)+1:3*O))';
    R(:,3) = normr( marker(i,3*(Z-1)+1:3*Z) - marker(i,3*(O-1)+1:3*O))';
    % do differentiation
    quaternion(i,:) = rotm2quat(R);
end

plot(quaternion(:,2).*quaternion(:,1));hold on;plot(D_com(:,6).*D_com(:,5));hold off
plot(quaternion(:,3).*quaternion(:,1));hold on;plot(D_com(:,7).*D_com(:,5));hold off
plot(quaternion(:,4).*quaternion(:,1));hold on;plot(D_com(:,8).*D_com(:,5));hold off