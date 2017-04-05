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
[D_com,names,units,freq] = mrdplot_convert('../../data/d00063');
D = D_com(:,findMRDPLOTindex(names,'m0x') :findMRDPLOTindex(names,'m7z'));
quaternion = zeros(size(D,1),4);
R = zeros(3);
% O = 5;           % origin
% X = 7;           % O --> X    x-axis
% Y = 1;           % O --> Y    y-axis
% Z = 6;           % O --> Z    z-axis

O =1;
X=5;
Y=3;
Z=2;

for i = 1: size(D,1)
    % over here I have a diubt .. wheather it should be column or row wise
    tmp = reshape(D(i,:,:),[3,8]);
    R(1,:) = ( tmp(:,X)-tmp(:,O) )/norm(( tmp(:,X)-tmp(:,O) )) ;
    R(2,:) = ( tmp(:,Y)-tmp(:,O) )/norm(( tmp(:,Y)-tmp(:,O) )) ;
    R(3,:) = ( tmp(:,Z)-tmp(:,O) )/norm(( tmp(:,Z)-tmp(:,O) )) ;
   
    %R = R / norm(R);
%     R = R';

    %% convert the roation matrix to quaternion
    quaternion(i,:) = rotm2quat(R);
    R;
    quaternion(i,:);
    
end

plot(quaternion(:,2).*quaternion(:,1));hold on;plot(D_com(:,6).*D_com(:,5));hold off
plot(quaternion(:,3).*quaternion(:,1));hold on;plot(D_com(:,7).*D_com(:,5));hold off
plot(quaternion(:,4).*quaternion(:,1));hold on;plot(D_com(:,8).*D_com(:,5));hold off