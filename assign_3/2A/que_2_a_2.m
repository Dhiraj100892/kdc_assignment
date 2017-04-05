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
M = get_the_markers_trajectory('../../data/d00060');

quternion = zeros(size(M,1),4);
R = zeros(3);
O = 5;           % origin
X = 7;           % O --> X    x-axis
Y = 1;           % O --> Y    y-axis
Z = 6;           % O --> Z    z-axis
temp = zeros(size(M,1),3);
for i = 1: size(M,1)
    % over here I have a diubt .. wheather it should be column or row wise
    % R(1,:) = reshape( M(i,X,:) - M(i,O,:), [1,3] ) / norm( reshape( M(i,X,:) - M(i,O,:), [1,3] ) );
    % R(2,:) = reshape( M(i,Y,:) - M(i,O,:), [1,3] ) / norm( reshape( M(i,X,:) - M(i,O,:), [1,3] ) );
    % R(3,:) = reshape( M(i,Z,:) - M(i,O,:), [1,3] ) / norm( reshape( M(i,X,:) - M(i,O,:), [1,3] ) );
    
    R(1,:) = normr(squeeze(M(i,X,:) - M(i,O,:)));
    R(2,:) = normr(squeeze(M(i,Y,:) - M(i,O,:)));
    R(3,:) = normr(squeeze(M(i,Z,:) - M(i,O,:)));

    %% normalize columns
    R(:,1) = R(:,1)/norm(R(:,1));
    R(:,2) = R(:,2)/norm(R(:,2));
    R(:,3) = R(:,3)/norm(R(:,3));
%     temp(i,:) = R(1,:);
    %% convert the roation matrix to quaternion
    quternion(i,:) = rotm2quat(R); 
end

% ouput->
% quternion

