
%% trying to find the angular velocity of moving object by tracking some of its point
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
O = 5;           % origin
X = 7;           % O --> X    x-axis
Y = 1;           % O --> Y    y-axis
Z = 6;           % O --> Z    z-axis
for i = 1: size(marker,1)
    % over here I have a diubt .. wheather it should be column or row wise
    R(1,:) = normr( marker(i,3*(X-1)+1:3*X) - marker(i,3*(O-1)+1:3*O));
    R(2,:) = normr( marker(i,3*(Y-1)+1:3*Y) - marker(i,3*(O-1)+1:3*O));
    R(3,:) = normr( marker(i,3*(Z-1)+1:3*Z) - marker(i,3*(O-1)+1:3*O));
    R = R';
    % do differentiation
    temp = zeros(3);
    if i >= 2
        temp = inv(R_prev) .* ((R - sR_prev) * freq);
%         temp = R' .* ((R - R_prev) * freq);
    end
    ang_vel(i,1) = temp(3,2);
    ang_vel(i,2) = temp(3,1);
    ang_vel(i,3) = temp(2,3);
    R_prev = R;
end
