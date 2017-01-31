function [ d_rx ] = d_rotx( ang )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
d_rx = [1 0 0 0; 0 -sin(ang) -cos(ang) 0; 0 cos(ang) -sin(ang) 0; 0 0 0 1];

end

