function [ d_ry ] = d_roty( ang )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
d_ry = [-sin(ang) 0 cos(ang) 0; 0 1 0 0; -cos(ang) 0 -sin(ang) 0; 0 0 0 1];

end

