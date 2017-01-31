function [ d_rz ] = d_rotz( ang )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

d_rz = [-sin(ang) -cos(ang) 0 0; cos(ang) -sin(ang) 0 0; 0 0 1 0; 0 0 0 1];

end

