function [ ry ] = roty( ang )
%finds the rotation matrix given the rotation ang(rad) about the y azis
ry = [cos(ang) 0 sin(ang) 0; 0 1 0 0; -sin(ang) 0 cos(ang) 0; 0 0 0 1];


end

