function [ rz ] = rotz( ang )
%finds the rotation matrix given the rotation ang(rad) about the z azis
rz = [cos(ang) -sin(ang) 0 0; sin(ang) cos(ang) 0 0; 0 0 1 0; 0 0 0 1];


end

