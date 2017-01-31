function [ rx ] = rotx( ang )
%finds the rotation matrix given the rotation ang(rad) about the x azis
rx = [1 0 0 0; 0 cos(ang) -sin(ang) 0; 0 sin(ang) cos(ang) 0; 0 0 0 1];
end

