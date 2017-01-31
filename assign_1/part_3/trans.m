function [ T ] = trans( len )
% find the translation matrix considering the translation of link_len alog
% X axis
T = eye(4);
T(1,4) = len;

end

