function [ q ] = rot_to_q( rot )
% helpful for converting roation matrix to Q matrix

Rxx = rot(1,1); Rxy = rot(1,2); Rxz = rot(1,3);
Ryx = rot(2,1); Ryy = rot(2,2); Ryz = rot(2,3);
Rzx = rot(3,1); Rzy = rot(3,2); Rzz = rot(3,3);

w = sqrt( trace( rot ) + 1 ) / 2;

if( imag( w ) > 0 )
     w = 0;
end

% x = sqrt( 1 + Rxx - Ryy - Rzz ) / 2;
% y = sqrt( 1 + Ryy - Rxx - Rzz ) / 2;
% z = sqrt( 1 + Rzz - Ryy - Rxx ) / 2;

% [element, i ] = max( [w,x,y,z] );

% if( i == 1 )
    x = ( Rzy - Ryz ) / (4*w);
    y = ( Rxz - Rzx ) / (4*w);
    z = ( Ryx - Rxy ) / (4*w);
% elseif( i == 2 )
%     w = ( Rzy - Ryz ) / (4*x);
%     y = ( Rxy + Ryx ) / (4*x);
%     z = ( Rzx + Rxz ) / (4*x);
% elseif( i == 3 )
%     w = ( Rxz - Rzx ) / (4*y);
%     x = ( Rxy + Ryx ) / (4*y);
%     z = ( Ryz + Rzy ) / (4*y);
% elseif( i == 4 )
%     w = ( Ryx - Rxy ) / (4*z);
%     x = ( Rzx + Rxz ) / (4*z);
%     y = ( Ryz + Rzy ) / (4*z);
% end

q = [ w; x; y; z ];


end