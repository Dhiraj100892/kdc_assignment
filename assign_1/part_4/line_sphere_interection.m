function [ inter ] = line_sphere_interection( P1, P2, X, R )
% check wheather line made from P1 & P2 intersecting the sphere defined by position X and radius defined by R
l = P2 - P1;
l = l / norm(l,2);

d =  ( l' * (P1 - X) )^2 - ( norm((P1 - X),2 ) )^2 + R^2;

if d < 0
    inter = 0;
else
    d_s = -(l' * (P1 - X)) - d ;            %% s small
    d_b = -(l' * (P1 - X)) + d ;            %% b big
   
    p_int_1 = P1 + d_s * l;
    p_int_2 = P2 + d_b * l;
    
    %% for intersection point to lie in sphere compare the ratio = (X - P1)/(P2 - P1)..for intersection point to lie on sphere 0 < ratio < 1
    r_int_1 = (p_int_1(1) - P1(1)) / (P2(1) - P1(1));
    r_int_2 = (p_int_2(1) - P1(1)) / (P2(1) - P1(1));
    
    if (r_int_1 > 0 && r_int_1 < 1) || (r_int_2 >0 && r_int_2 < 1)
        inter = 1000;
    else
        inter = 0;
    end
    
    %% there is no check if the line is completely inside the sphere
    
end

end

