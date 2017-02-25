function [ A,B ] = cart_pole_linearize_system( )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
global m_p m_c l g x_sym u_sym d_t

%x_new = zeros(4,1);

x_new(1) = x_sym(1) + d_t * x_sym(2);
% x_new(2) = x_sym(2) +  d_t * ( 1 / ( l * (m_c + m_p * sin(x_sym(1))^2 ) ) ) * ( -u_sym * cos(x_sym(1)) - m_p * l * x_sym(2)^2 * cos(x_sym(1)) * sin(x_sym(1)) - ( m_c + m_p) * g * sin(x_sym(1)) );
x_new(2) = x_sym(2) +  d_t  * ( 1 / ( l * (m_c + m_p * sin(x_sym(1))^2 ) ) ) * ( - u_sym * cos(x_sym(1)) - m_p * l * x_sym(2)^2 * cos(x_sym(1)) * sin(x_sym(1)) + ( m_c + m_p) * g * sin(x_sym(1)) );

x_new(3) = x_sym(3) + d_t * x_sym(4);
% x_new(4) = x_sym(4) + d_t * (1 / (m_c + m_p * sin(x_sym(1))^2 )) * (u_sym + m_p * sin(x_sym(1)) * ( l * x_sym(2)^2 + g * cos(x_sym(1)) ) );
x_new(4) = x_sym(4) + d_t * (1 / (m_c + m_p * sin(x_sym(1))^2 )) * (u_sym + m_p * sin(x_sym(1)) * ( l * x_sym(2)^2 - g * cos(x_sym(1)) ) );

% A = zeros(4,4);
% B = zeros(4,1);

for i = 1:4
    for j = 1:4
        A(i,j) = diff(x_new(i),x_sym(j));
    end
    B(i,1) = diff(x_new(i), u_sym);
end


end

