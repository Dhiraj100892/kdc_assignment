function [ d_x ] = cart_pole_dynamics( x,u )
%the dynamic symulator fucntion.. given the state and control action .. it
%will tell you what will be the state after time dt
% x = {pos,theta , d_pos, d_theta}
%% simple implemantation.. 
global m_p m_c l g

%syms theta ang_vel pos vel
theta = x(2,:);
ang_vel = x(4,:);
pos = x(1,:);
vel = x(3,:);

d_theta =  ang_vel;
d_ang_vel = ( 1 ./ ( l * (m_c + m_p * sin(theta).^2 ) ) ) .* ( -u .* cos(theta) - m_p * l * (ang_vel.^2 .* cos(theta) .* sin(theta)) - ( m_c + m_p) * g * sin(theta) );
%d_ang_vel = ( 1 ./ ( l * (m_c + m_p * sin(theta).^2 ) ) ) .* ( -u .* cos(theta) - m_p * l * (ang_vel.^2 .* cos(theta) .* sin(theta)) + ( m_c + m_p) * g * sin(theta) );

d_pos = vel;
d_vel =  (1 ./ (m_c + m_p * sin(theta).^2 )) .* (u + m_p * (sin(theta) .* ( l * ang_vel.^2 + g * cos(theta) ) ));
%d_vel =  (1 ./ (m_c + m_p * sin(theta).^2 )) .* (u + m_p * (sin(theta) .* ( l * ang_vel.^2 - g * cos(theta) ) ));


d_x = [d_pos ;d_theta; d_vel; d_ang_vel];


end

