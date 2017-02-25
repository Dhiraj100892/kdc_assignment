function [ new_x ] = cart_pole_system( x,u,dt )
%the dynamic symulator fucntion.. given the state and control action .. it
%will tell you what will be the state after time dt
% x = {theta, d_theta, pos, d_pos}
%% simple implemantation.. 
global m_p m_c l g
%syms theta ang_vel pos vel
theta = x(1);
ang_vel = x(2);
pos = x(3);
vel = x(4);

new_theta = theta + dt * ang_vel;
% new_ang_vel = ang_vel +  dt * ( 1 / ( l * (m_c + m_p * sin(theta)^2 ) ) ) * ( -u * cos(theta) - m_p * l * ang_vel^2 * cos(theta) * sin(theta) - ( m_c + m_p) * g * sin(theta) );
new_ang_vel = ang_vel +  dt * ( 1 / ( l * (m_c + m_p * sin(theta)^2 ) ) ) * ( -u * cos(theta) - m_p * l * ang_vel^2 * cos(theta) * sin(theta) + ( m_c + m_p) * g * sin(theta) );

new_pos = pos + dt * vel;
% new_vel = vel + dt * (1 / (m_c + m_p * sin(theta)^2 )) * (u + m_p * sin(theta) * ( l * ang_vel^2 + g * cos(theta) ) );
new_vel = vel + dt * (1 / (m_c + m_p * sin(theta)^2 )) * (u + m_p * sin(theta) * ( l * ang_vel^2 - g * cos(theta) ) );

new_x = [new_theta new_ang_vel new_pos new_vel];


end

