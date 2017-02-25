function [A_eval, B_eval] = ilqr_linearization(x, u)

% [A, B] = ilqr_linearization(x, u, dt)
%
% linearizes the system in preparation to apply the ilqr techniques.
%
% given	-> x the state of the system at some time
%	-> u the command into the system at some time
%	-> dt the current time step used
%
% returns -> A the current jacobian Dxf(x,u) as defined by todorov
%	  -> B the current jacobian Duf(x,u) as defined by todorov
%
% created by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005
% last edited by timothy lillicrap (tim@biomed.queensu.ca); december 1, 2005
global m_p m_c l g x_sym u_sym d_t A B

x_sym1 = x(1);
x_sym2 = x(2);
x_sym3 = x(3);
x_sym4 = x(4);
u_sym  = u;

A_eval = eval(A);
B_eval = eval(B);