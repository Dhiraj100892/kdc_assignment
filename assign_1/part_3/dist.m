function [ dist ] = dist( actu_ang )
% help to find the dist of des_pos and current pos .. for position ecludian dist is
% calculated .. for orientation dist is calculated using the dot product

global link_length des_pos actu_ang_ll actu_ang_ul obstacles link_vis_clr fig figcounter

%% get th pos for actu_ang
vis(actu_ang);
cur_pos = forward_kine(actu_ang);
%cur_pos = fk(actu_ang);
X = cur_pos(1:3);
q = cur_pos(4:7);
X_d = des_pos(1:3);
q_d = des_pos(4:7);
mean_ang = (actu_ang_ll + actu_ang_ul) / 2;

%% find out the distance
x_dist = (X_d - X)'*(X_d - X);
q_dist = 1 - q_d'*q;
actu_ang_dist = (actu_ang - mean_ang)'* (actu_ang - mean_ang);          %% how far actuation angle deviating from mean position

dist = 2 * x_dist + q_dist + 0.001 * actu_ang_dist;
end