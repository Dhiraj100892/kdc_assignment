%% init
clear
close all
clc

%% desciption

%% var (ll-> lower limit ul-> upper limit)
global other_sol vis_color
Target =  [0.8398 2.2293 0.8730 0.5 0.3 0.7 -0.06]';
LinkLength = [1.0, 1.0, 1.0 1.0];
MinRoll = -pi * ones(length(LinkLength),1);
MaxRoll = pi * ones(length(LinkLength),1);
MinPitch = -pi * ones(length(LinkLength),1);
MaxPitch = pi * ones(length(LinkLength),1);
MinYaw = -pi * ones(length(LinkLength),1);
MaxYaw = pi * ones(length(LinkLength),1);
Obstacles = [0.6 0.3 0.3 0.2; 0.2 0.2 0.5 0.1]; 

other_sol = [];
num_sol_req = 5;
pos_diff_thr = 0.1; 
num_link = length(LinkLength);
vis_color = rand(num_link,3);
%% find no of solutions
i = 0;
while i < num_sol_req
    
    [r, p, y] = part4( Target, LinkLength, MinRoll, MaxRoll, MinPitch, MaxPitch, MinYaw, MaxYaw, Obstacles );
    for j = 1 : num_link
        actu_ang((j - 1) * 3 + 1) = r(j); 
        actu_ang((j - 1) * 3 + 2) = p(j);
        actu_ang((j - 1) * 3 + 3) = y(j);
    end
    % find dist
    end_effector_pos = forward_kine(actu_ang);
    end_effector_X =end_effector_pos(1:3);
    end_effector_q =end_effector_pos(4:7);
    cost = (Target(1:3) - end_effector_X(1:3))'*(Target(1:3) - end_effector_X(1:3)) + (end_effector_q - Target(4:7))'*(end_effector_q - Target(4:7));
    if cost < pos_diff_thr
        other_sol = [other_sol; actu_ang ];
        fig = figure();
        vis(actu_ang)
        i = i + 1;
    end
end
