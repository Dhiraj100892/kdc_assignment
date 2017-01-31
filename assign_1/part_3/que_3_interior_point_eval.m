%% desciption


%% init
clc
clear
close all

%% variable declaraion
Target =  [0.8398 2.2293 0.8730 0.5 0.3 0.7 -0.06]';
LinkLength = [1.0, 1.0, 1.0 1.0];
MinRoll = -pi * ones(length(LinkLength),1);
MaxRoll = pi * ones(length(LinkLength),1);
MinPitch = -pi * ones(length(LinkLength),1);
MaxPitch = pi * ones(length(LinkLength),1);
MinYaw = -pi * ones(length(LinkLength),1);
MaxYaw = pi * ones(length(LinkLength),1);
Obstacles = [0.6 0.3 0.3 0.2; 0.2 0.2 0.5 0.1]; 

num_iter = 10;
num_link = length(LinkLength);
cost = [];
tic;
for i = 1 : num_iter
    
    [r, p, y] = interior_point_algo( Target, LinkLength, MinRoll, MaxRoll, MinPitch, MaxPitch, MinYaw, MaxYaw, Obstacles );
    actu_ang = zeros( 3 * num_link, 1);
    for j = 1 : num_link
        actu_ang((j - 1) * 3 + 1) = r(j); 
        actu_ang((j - 1) * 3 + 2) = p(j);
        actu_ang((j - 1) * 3 + 3) = y(j);
    end
    cost = [cost dist(actu_ang)];
end
time_taken = toc;
avg_time_taken = time_taken / num_iter;
mean_cost = mean(cost);

%% TIME = 1.0788
disp('Time Taken = ');
disp(avg_time_taken);

%% MEAN_ERROR = 0.1189
disp('mean_error = ');
disp(mean_cost);