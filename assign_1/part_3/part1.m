function [r, p, y] = part1( Target, LinkLength, MinRoll, MaxRoll, MinPitch, MaxPitch, MinYaw, MaxYaw, Obstacles )
%% Function that uses optimization to do inverse kinematics for a snake robot

%%Outputs 
  % [r, p, y] = roll, pitch, yaw vectors of the N joint angles
  %            (N link coordinate frames)
%%Inputs:
    % target: [x, y, z, q0, q1, q2, q3]' position and orientation of the end
    %    effector
    % link_length : Nx1 vectors of the lengths of the links
    % min_xxx, max_xxx are the vectors of the 
    %    limits on the roll, pitch, yaw of each link.
    % limits for a joint could be something like [-pi, pi]
    % obstacles: A Mx4 matrix where each row is [ x y z radius ] of a sphere
    %    obstacle. M obstacles.

global link_length des_pos actu_ang_ll actu_ang_ul obstacles vis_color fig

link_length = LinkLength;
des_pos = Target;
num_link = length(link_length);
actu_ang_ll = ones(num_link * 3 , 1); 
actu_ang_ul = ones(num_link * 3 , 1);
for i =  1: num_link
    actu_ang_ll( (i - 1) * 3 + 1 ) = MinRoll(i);
    actu_ang_ll( (i - 1) * 3 + 2 ) = MinPitch(i);
    actu_ang_ll( (i - 1) * 3 + 3 ) = MinYaw(i);
    actu_ang_ul( (i - 1) * 3 + 1 ) = MaxRoll(i);
    actu_ang_ul( (i - 1) * 3 + 2 ) = MaxPitch(i);
    actu_ang_ul( (i - 1) * 3 + 3 ) = MaxRoll(i);
    
end
obstacles = [0.6 0.2 0.6 0.2; 0.2 0.2 0.5 0.1; 0.2 0.8 0.5 0.1; 0.8 0.9 0.1 0.1];
vis_color = rand(num_link,3);


init_actu_ang = -pi * ones(num_link*3,1) + 2*pi*rand(num_link*3,1);

%% optimization
fig = figure()
options = optimset('Display','iter','MaxFunEvals',1000,'Algorithm','sqp');
[answer,fval,exitflag]=fmincon(@dist,init_actu_ang, [], [], [], [], actu_ang_ll, actu_ang_ul, @sphere_intersec,options);

vis(answer)

%% output 
r = ones(num_link,1);
p = ones(num_link,1);
y = ones(num_link,1);
for i = 1 : num_link
   r = answer( (i - 1) * 3 + 1 );
   p = answer( (i - 1) * 3 + 2 );
   y = answer( (i - 1) * 3 + 3 );
end

end