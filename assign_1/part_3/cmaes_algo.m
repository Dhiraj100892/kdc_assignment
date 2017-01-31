function [r, p, y] = cmaes_algo( Target, LinkLength, MinRoll, MaxRoll, MinPitch, MaxPitch, MinYaw, MaxYaw, Obstacles )
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
init_actu_ang = ones(num_link * 3, 1);
for i =  1: num_link
    actu_ang_ll( (i - 1) * 3 + 1 ) = MinRoll(i);
    actu_ang_ll( (i - 1) * 3 + 2 ) = MinPitch(i);
    actu_ang_ll( (i - 1) * 3 + 3 ) = MinYaw(i);
    actu_ang_ul( (i - 1) * 3 + 1 ) = MaxRoll(i);
    actu_ang_ul( (i - 1) * 3 + 2 ) = MaxPitch(i);
    actu_ang_ul( (i - 1) * 3 + 3 ) = MaxYaw(i);
    init_actu_ang( (i - 1) * 3 + 1 ) = ( MinRoll(i) + MaxRoll(i) ) /2 ;
    init_actu_ang( (i - 1) * 3 + 2 ) = ( MinPitch(i) + MaxPitch(i) ) /2 ;
    init_actu_ang( (i - 1) * 3 + 3 ) = ( MinYaw(i) + MaxYaw(i) ) /2 ;
end
obstacles = Obstacles;
vis_color = rand(num_link,3);

%% optimization
%fig = figure()
opts = cmaes;
opts.MaxFunEvals = 1000;
opts.LBounds = actu_ang_ll;
opts.UBounds = actu_ang_ul;
[answer,fval,exitflag, stop, out]=cmaes('dist',init_actu_ang, [],opts, @sphere_intersec);

%vis(answer)

%% output 
r = ones(num_link,1);
p = ones(num_link,1);
y = ones(num_link,1);
for i = 1 : num_link
   r(i) = answer( (i - 1) * 3 + 1 );
   p(i) = answer( (i - 1) * 3 + 2 );
   y(i) = answer( (i - 1) * 3 + 3 );
end

end