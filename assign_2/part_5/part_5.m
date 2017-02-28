clc;
clear;
close all;

%% init
M = 1.0;            %% Mass of cart
m = 0.1;            %% Mass of pole
l = 1;              %% length of pole

%% 
I = (m * l^2) / 12; %% momment of inertia
g = 9.8;            %% acceleration due to gravity


%% forming a system
% X_dot = AX + Bu  ... after linearing the mdoel around vertical position
% of pendulum
% X = {'x' 'x_dot' 'phi' 'phi_dot'};
p = I*(M+m)+M*m*l^2;
A = [0      1              0           0;
     0      0          (m^2*g*l^2)/p   0;
     0      0              0           1;
     0      0          m*g*l*(M+m)/p  0];
 
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
    
C = [1 0 0 0;
     0 0 1 0];
 
D = [0;
     0];

%% lqr gain
Q = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
%R = 0.1;

%Rs = 0.5: 0.5 : 5; 
Rs = [0.01 0.02 0.05 0.1 0.15 0.2 0.5 1 1.5 2 3:0.5:5]; 

percentage = zeros(numel(Rs), 1); 
for ri = 1:numel(Rs)
  R = Rs(ri); 
  
  K = lqr(A,B,Q,R);
 
  sys = ss(A-B*K ,B ,C ,D);
  
  %% open loop responce
  t = 0:1:10;
  u = zeros(size(t));
  
  x_n = 20;
  x_min = -1;
  x_max = 1;
  x_step = (x_max - x_min)/(x_n-1);
  xs = x_min : x_step : x_max;
  
  theta_n = 20;
  theta_min = -pi;
  theta_max = pi;
  theta_step = (theta_max - theta_min)/(theta_n-1);
  thetas = theta_min : theta_step : theta_max;
  
  ytol = 0.01;
  
  flags = false(numel(xs), numel(thetas));
  for i = 1:numel(xs)
    for j = 1:numel(thetas)
      x_init = xs(i);
      theta_init = thetas(j);
      state_init =  [x_init; 0; theta_init; 0];
      [y,t] = lsim(sys,u,t,state_init);
      %plot(t,y(:,2));
      flags(i,j) = all( norm(y(end,:)) < ytol );
      %     hold on;
    end
  end
  
  imagesc(flags); 
  axis image; 
  colormap(gray);
  caxis([0 1]); 
  colorbar; 
  
  set(gca, 'YTick', 1:2:x_n);
  set(gca, 'YTickLabel', sprintfc('%.2f', xs(1:2:end)));
  set(gca, 'XTick', 1:2:theta_n);
  set(gca, 'XTickLabel', sprintfc('%.2f', thetas(1:2:end)));
  title(sprintf('R: %.2f', Rs(ri)));
  print('-dpng', sprintf('basin_R%.2f.png', Rs(ri)));
  
  percentage(ri) = mean(flags(:)==1);
  
%   fprintf('R: %f, percentage: %f\n', Rs(ri), percentage(ri));
end

plot(Rs, percentage); 
print('-dpng', 'basin_R.png');

% imshow(flags);

%% close loop control
% %% get the lqt
% t = 0:0.01:1;
% u = zeros(size(t));
% x_init =  [0; 0; pi; 0];
% sys_c = ;
% [y,t] = lsim(sys,u,t, x_init);
% plot(t,y);
