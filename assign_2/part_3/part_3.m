clc;
clear;
close all;

%% init
M = 1.0;            %% Mass of cart
m = 0.1;            %% Mass of pole
l = 1;              %% length of pole
I = (m * l^2) / 12; %% moment of inertia
g = 9.8;            %% acceleration due to gravity


%% forming a system
% after linearizing the model around the vertical position of the pendulum,
% we have...
% X_dot = AX + Bu
% where 
% X = {'x' 'x_dot' 'phi' 'phi_dot'};
p = I*(M+m)+M*m*l^2;
A = [0      1              0           0;
    0      0          (m^2*g*l^2)/p   0;
    0      0              0           1;
    0      0           m*g*l*(M+m)/p  0];

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
R = 0.1;
% K = [0,-3,50,20]

figure(1);
hold on;

t = 0:0.001:50;
axis([0,50,-0.3,0.3])
axis on;
grid on;


%% we tune(d) these ranges manually
for i=0:-1:-2
    for j=-2:-1:-4
        for l=35:10:65
            for m=15:1:20
                K = [i,j,l,m];
                
                sys = ss(A-B*K ,B ,C ,D);
                
                %% open loop response
                u = zeros(size(t));
                x_init =  [0; 0; 0 + pi /10 ; 0];
                [y,t] = lsim(sys,u,t, x_init);
                
                plot(t,y(:,2));
                pause(0.01);
            end
        end
    end
end
%% any answer in those ranges converges, e.g., 
K = [-1,-2,50,20];
sys = ss(A-B*K ,B ,C ,D);
u = zeros(size(t));
x_init =  [0; 0; 0 + pi /10 ; 0];
[y,t] = lsim(sys,u,t, x_init);
plot(t,y(:,2),'r-','linewidth',3);
pause(0.01);

