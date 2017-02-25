%%using the trajectory optimization code written by

%%
clc
clear
close all
addpath optimTrajLib/

%% for systrem
global m_p m_c l g
m_p = 0.1;
m_c = 1;
l = 1.0;
g = 9.8;

%% 
dist = 1.0;  %How far must the cart translate during its swing-up
maxForce = 50;  %Maximum actuator forces



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( cart_pole_dynamics(x,u) );
problem.func.pathObj = @(t,x,u)( ones(size(t)) ); 

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0.01;
problem.bounds.finalTime.upp = inf;

problem.bounds.initialState.low = [0;0;0;0];
problem.bounds.initialState.upp = [0;0;0;0];
problem.bounds.finalState.low = [dist;pi;0;0];
problem.bounds.finalState.upp = [dist;pi;0;0];

problem.bounds.state.low = [-2*dist;-2*pi;-inf;-inf];
problem.bounds.state.upp = [2*dist;2*pi;inf;inf];

problem.bounds.control.low = -maxForce;
problem.bounds.control.upp = maxForce;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.time = [0,2];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [0,0];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options(1).nlpOpt = optimset(...
    'Display','iter',...
    'TolFun',1e-3,...
    'MaxFunEvals',1e5);
problem.options(1).method = 'trapezoid';
problem.options(1).trapezoid.nGrid = 10;

problem.options(2).nlpOpt = optimset(...
    'Display','iter',...
    'TolFun',1e-6,...
    'MaxFunEvals',1e5);
problem.options(2).method = 'trapezoid';
problem.options(2).trapezoid.nGrid = 30;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);

%%%% Unpack the simulation
t = linspace(soln(end).grid.time(1), soln(end).grid.time(end), 150);
z = soln(end).interp.state(t);
u = soln(end).interp.control(t);

% z(1,:) = pos | z(2,:) = theta  
plot(t,z(2,:),'b')
plot(t,u,'r')



