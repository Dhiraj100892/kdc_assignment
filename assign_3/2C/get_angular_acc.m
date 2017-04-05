filename = '../2B/problem_2_1.dat';
angular_velocity =load(filename,'-ascii');
F = length(angular_velocity);
dt = 0.01;
angular_acc = (angular_velocity(2:F,:)-angular_velocity(1:F-1,:))/ dt;