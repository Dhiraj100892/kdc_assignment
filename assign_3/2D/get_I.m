% w_dot = sym('w_dot',[3,1]);
% w = sym('w',[3,1]);
% vars = sym('I', [6,1 ]);
% I = [vars(1:3)';
%     vars(2),var(4),var(5);
%     vars(3),var(5),var(6)];

%% load data
filename = '../2B/problem_2_1.dat';
w =load(filename,'-ascii');
filename = '../2C/problem_2_2.dat';
w_dot =load(filename,'-ascii');
%% make equations
% eq1 = I*angular_acc(2,:)' + cross(angular_velocity(2,:)', I*angular_velocity(2,:)')==zeros(3,1);
% eq2 = I*angular_acc(3,:)' + cross(angular_velocity(3,:)', I*angular_velocity(3,:)')==zeros(3,1);

A = [w_dot(:,1), (w_dot(:,2) - w(:,3).*w(:,1)), (w_dot(:,3) + w(:,2).*w(:,1)), - w(:,2).*w(:,3), (w(:,2).^2 - w(:,3).^2), w(:,2).*w(:,3);
    w(:,1).*w(:,3), (w_dot(:,1) + w(:,2).*w(:,3)), (w(:,3).^2 - w(:,1).^2), w_dot(:,2), (w_dot(:,3) - w(:,2).*w(:,1)), -w(:,1).*w(:,3);
    -w(:,1).*w(:,2), (w(:,1).^2 - w(:,2).^2), (w_dot(:,1) - w(:,2).*w(:,3)), w(:,1).*w(:,2), (w_dot(:,2) + w(:,1).*w(:,3)), w_dot(:,3)];


tmp = A'*A;
[U,S,V] = svd(tmp);
I = [V(6,1:3);
    V(6,2), V(6,4), V(6,5);
    V(6,3), V(6,5), V(6,6)]
