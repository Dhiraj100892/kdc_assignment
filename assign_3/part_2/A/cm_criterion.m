function [ score ] = cm_criterion( CM )
% Implimenting the loss function for center of mass under no external force

global M
global NF
NM = 8; % number of markers
dt = 0.01;

% pull out parameters
com = [ CM(1) CM(2) CM(3) ];
vel = [ CM(4) CM(5) CM(6) ];

score = 0;

% calculate distances from com to each marker.
d2(NM) = 0;
for j = 1:NM
 v = reshape(M(1,j,:),[1,3]) - com;
 d2(j) = v*v'; % this is actually an inner product (since v is horizontal)
end

for i = 2:NF
 for j = 1:NM
  % last +1 to skip initial count variable
%   v = reshape(M(i,j,:),[1,3]) - com - vel * dt * 100 * (i - 1); % 100 time steps
  v = reshape(M(i,j,:),[1,3]) - com - vel * dt * 1 * (i - 1); % 1 time steps
  dist = v*v';
  score = score + (d2(j) - dist)*(d2(j) - dist);
 end
end

end

