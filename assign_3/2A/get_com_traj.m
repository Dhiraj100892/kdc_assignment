com = [-11,0.0,0.0];
vel = [0.02,-0.05,0.01];
freq = 100;
T = 20*freq;
pos = zeros(T,length(com));
pos(1,:) = com;
for i=2:T
    pos(i,:) = pos(i-1,:) + vel*dt;
end