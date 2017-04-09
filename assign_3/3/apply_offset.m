offset = [0,3,0];
pose = load('../2E/problem_2_3.dat');
T = size(pose,1);
offset = [0;2.2;0];
com_offset = zeros(T,3);
for i=1:T
    com = pose(i,1:3);
    q = pose(i,4:7);
	R = quat2rotm(q);
	com_offset(i,:) = com + (inv(R)*offset)';
end
quat = pose(:,4:7);
pose_offset = [com_offset, quat];
csvwrite('problem_3.dat',pose_offset)
