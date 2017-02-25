function du = ilqr_iteration(u, x0, xf, Qf, Q, R, dt)
%% need to provide the u .. it will calaculate the du by which it need to change
% du = ilqr_iteration(u, x0, xf, Qf, Q, R, dt)

du = zeros(size(u));

nx = length(x0);
nu = length(u(1,:));

x = ilqr_openloop(x0, u, dt); 
N = length(x);
dx = 0.00000000000001*ones(size(x));

A = zeros(nx, nx, N-1);
B = zeros(nx, nu, N-1);

for i = 1:N-1
	[A(:,:,i) B(:,:,i)] = ilqr_linearization(x(i,:), u(i,:));
end;

S = zeros(nx, nx, N);
S(:,:,N) = Qf;

nv = nx;
v = zeros(N, nx);
v(N,:) = (Qf*(x(N,:) - xf)')';

K = zeros(nu, nx, N-1);
Kv = zeros(nu, nv, N-1);
Ku = zeros(nu, nu, N-1);

for k = N-1:-1:1
	
 	K(:,:,k) = inv(B(:,:,k)' * S(:,:,k+1) * B(:,:,k) + R) * B(:,:,k)' * S(:,:,k+1) * A(:,:,k);
	Kv(:,:,k) = inv(B(:,:,k)' * S(:,:,k+1) * B(:,:,k) + R) * B(:,:,k)';
	Ku(:,:,k) = inv(B(:,:,k)' * S(:,:,k+1) * B(:,:,k) + R) * R;

	S(:,:,k) = A(:,:,k)' * S(:,:,k+1) * (A(:,:,k) - B(:,:,k) * K(:,:,k)) + Q;
	v(k,:) = ((A(:,:,k) - B(:,:,k) * K(:,:,k))' * v(k+1,:)' - K(:,:,k)' * R * u(k,:)' + Q * x(k,:)')';

	du(k,:) = (-1) * (K(:,:,k)* dx(k,:)' + Kv(:,:,k) * v(k+1,:)' + Ku(:,:,k) * u(k,:)')';
end;
