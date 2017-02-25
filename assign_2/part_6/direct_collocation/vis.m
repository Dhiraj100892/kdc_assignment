%% helpful for visualizing the solution
close all


%% cart_pos
figure(1)
set(gca,'nextplot','replacechildren');
mov(1:size(z,2)) = struct('cdata', [],...
                        'colormap', []);
for i = 1: size(z,2)
    
    lim = 6;
    cart_cm = [-2 0];
    cart_cm = [z(1,i) 0];
    cart_size = 3;
    cart_pos = [cart_cm(1)-0.5 * cart_size cart_cm(2)-0.5 * cart_size cart_size cart_size];
    rectangle('Position',cart_pos,'FaceColor',[0 .5 .5])

    axis([-lim lim -lim lim])
    axis off
    grid on

    hold on
    % pole pos
    l = 5;
    w = 0.5;

    ang = z(2,i);   %% from verticle

    x = [0.5*w 0.5*w -0.5*w -0.5*w ];

    y = [0 -l -l 0];
    rot = [cos(ang) -sin(ang);
           sin(ang) cos(ang)];

    X = zeros(1,4);
    Y = zeros(1,4);

    for i = 1 : 4
       temp = rot * [x(i); y(i)];
       X(i) = temp(1);
       Y(i) = temp(2);
    end
    X = X + cart_cm(1);
    Y = Y + cart_cm(2);
    fill(X,Y,'r')

    mov(i) = getframe(gcf);
    clf
end

movie2avi(mov, 'test.avi', 'compression', 'None');
% nFrames = 20;
% 
% Preallocate movie structure.
% mov(1:nFrames) = struct('cdata', [],...
%                         'colormap', []);
% 
% Create movie.
% Z = peaks; surf(Z); 
% axis tight manual
% set(gca,'nextplot','replacechildren');
% for k = 1:nFrames 
%    surf(sin(2*pi*k/20)*Z,Z)
%    mov(k) = getframe(gcf);
% end
% 
% % Create AVI file.
% movie2avi(mov, 'myPeaks.avi', 'compression', 'None');