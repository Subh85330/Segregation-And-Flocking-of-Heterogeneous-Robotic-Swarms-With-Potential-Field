clear
clc
close all
u1=13;   %Velocity of leader x3
theta1=0; %initial angle of velocity with horizontal axis

x0 = [130;  0;  0; 130; 200;   200];
%    [  x1; y1; x2;  y2;  x3;  y3]
xf = x0(1:4);

xl = x0(5:6);
% xd_l = [0; -200;-300;  0;  200;  30];  % disired goal position
L = [2 -1 -1
    -1  2 -1 ];


A = kron(-L(1:2,1:2), eye(2));
B = kron(-L(1:2,3), eye(2));




% xl_fin = [10;20;30;40;50;60];
i=1;
xl_new =[0;0];
h = figure(1);
axis tight manual
while norm(xl_new - xl) ~= 0
    
    vel_of_leader = [u1*cosd(theta1)
        u1*sind(theta1)];
    
    
    
    XX_follower = xf(1:2:4);
    YY_follower = xf(2:2:4);
    if i==1
        %First time leaders new state updated according to xl initial
        %leader position
        XX_leaders = xl(1);
        XX_traj(i) = XX_leaders;
        YY_leaders = xl(2);
        YY_traj(i) = YY_leaders;
        xl_new = xl + vel_of_leader;
    else
        % After first iteration leader position will be updated according
        % to new position of leader
        xl_new = xl_new + vel_of_leader;
        XX_leaders = xl_new(1);
        XX_traj(i) = XX_leaders;
        YY_leaders = xl_new(2);
        YY_traj(i) = YY_leaders;
        
    end
    
    %     for j = (1:length(XX_follower))
    plot(XX_follower,YY_follower,"*",...
        XX_leaders, YY_leaders, 'h',...
        XX_traj(i),YY_traj(i))
    axis([-1500 2000 -100 3500])
    xlabel("X-axis")
    ylabel("Y-axis")
    title("one leader followed by two followers")
    pause(0.01)
    
    hold on
    plot(XX_traj, YY_traj)
    hold off
    
    
    % Gif file creation
    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    % Write to the GIF File
    if i == 1
        imwrite(imind,cm,"test.gif",'gif', 'Loopcount',inf,'DelayTime',2);
        
    else
        imwrite(imind,cm,"test.gif",'gif','WriteMode','append','DelayTime',2);
    end
       
    
    
    % States updating
    xf_dot = A*xf + B*xl_new;
    alpha = 0.1;   % important parameter need to study
    xf = xf + alpha*xf_dot;
    theta1 = theta1 + 0.5;
  
    i=i+1;
end