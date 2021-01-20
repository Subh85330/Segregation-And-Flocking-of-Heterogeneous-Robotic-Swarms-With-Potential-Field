clear
clc
close all
u1=1;
% u2=5;u3=5;
theta1=0;
% theta2=0;
% theta3=0;
x0 = [0;  -300;  0; 300; 300;   0];
%    [  x1; y1; x2;  y2;  x3;  y3]
xf = x0;

% xl = x0(5:6);
% xd_l = [0; -200;-300;  0;  200;  30];  % disired goal position
L = [2 -1 -1
     -1 2 -1
     -1 -1 2];

A = kron(-L, eye(2));
% A = kron(-L(1:2,1:2), eye(2));
% B = kron(-L(1:2,3), eye(2));
% 
% vel_of_leader = [u1*cosd(theta1)
%                  u1*sind(theta1)];


% xl_fin = [10;20;30;40;50;60];
i=1;
xl_new =[0;0];
while true  
    
    
    
    
    
    XX_follower = xf(1:2:6);
    YY_follower = xf(2:2:6);
%     if i==1
%         XX_leaders = xl(1);
%         YY_leaders = xl(2);
%         xl_new = xl + vel_of_leader;
%     else
%         xl_new = xl_new + vel_of_leader;
%         XX_leaders = xl_new(1)
%         YY_leaders = xl_new(2)
%         
%     end
    for i = (1:length(XX_follower))
        plot(XX_follower,YY_follower,"*")
        axis([-800 800 -800 800])
        xlabel("X-axis")
        ylabel("Y-axis")
        title("3 leaders followed by four four followers")
        pause(0.001)
        hold on
    end
    
    hold off

    xf_dot = A*xf;
    xf = xf + 0.01*xf_dot;
   
    pause(0.001)
    i=i+1;
end