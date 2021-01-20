clear
clc
close all
u1=5;u2=5;u3=5;
theta1=0;
theta2=30;
theta3=30;
x0 = [-100;  0;  0; 150; 200;   0;   0;-175;  0; -200;-300;  0;  200;  30];
%    [  x1; y1; x2;  y2;  x3;  y3;  x4;  y4;  x5;  y5;  x6;  y5;  x7;  y7]
xf = x0(1:8);

xl = x0(9:14);
% xd_l = [0; -200;-300;  0;  200;  30];  % disired goal position
L = [4 -1 -1  0 -1 -1  0
    -1  3 -1  0  0 -1  0
    -1 -1  5 -1 -1  0 -1
    0  0 -1  3 -1  0 -1];


A = kron(-L(1:4,1:4), eye(2));
B = kron(-L(1:4,5:7), eye(2));

vel_of_leader = [u1*cosd(theta1)
                 u1*sind(theta1)
                 u2*cosd(theta2)
                 u2*sind(theta2)
                 u3*cosd(theta3)
                 u3*sind(theta3)];


% xl_fin = [10;20;30;40;50;60];
i=1;
xl_new =[0;0;0;0;0;0];
while norm(xl_new - xl) ~= 0    
    
    
    
    
    
    XX_follower = xf(1:2:8);
    YY_follower = xf(2:2:8);
    if i==1
        XX_leaders = xl(1:2:6);
        YY_leaders = xl(2:2:6);
        xl_new = xl + vel_of_leader;
    else
        xl_new = xl_new + vel_of_leader;
        XX_leaders = xl_new(1:2:6);
        YY_leaders = xl_new(2:2:6);
        
    end
    for i = (1:length(XX_follower))
        plot(XX_follower,YY_follower,"*", XX_leaders, YY_leaders, 'o')
        axis([-400 400 -400 400])
        xlabel("X-axis")
        ylabel("Y-axis")
        title("3 leaders followed by four four followers")
        pause(0.001)
        hold on
    end
    
    hold off

    xf_dot = A*xf + B*xl_new;
    xf = xf + 0.01*xf_dot;
   
    pause(0.001)
    i=i+1;
end