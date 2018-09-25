clear; clc; close all;
%% Solver Parameters
t_i = 0;    %start time, s
t_f = 20;   %end time, s
t_span = [t_i t_f]; %start and end times, s
%To add transitions, include the times in this vector:
t_switch = [t_i, t_i+6, t_i+13 t_f];

cable_len = 2;  %length of each cable, m
L = 3;  %length of payload (assumed to be cylindrical rod), m
R1 = 0.099; %inner radius of payload (assumed to be cylindrical tube), m
R2 = 0.1;   %outer radius of payload (assumed to be cylindrical tube), m
radial_os = sqrt(L^2+R2^2)/2;   %distance from cable connection to tube CoM
m = 1;  %mass of payload, kg
g = 9.81;  %gravitational constant, m.s^-2
T = m*g/2;    %cable tension (assumed constant when in flight), N 
kcable = 1.8e11*pi*(1e-3)^2/cable_len;  %cable spring constant = E*A/l, N.m^-1
Iz = m*((R1^1+R2^2)/4 + L^2/12);    %payload mass moment of inertia, kg.m^2

ki = [0,0,0,0]; %integral constant for x1,y1,x2,x2
kp = [5,5,5,5]; %proportional constant for x1,y1,x2,y2
kd = [4,4,60,4]; %derivative constant for x1,y1,x2,y2
des = [3 9 10 10;10 10 0 0;10 10 0 0]; %setpoints for each transition, for x1,y1,y2
v_max = [2 2 2 2];  %max velocity for x1,y1,x2,y2
init = [0,3,0,...   %integral(x1),x1,x1'
    0,0,0,...      %integral(x2),x2,x2'
    0,1.5,0,...       %integral(xc),xc,xc'
    0,0,0,...       %integral(y1),y1,y1'
    0,0,0,...       %integral(y2),y2,y2'
    0,0,0,...       %~,~,yc
    0,0,0,...       %integral(tet1),tet1,tet1',
    0,0,0,...      %integral(tet2),tet2,tet2'
    0,0,0,...     %integral(psi),psi,psi'
   ];        

%% ODE Solver
[t_a,a] = ode15s(@(tx,x) cpt_startupmode(tx,x,ki,kp,kd,t_switch,des,v_max,T,cable_len,L,R2,radial_os,m,Iz), [0 5], init);
[t_b,b] = ode15s(@(tx,x) cpt_carrymode(tx,x,ki,kp,kd,t_switch,des,v_max,T,cable_len,L,radial_os,m,Iz), [5 12], a(size(a,1),:));
[t_c,c] = ode15s(@(tx,x) cpt_dropoffmode(tx,x,ki,kp,kd,t_switch,des,v_max,T,cable_len,L,radial_os,m,Iz), [12 t_f], b(size(b,1),:));

%% Plot

figure('Name','Evolution of System State')
subplot(3,1,1)
plot(t_a,a(:,2),'b-') %x1
hold on
plot(t_a,a(:,5),'r-') %x2
hold on
plot(t_a,(a(:,2)+a(:,5))/2,'k--') %xav
hold on
plot(t_a,a(:,8),'g-') %xc
hold on
plot(t_b,b(:,2),'b-')
hold on
plot(t_c,c(:,2),'b-')
hold on
plot(t_b,b(:,5),'r-')
hold on
plot(t_c,c(:,5),'r-')
hold on
plot(t_b,(b(:,2)+b(:,5))/2,'k--')
hold on
plot(t_c,(c(:,2)+c(:,5))/2,'k--')
hold on
plot(t_b,b(:,8),'g-')
hold on
plot(t_c,c(:,8),'g-')
title('Motion in x-axis')
ylabel('x / m')
xlabel('t / s')
legend('x_1','x_2','x_a_v','x_c')
axis([-inf inf -1 20])
subplot(3,1,2)
plot(t_a,a(:,11),'b-') %y1
hold on
plot(t_a,a(:,14),'r-') %y2
hold on
plot(t_a,a(:,18),'g-') %yc
hold on
plot(t_b,b(:,11),'b-')
hold on
plot(t_c,c(:,11),'b-')
hold on
plot(t_b,b(:,14),'r-')
hold on
plot(t_c,c(:,14),'r-')
hold on
plot(t_b,b(:,18),'g-')
hold on
plot(t_c,c(:,18),'g-')
title('Motion in y-axis')
ylabel('y / m')
xlabel('t / s')
legend('y_1','y_2','y_c')
axis([-inf inf 0 11])
subplot(3,1,3)
plot(t_a,a(:,20),'b-') %theta1
hold on
plot(t_a,a(:,23),'r-') %theta2
hold on
% plot(t_a,a(:,26),'g-') %phi
% hold on
plot(t_a,pi/4*ones(size(t_a)),'k-.') %upper bound
hold on
plot(t_a,-pi/4*ones(size(t_a)),'k-.') %lower bound
hold on
plot(t_b,b(:,20),'b-')
hold on
plot(t_c,c(:,20),'b-')
hold on
plot(t_b,b(:,23),'r-')
hold on
plot(t_c,c(:,23),'r-')
hold on
plot(t_b,pi/4*ones(size(t_b)),'k-.') %upper bound
hold on
plot(t_c,pi/4*ones(size(t_c)),'k-.') %upper bound
hold on
plot(t_b,-pi/4*ones(size(t_b)),'k-.') %lower bound
hold on
plot(t_c,-pi/4*ones(size(t_c)),'k-.') %lower bound
title('Cable Angles')
ylabel('Angle from the normal / rad')
xlabel('t / s')
legend('\theta_1','\theta_2','\pi/4','-\pi/4')
axis([-inf inf -pi pi])