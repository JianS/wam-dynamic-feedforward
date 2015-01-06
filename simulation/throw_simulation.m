close all
clear
clc

%% Define parameters
% Initial state of the hand
x_0 = 0.; %-0.1;
z_0 = 0.62; %0.6;
theta_0 = -0.1;

Dx_0 = 0;
Dz_0 = 0;
Dtheta_0 = 0;

DDx_0 = 0;
DDz_0 = 0;
DDtheta_0 = 0;

% Final goal state of the hand
x_f = -0.55;%-0.55;
z_f = 0.75;%0.75;
theta_f = -0.8;

Dx_f = -4; %-4
Dz_f = 2; %2
Dtheta_f = -6;%-6;

DDx_f = -15;%-15
DDz_f = 25; %25
DDtheta_f = -25;%-25;

% time
t0 = 0;
tf = 0.4;
t_int = 0.002;
T = tf - t0;
n = T/t_int;
%% Calculate trajectory 
% Equations see <IROS2014 M. buss, et al. "Hierarchical Robustness Approach for Nonprehensile Catching of Rigid Objects">
% Chapter IV.E (4)(5)



Tmatrix = [0 0 0 0 0 1;
           0 0 0 0 1 0;
           0 0 0 2 0 0;
           T^5 T^4 T^3 T^2 T 1;
           5*T^4 4*T^3 3*T^2 2*T 1 0;
           20*T^3 12*T^2 6*T 2 0 0];
       
X_condition = [x_0; Dx_0; DDx_0; x_f; Dx_f; DDx_f];
Z_condition = [z_0; Dz_0; DDz_0; z_f; Dz_f; DDz_f];
Theta_condition = [theta_0; Dtheta_0; DDtheta_0; theta_f; Dtheta_f; DDtheta_f];

A = (Tmatrix)\X_condition;
B = (Tmatrix)\Z_condition;
C = (Tmatrix)\Theta_condition;

t = t0;
for i = 1:n+1
    x(i) = A(1)*t^5 + A(2)*t^4 + A(3)*t^3 + A(4)*t^2 + A(5)*t + A(6);
    z(i) = B(1)*t^5 + B(2)*t^4 + B(3)*t^3 + B(4)*t^2 + B(5)*t + B(6);
    theta(i) = C(1)*t^5 + C(2)*t^4 + C(3)*t^3 + C(4)*t^2 + C(5)*t + C(6);
    
    p_hand(1,1,i) = x(i);
    p_hand(1,2,i) = z(i);
    q_hand(i) = quaternion.angleaxis(theta(i), [0 1 0]);
    t = t + t_int;
end

disp('Init position: ');disp(x_0);disp(z_0);disp(theta_0);
disp('tf = ');disp(tf);
% disp('A=');disp(A');
disp(['A = ' num2str(A(1)) ', ' num2str(A(2)) ', ' num2str(A(3)) ', ' num2str(A(4)) ', ' num2str(A(5)) ', ' num2str(A(6))]);
disp(['B = ' num2str(B(1)) ', ' num2str(B(2)) ', ' num2str(B(3)) ', ' num2str(B(4)) ', ' num2str(B(5)) ', ' num2str(B(6))]);
disp(['C = ' num2str(C(1)) ', ' num2str(C(2)) ', ' num2str(C(3)) ', ' num2str(C(4)) ', ' num2str(C(5)) ', ' num2str(C(6))]);
% disp('B=');disp(B');
% disp('C=');disp(C');
%% Plot
% figure
% hold on
% plot(t0:t_int:tf, x)
% plot(t0:t_int:tf, z)
% plot(t0:t_int:tf, theta)

% hand geometry
l_hand = 12*0.0254;
h_hand = 0.0243;

l_robot = 0.45;

xc = 0; yc = 0; zc = 0; % robot origin

n_hand = size(p_hand,3);

r = 0.95; % raudius of the workspace

PlotResult

%%
figure;
subplot(2,1,1);
plot(t0:t_int:tf-t_int, diff(x)/t_int);
title('x Velocity');
grid on;
subplot(2,1,2);
plot(t0:t_int:tf-t_int, diff(z)/t_int);
title('z Velocity');
grid on;

figure;
subplot(2,1,1);
plot(t0:t_int:tf-2*t_int, diff(diff(x))/t_int^2);
title('x Accleration');
grid on;
subplot(2,1,2);
plot(t0:t_int:tf-2*t_int, diff(diff(z))/t_int^2);
title('z Accleration');
grid on;

figure;
subplot(2,1,1);
plot(t0:t_int:tf-t_int, diff(theta)/t_int);
title('angular velocity');
grid on;
subplot(2,1,2);
plot(t0:t_int:tf-2*t_int, diff(diff(theta))/t_int^2);
title('angular Accleration');
grid on;

