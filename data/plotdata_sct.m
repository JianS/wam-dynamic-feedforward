clc
clear

close all

IfPlot_XYZvelocity = 1;
IfPlot_ObjAcc = 0;
IfPlot_SlowMotion = 1;
IfPlot_ObjAngularVelocity = 0;
IfPlot_ObjRotationAxis = 0;
IfPlot_ObjAngularMomentum = 0;


%%

cam = load('cam_sct');
% cam1 = load('cam2'); %This the data using wam kinematics but same gain as [80 80 80](original [2000 2000 2000])
% wam = load('wam2');
wam = load('wam_sct');
wamRef = load('wamRef_sct');
wamToolVel = load('wamToolVel_sct');
parabola = load('objvel_sct');

cut_begin = 50.5*500 + 1;
cut_end = 14.9*500;

n = min([size(cam,1), size(wam,1), size(wamRef,1), size(parabola,1)]);
cam = cam(cut_begin:n-cut_end,:);
wam = wam(cut_begin:n-cut_end,:);
wamRef = wamRef(cut_begin:n-cut_end,:);
wamToolVel = wamToolVel(cut_begin:n-cut_end,:);
parabola = parabola(cut_begin:n-cut_end,:);
n = size(cam, 1);

t = parabola(:,1);
Vx = parabola(:,2);
Vy = parabola(:,3);
Vz = parabola(:,4);

Vx_wam = wamToolVel(:,2);
Vy_wam = wamToolVel(:,3);
Vz_wam = wamToolVel(:,4);

% load '1storderEstimator_withoutdetectStartpoint';
% load 'MTC_sphere_markers';
% load 'MTC_6inCube_success.mat';
%% Plot object Vx Vy Vz
if (IfPlot_XYZvelocity)
    
figure
title('Object center linear velocity infomation (filtered)');

subplot(4,1,1);
plot(t, cam(:,4), t, wam(:,4), 'r');
legend('Object z position', 'WAM end-point z position');
grid on

subplot(4,1,2);
plot(t, Vx, t, Vx_wam, 'r');
legend('object x velocity', 'WAM end-point x vel');
grid on

subplot(4,1,3);
plot(t, Vy, t, Vy_wam, 'r');
legend('y velocity', 'WAM end-point y vel');
grid on

subplot(4,1,4);
plot(t, Vz, t, Vz_wam, 'r');
legend('z velocity', 'WAM end-point z vel');
grid on

end
%%
figure
plot(t, cam(:,4), t, wam(:,4), 'r', t, wamRef(:,4));
figure
plot(t(1:n-1),diff(wamRef(:,4))*500)
%% Plot Vz Az and Jz

if (IfPlot_ObjAcc)
figure
title('Object z-direction: velocity, acceleration, jerk (filtered)');

subplot(3,1,1);
plot(t, Vz);
legend('z velocity');

subplot(3,1,2);

plot(t(1:n-1), diff(Vz)*500);
legend('z accleration');

subplot(3,1,3);
plot(t(1:n-2), diff(diff(Vz)*500)*500);
legend('z jerk');
end
%% Compare Vz between vision data diff and filtered
% figure
% 
% n_cam = size(cam,1);
% subplot(2,1,1);
% grid on
% plot(cam(1:n_cam-1,1),diff(cam(1:n_cam,4))*500);
% legend('z velocity calculated from the vision position diff');
% 
% subplot(2,1,2);
% grid on
% plot(t, Vz);
% legend('z velocity');

%% Plot center trajactory

if(IfPlot_SlowMotion)
% figure
% 
% hold on
% grid on
% axis equal
% plot3(cam(:,2), cam(:,3), cam(:,4), 'g');
% % plot3(cam1(:,2), cam1(:,3), cam1(:,4), 'g');
% plot3(wam(:,2), wam(:,3), wam(:,4));
% plot3(wamRef(:,2), wamRef(:,3), wamRef(:,4), 'r');
% legend('Object center traj', 'WAM end-point traj', 'WAM end-point ref signal');
% 
% l = 0.35;
% line([0 l],[0 0],[0 0],'Color','r','linewidth',5);
% line([0 0],[0 l],[0 0],'Color','g','linewidth',5);
% line([0 0],[0 0],[0 l],'Color','b','linewidth',5);
% 
% r_ws = 0.82; % radius of WAM workspace
% [ex,ey,ez] = ellipsoid(0,0,0,r_ws,r_ws,r_ws,12);
% mesh(ex, ey, ez, 'EdgeColor',[.9 .9 .9], 'FaceColor','none');

MUL = 5;
% delay = 0.02;
delay = inf;

% view(-180,0);
% set(gcf, 'Position', get(0,'ScreenSize'));

% for i = 1:MUL:n
%     if i==2
% %         legend('Object center traj','origin','x','y','z')
%     end    
%     title(['Time: ' num2str(wamRef(i,1)) 'sec']);
%     
%     plot3(cam(i,2),cam(i,3),cam(i,4),'o','Color','k','linewidth', 1);    
%     plot3(wam(i,2),wam(i,3),wam(i,4),'o','Color','b','linewidth', 1);      
%     plot3(wamRef(i,2),wamRef(i,3),wamRef(i,4),'*','Color','r','linewidth', 1);
%   
% %     pause(delay); 
% %     pause();
% end

cam_q = plot3Dpose(cam, wam, wamRef, inf, MUL, 'Filtered data');

end
%% Plot object Angular Velocity
if(IfPlot_ObjAngularVelocity)
omega_axis = zeros(n, 3);
omega_angel = zeros(n,1);

% In this task, unfiltered vision data is not logged
% omega_axis_unf = zeros(n, 3);
% omega_angel_unf = zeros(n,1);

for i = 1:n
    omega_q(i) = quaternion(parabola(i,5:8));
    [omega_angel(i), omega_axis(i, :)] = omega_q(i).AngleAxis;
    
%     cam_q(i) = quaternion(cam(i, 5:8));
    
%     omega_q_unf(i) = quaternion(wamRef(i,5:8));
%     [omega_angel_unf(i), omega_axis_unf(i, :)] = omega_q_unf(i).AngleAxis;

end

figure
% np=4;
% subplot(np,1,1);
hold on
plot(t, omega_angel*500);
% plot(t, abs(omega_angel), 'r');
grid on
axis tight
title('Angular velocity (filtered)');
legend('Angular velocity (rad/sec)', 'absolute value');
mean(omega_angel)

end

%% Plot rotation axis: Filtered
if (IfPlot_ObjRotationAxis)
figure
hold on;
l = 0.5;
line([0 l],[0 0],[0 0],'Color','r','linewidth',5);
line([0 0],[0 l],[0 0],'Color','g','linewidth',5);
line([0 0],[0 0],[0 l],'Color','b','linewidth',5);
axis equal;
grid on;
axis([-1 1 -1 1 -1 1]);

view(240, 26);

MUL = 5;
delay = 0.02;

for i = 1:MUL:n
    line([0 omega_axis(i,1)],[0 omega_axis(i,2)],[0 omega_axis(i,3)], 'Color',[.9 .9 .9]);
    plot3(omega_axis(i,1),omega_axis(i,2),omega_axis(i,3),'*r');
%     text(1,1,1,['time: ' num2str(wamRef(i,1))]);
    title(['Rotation axis(filtered). Time: ' num2str(wamRef(i,1)) 'sec']);
    
    pause(delay);
end

end
%% Plot rotation axis: UNFiltered
% figure
% hold on;
% l = 0.5;
% line([0 l],[0 0],[0 0],'Color','r','linewidth',5);
% line([0 0],[0 l],[0 0],'Color','g','linewidth',5);
% line([0 0],[0 0],[0 l],'Color','b','linewidth',5);
% axis equal;
% grid on;
% axis([-1 1 -1 1 -1 1]);
% 
% view(240, 26);
% 
% for i = 1:MUL:n
%     plot3(omega_axis_unf(i,1),omega_axis_unf(i,2),omega_axis_unf(i,3),'*r');
%     line([0 omega_axis_unf(i,1)],[0 omega_axis_unf(i,2)],[0 omega_axis_unf(i,3)], 'Color',[.9 .9 .9]);
% %     text(1,1,1,['time: ' num2str(wamRef(i,1))]);
%     title(['Rotation axis(UNfiltered). Time: ' num2str(wamRef(i,1)) 'sec']);
%     
% %     pause(delay);
% end

%%
% figure
% [cam_omega cam_rotAxis]= cam_q.OmegaAxis;
% 
% [cam_angle cam_axis] = cam_q.AngleAxis;
% 
% plot(t, cam_omega);
% mean(cam_omega)

%% Plot the angular momentum
if(IfPlot_ObjAngularMomentum)
m = 0.138; % kg
L = 0.1524; % 6in in meter
I = m*L^3/6;

figure

sum_omega_axis = zeros(n,1);
for i = 1:n
    sum_omega_axis(i) = sum(abs(omega_axis(i,:)));
end

plot(t, omega_angel*500.*sum_omega_axis);
grid on
title('Angular momentum (filtered)');
legend('Angular momentum (kg·m^2/s)');

end