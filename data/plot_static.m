clc
clear
close all

%%

cam = load('cam_static');
% cam1 = load('cam2'); %This the data using wam kinematics but same gain as [80 80 80](original [2000 2000 2000])
% wam = load('wam2');
wam = load('wam_static');
wamRef = load('wamRef_static');
parabola = load('data_static');

cut_begin = 1;
cut_end = 0;

n = min([size(cam,1), size(wam,1), size(wamRef,1), size(parabola,1)]);
cam = cam(cut_begin:n-cut_end,:);
wam = wam(cut_begin:n-cut_end,:);
wamRef = wamRef(cut_begin:n-cut_end,:);
parabola = parabola(cut_begin:n-cut_end,:);

t = parabola(:,1);
Vx = parabola(:,2);
Vy = parabola(:,3);
Vz = parabola(:,4);

%%

d = cam(:, 2:4) - wam(:, 2:4);

for i = 1:n
    d_norm(i) = norm(d(i,:));
end

mean(d_norm)
