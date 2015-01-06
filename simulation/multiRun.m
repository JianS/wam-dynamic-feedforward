clear;

IfmulRun = 1;

parameters;
% t_int = 0.001;

maxlen_resting = 1/t_int; % maximun resting time

omegalist = [-5; -1; 6; 10];
% omegalist = [-10; -5; 0; 5; 10];
% omegalist = [0;];
n_ol = size(omegalist,1);

Dx_log_vec = NaN.*zeros(n_ol,maxlen_resting);
Dz_log_vec = NaN.*zeros(n_ol,maxlen_resting);
Dp_log_vec = NaN.*zeros(n_ol,maxlen_resting);
angle_log_vec = NaN.*zeros(n_ol,maxlen_resting);
Dang_log_vec = NaN.*zeros(n_ol,maxlen_resting);
delta_cp_log = NaN.*zeros(n_ol, 1);

% ii = 0;
softcatch_simulation;
% for i_omega = -10:5:10
%     ii = ii + 1;
for ii = 1:n_ol
    clc
    clear x z q p_hand q_hand Dx_log Dz_log angle_log Dang_log logang f_limit;
    close all;
    
%     omega = i_omega;
    omega = omegalist(ii);
    softcatch_simulation;
    
    Dx_log_vec(ii, 1:size(Dx_log,2)) = Dx_log;
    
    Dz_log_vec(ii, 1:size(Dz_log,2)) = Dz_log;
    
    for jj = 1:size(Dx_log,2)
        Dp_log_vec(ii, jj) = norm([Dx_log(jj) Dz_log(jj)]);
    end
    
    angle_log_vec(ii, 1:size(angle_log,2)) = angle_log;
    
    Dang_log_vec(ii, 1:size(Dang_log,2)) = Dang_log;
    
    delta_cp_log(ii) = delta_cp_ang;
end
    

%% Plot
figure
% plot3(angle_log_vec(1,:), Dang_log_vec(1,:), Dx_log_vec(1,:));
plot3(angle_log_vec', Dang_log_vec', Dp_log_vec', 'linewidth',2);
xlabel('\Phi','FontSize',16); ylabel('\omega','FontSize',16); zlabel('||v||','FontSize',16);

% lengend_list = ['omega: ' num2str(omegalist(1))];
n_ol = size(angle_log_vec,1);
for ii = 1:n_ol
    lengend_list(ii) = {['omega: ' num2str(omegalist(ii)) ' rad/s; \Delta\theta: ' num2str(delta_cp_log(ii)) 'rad']};
end
legend(lengend_list)
%
[CloseX, CloseY, CloseZ] = ellipsoid(0,0,0,angle2vertical_near,Dang_near,Dp_near,12);
hold on
mesh(CloseX, CloseY, CloseZ, 'EdgeColor','k')%,'FaceColor','red','EdgeColor','none');

hidden off
grid on

set(gcf, 'Position', [700 200 1000 800]);

% save('mulRun_success.mat', 'angle_log_vec', 'Dang_log_vec', 'Dp_log_vec','angle2vertical_near','Dang_near','Dp_near','omegalist')
% load('mulRun_success.mat')
