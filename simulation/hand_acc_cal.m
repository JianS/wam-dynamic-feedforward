clc
clear
close all

%% Calculate the hand acceleration from the edge of workspace to catchpoint
n = 10;

v0_start = -0.1; v0_end = -2; v0_int = (v0_end - v0_start)/n;
d_start = -0.05; d_end = -0.3; d_int = (d_end - d_start)/n;
% vc_start = -2.5; vc_end = -5; vc_int = (vc_end - vc_start)/n;
% t_start = 0.4; t_end = 0.5; t_int = (t_end - t_start)/n;

v0 = v0_start:v0_int:v0_end; % velocity of object when hand start to move;
d = d_start:d_int:d_end; % distance from hand to object at start;

% vc = vc_start:vc_int:vc_end; % catch velocity
% t = t_start:t_int:t_end; % time from hand start moving till catching;
% d = -0.1; 
g = -10; % m/s^2

%%
% for k = 1:4
    for i = 1:n+1
        for j = 1:n+1
%             num = v0(i)^2 + 2*g*t(j)*v0(i) + g^2*(t(j)^2);
%             den = -2.*d + 2.*t(j).*v0(i) + g*(t(j)^2);
%             num = vc(j)^2;
%             den = -2*d + (vc(j)^2 - v0(i)^2)/g;
            vc(j,i) = v0(i) + 2*g*d(j)/v0(i);
            acc_hand(j,i) = vc(j,i)*g/(vc(j,i) - v0(i));
        end
    end
%     d = d - 0.05;
% end

%% Plot 
% 
ax = 48; bx = 16;

subplot(1,2,1);
mesh(v0, d, acc_hand);
xlabel('v_0', 'FontSize',18);
%     ylabel('\Deltat', 'FontSize',18);
ylabel('d', 'FontSize',18);
zlabel('acc_h_a_n_d', 'FontSize',18);
%     title(['d = ' num2str(-0.1*i) 'm'], 'FontSize',18);
view(ax,bx);
colorbar;
axis tight;

subplot(1,2,2);
mesh(v0, d, vc);
xlabel('v_0', 'FontSize',18);
%     ylabel('\Deltat', 'FontSize',18);
ylabel('d', 'FontSize',18);
zlabel('v_c', 'FontSize',18);
%     title(['d = ' num2str(-0.1*i) 'm'], 'FontSize',18);
view(ax,bx);
colorbar;
axis tight;
