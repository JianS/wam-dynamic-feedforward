function cam_quaternion = plot3Dpose(data1,data2,data3, delay, MUL, title_)
   
cam = data1;
wam = data2;
wamRef = data3;
    
%% Orientation Calculation
n = size(cam,1);
R_cam = zeros(3,3,n);

% tm = load('transferMatrix');
% R_cam2wam = tm(1:3,1:3);

R = zeros(3,3,n);

x_unitVec_cam = zeros(3,1,n);
y_unitVec_cam = zeros(3,1,n);
z_unitVec_cam = zeros(3,1,n);

x_unitVec_wam = zeros(3,1,n);
y_unitVec_wam = zeros(3,1,n);
z_unitVec_wam = zeros(3,1,n);

x_unitVec_wamRef = zeros(3,1,n);
y_unitVec_wamRef = zeros(3,1,n);
z_unitVec_wamRef = zeros(3,1,n);

% unitLength = mean(0.05*(max(cam(:,2:4))-min(cam(:,2:4))));
unitLength = 0.03;
for i = 1:n
    cam_q(i) = quaternion(cam(i,5:8));
    R_cam(:,:,i) = cam_q(i).RotationMatrix;
%     R(:,:,i) = R_cam2wam*R_cam(:,:,i);
    R(:,:,i) = R_cam(:,:,i);
    x_unitVec_cam(:,:,i) = R(:,:,i)*[unitLength;0;0];
    y_unitVec_cam(:,:,i) = R(:,:,i)*[0;unitLength;0];
    z_unitVec_cam(:,:,i) = R(:,:,i)*[0;0;unitLength];
    
    R_tem = quaternion(wam(i,5:8)).RotationMatrix;
    x_unitVec_wam(:,:,i) = R_tem*[unitLength;0;0];
    y_unitVec_wam(:,:,i) = R_tem*[0;unitLength;0];
    z_unitVec_wam(:,:,i) = R_tem*[0;0;unitLength];
    
    R_tem = quaternion(wamRef(i,5:8)).RotationMatrix;
    x_unitVec_wamRef(:,:,i) = R_tem*[unitLength;0;0];
    y_unitVec_wamRef(:,:,i) = R_tem*[0;unitLength;0];
    z_unitVec_wamRef(:,:,i) = R_tem*[0;0;unitLength];
end

cam_quaternion = cam_q;
%% Plot

% hold on

% plot3(cam(:,2), cam(:,3), cam(:,4), 'k', 'linewidth',1);
% xlabel('x_b_a_s_e');ylabel('y_b_a_s_e');zlabel('z_b_a_s_e');
% plot3(cam1(:,2), cam1(:,3), cam1(:,4), 'g');
% plot3(wam(:,2), wam(:,3), wam(:,4));
% plot3(ref(:,1), ref(:,2), ref(:,3), 'r');
% legend('End-point traj from VISION','End-point traj from WAM','ref traj');
% legend('Object traj from VISION');
% title(title_);
% az = 10; el = 10;
% view(az,el);

%% Slow motion
% n = size(cam); n = n(1);
figure;
set(gcf, 'Position', get(0,'ScreenSize'));
margin = 0.1;
% delay = 0.1;

r_ws = 0.82; % radius of WAM workspace
[ex,ey,ez] = ellipsoid(0,0,0,r_ws,r_ws,r_ws,12);
view(-180,0);
axis([min(cam(:,2))-margin max(cam(:,2))+margin min(cam(:,3))-margin max(cam(:,3))+margin min(cam(:,4))-margin max(cam(:,4))+margin]);

for i = 1:MUL:n
%     
%     if i==2
%          legend('Object center traj','origin','x','y','z')
%     end
   
    
    
%     plot3(cam(i,2),cam(i,3),cam(i,4),'o','Color','k');

    plot3(cam(:,2), cam(:,3), cam(:,4), 'g');
    
    hold on
    grid on
    axis equal
    view(-180,0);
    axis([min(cam(:,2))-margin max(cam(:,2))+margin min(cam(:,3))-margin max(cam(:,3))+margin min(cam(:,4))-margin max(cam(:,4))+margin]);

    title(['Time: ' num2str(wamRef(i,1)) 'sec']);
     
    plot3(wam(:,2), wam(:,3), wam(:,4));
    plot3(wamRef(:,2), wamRef(:,3), wamRef(:,4), 'r');
    legend('Object center traj', 'WAM end-point traj', 'WAM end-point ref signal');

    l = 0.35;
    line([0 l],[0 0],[0 0],'Color','r','linewidth',5);
    line([0 0],[0 l],[0 0],'Color','g','linewidth',5);
    line([0 0],[0 0],[0 l],'Color','b','linewidth',5);
   
%     mesh(ex, ey, ez, 'EdgeColor',[.9 .9 .9], 'FaceColor','none');

    plot3(cam(i,2),cam(i,3),cam(i,4),'o','Color','k','linewidth', 1);    
    plot3(wam(i,2),wam(i,3),wam(i,4),'o','Color','b','linewidth', 1);      
    plot3(wamRef(i,2),wamRef(i,3),wamRef(i,4),'*','Color','r','linewidth', 1);
    
%     DrawFrame(cam, x_unitVec_cam, y_unitVec_cam, z_unitVec_cam, i, 2, ':');
    DrawFrame(wam, x_unitVec_wam, y_unitVec_wam, z_unitVec_wam, i, 2,'-');
    DrawFrame(wamRef, x_unitVec_wamRef, y_unitVec_wamRef, z_unitVec_wamRef, i, 1, '--');

    if delay == inf
        pause();
    else
        pause(delay);
    end
    
    hold off
    drawnow
end
% legend('Object center traj','origin','x','y','z')



end