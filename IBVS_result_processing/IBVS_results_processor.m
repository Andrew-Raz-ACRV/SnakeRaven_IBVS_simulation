%IBVS Data Processing Script 25 May 2021
%Updated in 2023 for Tmech
clear all
close all
clc

%Include the functions and data:
addpath('functions')
addpath('A_direction')
addpath('D_direction')

addpath('Z_VPC_A_success')
addpath('Z_VPC_D_success')

%Settings:
a_direction = 1; %false for d
ibvs_test = 1; %false for VPC


%% Get NDI Data of endeffector pose and Feature Tracker Data and SnakeRaven Camera Pose
%NDI endeffector pose
%NDI = NDIcsv2structA('NDIdata_snakeravenTEST_VISION_a_direction.csv',false);
%NDI = NDIcsv2structA('NDIdata_snakeravenTEST_VPC_A_success.csv',false);
%NDI.A.T

%Get Feature Tracker Data
%feature_error = featuretrackercsv2struct('A_direction/FeatureTrackerIBVSData.csv');
%feature_error = featuretrackercsv2struct('Z_VPC_A_success/FeatureTrackerIBVSData.csv');
%feature_error.Error

%Get SnakeRaven Camera Pose:
%SnakeCamPose = SnakeRavencsv2struct('A_direction/SnakeRavenCameraData.csv');
%SnakeCamPose = SnakeRavencsv2struct('Z_VPC_A_success/SnakeRavenCameraData.csv');
%SnakeCamPose.Tend
%SnakeCamPose.Ttarget
if(ibvs_test)
    if(a_direction)
        NDI = NDIcsv2structA('NDIdata_snakeravenTEST_VISION_a_direction.csv',false);
        feature_error = featuretrackercsv2struct('A_direction/FeatureTrackerIBVSData.csv');
        SnakeCamPose = SnakeRavencsv2struct('A_direction/SnakeRavenCameraData.csv');
    else
        NDI = NDIcsv2structA('NDIdata_snakeravenTEST_VISION_d_direction.csv',false);
        feature_error = featuretrackercsv2struct('D_direction/FeatureTrackerIBVSData.csv');
        SnakeCamPose = SnakeRavencsv2struct('D_direction/SnakeRavenCameraData.csv');
    end
    u = [1.87034381248524,-1.62645948956948,0.0672522169248183]; %IBVS A/D
else
    if(a_direction)
        NDI = NDIcsv2structA('NDIdata_snakeravenTEST_VPC_A_success.csv',false);
        feature_error = featuretrackercsv2struct('Z_VPC_A_success/FeatureTrackerIBVSData.csv');
        SnakeCamPose = SnakeRavencsv2struct('Z_VPC_A_success/SnakeRavenCameraData.csv');
        u = [-1.29043267827014,1.93415347464714,0.000859331748205085]; %VPC A
    else
        NDI = NDIcsv2structA('NDIdata_snakeravenTEST_VPC_D_success.csv',false);
        feature_error = featuretrackercsv2struct('Z_VPC_D_success/FeatureTrackerIBVSData.csv');
        SnakeCamPose = SnakeRavencsv2struct('Z_VPC_D_success/SnakeRavenCameraData.csv');
        u = [-1.29038093890478,1.93414453299211,0.000880834255545453]; %VPC D
    end
end

%% Plot Feature Tracker error over time:
f2 = figure(2); 
leg = false;
if(leg)
    plot(feature_error.Error(250:350,2:21))
else
    plot(feature_error.Error(:,2:21))
end
title('Feature error over time')
xlabel('Iterations')
ylabel('Feature error [mm]')
axis([0 inf -0.4 0.4]);
if(leg)
    lgd = legend('e_{1x}','e_{1y}','e_{2x}','e_{2y}','e_{3x}','e_{3y}','e_{4x}','e_{4y}',...
    'e_{5x}','e_{5y}','e_{6x}','e_{6y}','e_{7x}','e_{7y}','e_{8x}','e_{8y}',...
    'e_{9x}','e_{9y}','e_{10x}','e_{10y}',...
    'Location','eastoutside','Orientation','horizontal'); 
    lgd.NumColumns = 2;
end
f2.Position(3) = 300;
f2.Position(4) = 300;

%% Solve Rotation from NDI sensor to Camera pose
%World to NDI rotation matrix by experiment setup inspection:
W_R_NDI = Rx(pi/2)*Rz(-pi/2);
%Calibration of sensor to camera:
%u = [2.23046404443725,1.25045568424440,-3.12400818708826]; %solved with fminsearch
%u = [deg2rad(-85) deg2rad(120) deg2rad(0)];
%u = [1.87034381248524,-1.62645948956948,0.0672522169248183]; %IBVS A/D
%u = [-0.3131   -0.7607    0.8119];
%u = [-1.29043267827014,1.93415347464714,0.000859331748205085]; %VPC A
%u = [-1.29038093890478,1.93414453299211,0.000880834255545453]; %VPC D
s_R_c = Rx(u(1))*Ry(u(2))*Rz(u(3));

%s_R_c = s_R_c * Rz(deg2rad(90))*Rx(deg2rad(90));


%% Get orientation Service Sphere Trajectory:

%Basically Compare Sim: w_R_c and reference w_R_c* 
%to NDI: w_R_ndi * ndi_R_s * s_R_c
R_ndi = zeros(3,3,NDI.N);
for ii = 1:NDI.N
    R_ndi(:,:,ii) = W_R_NDI * NDI.A.T(1:3,1:3,ii) * s_R_c;
end
R_cam = SnakeCamPose.Tend(1:3,1:3,:);
R_tar = SnakeCamPose.Ttarget(1:3,1:3,:);

f1 = figure;
Plot_orientation_trajectory_service_sphere(R_ndi,R_cam,R_tar)
%title('Orientations Explored about Target from the Camera')
title('Service sphere about target')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
%view([225 25])
%view([200 25])
view([0 90]) %Top down view
legend('EM','FK','Location','northeast')
axis equal
grid on
f1.Position(3) = 300;
f1.Position(4) = 300;

% %% Compute Rotation error over time:
% 
% %Decide how to truncate the data to be the same size:
% Nndi = length(R_ndi);
% Ncam = length(R_cam);
% if Ncam>Nndi
%     R_cam = R_cam(:,:,1:Ncam/Nndi:end); N = Nndi;
% else
%     R_ndi = R_ndi(:,:,1:Nndi/Ncam:end); N = Ncam;
% end
% 
% %Compute error between the rotation matrices:
% % error = zeros(3,1);
% % for ii=1:N
% %     T0 = [R_cam(:,:,ii) [0 0 0]'; 0 0 0 1];
% %     T1 = [R_ndi(:,:,ii) [0 0 0]'; 0 0 0 1];
% %     err = trans2dx(T0,T1);
% %     error = error + err(4:6,1);
% % end
% % out = norm(error/N);
% % disp(out)
% 
% figure(1)
% Plot_orientation_trajectory_service_sphere(R_ndi,R_cam,R_tar)
%% Plot Camera Trajectory - Too noisy and unclear to show
% 
% figure,
% plot3(SnakeCamPose.x_end, SnakeCamPose.y_end, SnakeCamPose.z_end,'r.')
% hold on
% plot3(SnakeCamPose.x_tar, SnakeCamPose.y_tar, SnakeCamPose.z_tar,'g.')
% 
% %Figure style
% title('SnakeRaven Camera Trajectory Results'), xlabel('X (mm)')
% ylabel('Y (mm)'), zlabel('Z (mm)')
% view(3)
% view([225 35]); %view([-75 20]);
% axis equal
% grid on
% legend('Camera Pose','Desired Camera Pose','Location','NorthEast')
% 
% %Denoise data:
% b = 100;
% figure,
% plot3(downsample(medfilt1(SnakeCamPose.x_end),b),downsample(medfilt1(SnakeCamPose.y_end),b),...
%     downsample(medfilt1(SnakeCamPose.z_end),b),'r.'), hold on
% plot3(downsample(medfilt1(SnakeCamPose.x_tar),b),downsample(medfilt1(SnakeCamPose.y_tar),b),...
%     downsample(medfilt1(SnakeCamPose.z_tar),b),'g.')
% 
% %Figure style
% title('SnakeRaven Camera Trajectory Results'), xlabel('X (mm)')
% ylabel('Y (mm)'), zlabel('Z (mm)')
% view(3)
% view([225 35]); %view([-75 20]);
% axis equal
% grid on
% legend('Camera Pose','Desired Camera Pose','Location','NorthEast')
% 
% 
% 
