function [out] = Offset_cameraNDI_calibration(u)
% Computes error between camera poses based on rotation input
%

%World to NDI rotation matrix by experiment setup inspection:
W_R_NDI = Rx(pi/2)*Rz(-pi/2);

%input calibration of sensor to camera frame:
s_R_c = Rx(u(1))*Ry(u(2))*Rz(u(3));

%Get Camera orientation from ndi
%NDI = NDIcsv2structA('NDIdata_snakeravenTEST_VISION_a_direction.csv',false);
NDI = NDIcsv2structA('NDIdata_snakeravenTEST_VPC_A_success.csv',false);
R_ndi = zeros(3,3,NDI.N);
for ii = 1:NDI.N
    R_ndi(:,:,ii) = W_R_NDI * NDI.A.T(1:3,1:3,ii) * s_R_c;
end

%Get camera orientation from forward kinematics
%SnakeCamPose = SnakeRavencsv2struct('A_direction/SnakeRavenCameraData.csv');
SnakeCamPose = SnakeRavencsv2struct('Z_VPC_A_success/SnakeRavenCameraData.csv');
R_cam = SnakeCamPose.Tend(1:3,1:3,:);

% new way of measuring error:
% Get trajectory point and plot
r = -1.0001; %radius arm of sphere
p = [0 0 r]';

%NDI points p1
Nndi = length(R_ndi);
p1 = zeros(Nndi,3);
for ii=1:Nndi
    %R_ndi,R_cam_R_tar    
    p1_ = R_ndi(:,:,ii)*p;   
    p1(ii,:) = p1_';   
end

%Cam pose points:
Ncam = length(R_cam);
p2 = zeros(Ncam,3);
for ii=1:Ncam
    %R_ndi,R_cam_R_tar    
    p2_ = R_cam(:,:,ii)*p;   
    p2(ii,:) = p2_';   
end


% Mean
mu = abs(mean(p1,1) - mean(p2,1));
sigma = abs(std(p1,1) - std(p2,1));

out = sum(mu + sigma);

% %Decide how to truncate the data to be the same size:
% Nndi = length(R_ndi);
% Ncam = length(R_cam);
% if Ncam>Nndi
%     R_cam = R_cam(:,:,1:Ncam/Nndi:end); N = Nndi;
% else
%     R_ndi = R_ndi(:,:,1:Nndi/Ncam:end); N = Ncam;
% end

% %Compute error between the rotation matrices:
% error = zeros(3,1);
% for ii=1:N
%     T0 = [R_cam(:,:,ii) [0 0 0]'; 0 0 0 1];
%     T1 = [R_ndi(:,:,ii) [0 0 0]'; 0 0 0 1];
%     err = trans2dx(T0,T1);
%     error = error + abs(err(4:6,1)); %sum of square error!
% end
% out = norm(error/N);
end