%% IBVS demo in matlab, 
% + approximates depth Z over time from initial estimate
% + has guassian noise from camera motion
% + uses adaptive gain for fast convergence visp ideas:
% + Looks down at target rather than up (notice feature correpondence)
%Written by Andrew Razjigaev rewritten in 2021 from 2017 version

clc
clear all
close all

% cd ..
% addpath('Math_functions');
% addpath('Plotting_functions');
% addpath('SnakeRaven_kinematics');
% addpath('IBVS_demo');
% cd('IBVS_demo');

%% Initialise simulator

%Control loop iterations:
iterations = 50;

%3D position [x y z] the four target dots in world frame:
target_points = [ 1, -1, 0;
                 -1, -1, 0;
                 -1,  1, 0;
                  1,  1, 0];
              
%Desired image [u,v] position of the four target dots:
desired_image_points = [ 1000, 1000;
                        -1000, 1000;
                        -1000,-1000;
                         1000,-1000];
                     
%Camera initial pose (looking Down at target rather than up)
W_T_c = txyz(1,1,3);
W_T_c(1:3,1:3) = Rx(pi-0.1)*Ry(0.1)*Rz(0.6);

%Record Camera trajectory over time:
Cam_traj = zeros(4,4,iterations);
N = size(target_points,1);
error_traj = zeros(2*N,iterations);

%Z initial approximation distance to target
Z = 1;
inv_Z = 1/Z;

%% IBVS CONTROL LOOP SIMULATION
%Start picture:
[image_features,image_Jacobian] = take_picture_perspective(W_T_c,target_points,1,desired_image_points);

for ii = 1:iterations
    %Take picture:
    [image_features,image_Jacobian] = take_picture_perspective(W_T_c,target_points,2,desired_image_points);

    %Feature Correspondance:
    P_desired = [desired_image_points(1,:)'; desired_image_points(2,:)';
                 desired_image_points(3,:)'; desired_image_points(4,:)'];

    p = [image_features(1,:)'; image_features(2,:)';
         image_features(3,:)'; image_features(4,:)'];

    %Control Error
    error = (P_desired - p);

    %Image Jacobian With Depth estimate:
    Jt = image_Jacobian(:,1:3); %translation partition
    Jw = image_Jacobian(:,4:6); %rotation partition
    Jp = [(Jt*inv_Z) Jw];

    %adaptive gain Lambda
    Lambda = Adaptive_gain(Jp,4,0.4,30);

    %Control Law for IBVS
    dx = Lambda*pinv(Jp)*error;
    
    %limit the magnitude:
    Vmax = 0.05;
    for j=1:6
        if abs(dx(j))>Vmax
            dx(j) = Vmax*(dx(j)/abs(dx(j)));
        end
    end
    
    %Noise:
    noise = 0.01*randn(6,1);
    dx = dx + noise;
    
    %Update Camera Motion
    W_T_c = Tnorm(W_T_c * dx2trans(dx));
    
    %Approximate new depth Z:
    v = dx(1:3); w = dx(4:6);
    J = Jt*v;
    d_inv_Z = J\(P_desired -Jw*w);
    dZ = 1/d_inv_Z;
    Z = Z + Lambda*dZ;
    inv_Z = 1/Z;
    
    %Record update:
    Cam_traj(:,:,ii) = W_T_c;
    error_traj(:,ii) = error;
    
    %Stop command
    if norm(dx)<0.0001
        break;
    end
end

%% PLOT RESULTS
%Error Figure
figure(3)
plot(1:iterations,error_traj)
title('Error in IBVS')
xlabel('Time in iterations')
ylabel('Error')

%3D PLOT OF VISUAL SERVO TRAJECTORY!!!!!
figure('Name','Task Space','units','normalized','outerposition',[0 0 1 1])
figure(4)
plot3(target_points(:,1),target_points(:,2),target_points(:,3),'ro')
hold on
for ii = 1:iterations
    plotcoord3(Cam_traj(:,:,ii),0.1,'b-','g-','r-');
end
title('3D View (X-Y-Z)')
xlabel('X displacement')
ylabel('Y displacement')
zlabel('Z displacement')
axis equal
grid on
