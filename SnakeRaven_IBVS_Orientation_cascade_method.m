%% IBVS demo in matlab, SnakeRaven Cascade Method WORKS
% + region reaching to a circular region
% + orientation control only whilst translation control for camera
%Written by Andrew Razjigaev rewritten in 2021 from 2017 version

clc
clear all
close all

%% Teleoperation Task

%Simulate motion in +x direction for camera teleop command:
%Control loop iterations:
iterations = 100;
v_c = [zeros(1,iterations); [0.1*ones(1,iterations/2) zeros(1,iterations/2)] ; zeros(1,iterations)];               
%v_c = [zeros(1,iterations); 0.05*ones(1,iterations) ; zeros(1,iterations)]; 

%Region reaching area for orientation control:
width = 384;
region_radius = width/5;
% center = [width/2,width/2];
center = [0,0];

%% Initialise simulator

%Setup SnakeRaven
S = SnakeRaven;

%3D position [x y z] the four target dots in T frame:
target_points = [ 0.2, 0.2, 0;
                 -0.2, 0.2, 0;
                 -0.2,-0.2, 0;
                  0.2,-0.2, 0];
Tg = S.Tend; 
T = S.Tend * txyz(0,0,2) * [Rx(0)*Ry(0) [0 0 0]'; 0 0 0 1];
target_points = TransformPoints(T,target_points);
              
%Desired image [u,v] position of the four target dots:
side = width;
% desired_image_points = [ 0,0;
%                          0, side;
%                          side, 0;
%                          side side];

desired_image_points = [ -side/2,-side/2;
                         side/2,-side/2;
                         side/2,side/2;
                         -side/2,side/2;];
                     
%Camera initial pose (looking Down at target rather than up)
Tgc = txyz(0, 0, -4); Tgc(1:3,1:3) = Rx(0); %Gripper to Camera
W_T_c = Tg*Tgc;
c_T_g = inv(Tgc);
%c_J_g = T2Jacobian(Tgc);

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
hold on
plotcircle(center(1),center(2),region_radius);

for ii = 1:iterations
    disp(ii)
    %Take picture:
    [image_features,image_Jacobian] = take_picture_perspective(W_T_c,target_points,2,desired_image_points);
    hold on
    plotcircle(center(1),center(2),region_radius);
    
    %Camera noise:
    noise = 10*randn(4,2);
    image_features = image_features + noise;

%     %Feature Correspondance:
%     P_desired = [desired_image_points(1,:)'; desired_image_points(2,:)';
%                  desired_image_points(3,:)'; desired_image_points(4,:)'];
% 
    p = [image_features(1,:)'; image_features(2,:)';
         image_features(3,:)'; image_features(4,:)'];
% 
%     %Control Error
%     error = (P_desired - p);
    error = circle_region_error(p,center,region_radius);

    %Image Jacobian With Depth estimate:
    Jt = image_Jacobian(:,1:3); %translation partition
    Jw = image_Jacobian(:,4:6); %rotation partition
    Jp = [(Jt*inv_Z) Jw];

    %adaptive gain Lambda
    %Lambda = Adaptive_gain(Jp,4,0.4,30);
    Lambda = 1;

    %Control Law for IBVS
    %dw = Lambda*pinv(Jp(:,4:6))*(error - Jt*v_c(:,ii));
    %dw = pinv(Jp(:,4:6))*(Lambda*error + Jt*v_c(:,ii));
    dw = Lambda*pinv(Jp(:,4:6))*error;
    
    %limit the magnitude:
    Vmax = 0.05;
    for j=1:3
        if abs(dw(j))>Vmax
            dw(j) = Vmax*(dw(j)/abs(dw(j)));
        end
    end
    
    %Noise:
    dx = [v_c(:,ii); dw];
    %noise = 0.01*randn(6,1);
    %dx = dx + noise;
    
    %Update Camera Motion
    W_T_c = Tnorm(W_T_c * dx2trans(dx));
    
    %Compute New Endeffector Pose for inverse kinematics:
    %[success,S] = S.IK(W_T_c * c_T_g); %Do IK to that target pose (works)
    S = S.IK_update(W_T_c * c_T_g); %Just do one iteration of IK towards
    W_T_c = S.Tend*Tgc;
    
    %Approximate new depth Z:
%     v = dx(1:3); w = dx(4:6);
%     J = Jt*v;
%     d_inv_Z = J\(P_desired -Jw*w);
%     dZ = 1/d_inv_Z;
%     Z = Z + Lambda*dZ;
%     inv_Z = 1/Z;
    
    %Record update:
    Cam_traj(:,:,ii) = W_T_c;
    error_traj(:,ii) = error;
    
%     %Stop command %dx
%     if norm(error)<200
%         break;
%     end
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
    plotcoord3(Cam_traj(:,:,ii),0.5,'b-','g-','r-');
end
title('3D View (X-Y-Z)')
xlabel('X displacement')
ylabel('Y displacement')
zlabel('Z displacement')
axis equal
grid on
%axis([-305 -295 55 65 -45 -20])
