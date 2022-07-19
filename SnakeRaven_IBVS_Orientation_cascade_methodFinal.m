%% Orientation Partition IBVS demo in matlab, SnakeRaven Cascade Method
% + region reaching to a circular region
% + orientation control only whilst translation control for camera
%Written by Andrew Razjigaev rewritten in 2021

clc
clear all
close all

%% Teleoperation Task

%Simulate motion in +x direction for camera teleop command:
%Control loop iterations:
iterations = 100;
%v_c = [zeros(1,iterations); [0.1*ones(1,iterations/2) zeros(1,iterations/2)] ; zeros(1,iterations)];               
%v_c = [0.1*ones(1,iterations); zeros(1,iterations) ; zeros(1,iterations)]; 
 

%Default
v_c = [zeros(1,iterations); 0.1*ones(1,iterations); zeros(1,iterations)];

%Intermitant
% iterations = 500;
% v_c = [zeros(1,iterations); zeros(1,iterations); zeros(1,iterations)];
% for ii=1:5:iterations
%     v_c(2,ii) = 0.1;
% end

%Region reaching area for orientation control:
width = 384;
region_radius = width/6;
center = [0,0]; %[width/2,width/2];

%% Initialise simulator

%Setup SnakeRaven
S = SnakeRaven;

%3D position [x y z] the four target dots in T frame:
target_points = [ 1, 1, 0;
                 -1, 1, 0;
                 -1,-1, 0;
                  1,-1, 0];
Tg = S.Tend; 
%T = S.Tend * txyz(0,0,3) * [Rx(0.2)*Ry(0.2) [5 0 0]'; 0 0 0 1];
T = S.Tend * txyz(0,0,3) * [Rx(0)*Ry(0) [0 0 0]'; 0 0 0 1];
target_points = TransformPoints(T,target_points);
              
%Desired image [u,v] position of the four target dots:
side = width/2;
% desired_image_points = [ 0,0;
%                          0, side;
%                          side, 0;
%                          side side];
desired_image_points = [ side,-side;
                         side, side;
                         -side, side;
                         -side -side;
                         side,-side];
                     
%Camera initial pose (looking Down at target rather than up)
Tgc = txyz(0.04, 0.05, -4); Tgc(1:3,1:3) = Rx(0.01); %Gripper to Camera
W_T_c = Tg*Tgc;
c_T_g = inv(Tgc);
%c_J_g = T2Jacobian(Tgc);

%Record Camera trajectory over time:
Cam_traj = zeros(4,4,iterations);
N = size(target_points,1);
error_traj = zeros(2*N,iterations);
feat_traj = zeros(2*N,iterations);

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

%     %Feature Correspondance:
%     P_desired = [desired_image_points(1,:)'; desired_image_points(2,:)';
%                  desired_image_points(3,:)'; desired_image_points(4,:)'];
% 
    p = [image_features(1,:)'; image_features(2,:)';
         image_features(3,:)'; image_features(4,:)'];
     
%     noise = 10*randn(length(p),1);
%     p = p + noise;   
% 
%     %Control Error
%     error = (P_desired - p);
    error = circle_region_error(p,center,region_radius);

    %Image Jacobian With Depth estimate:
    Jt = image_Jacobian(:,1:3); %translation partition
    Jw = image_Jacobian(:,4:6); %rotation partition
    Jp = [(Jt*inv_Z) Jw];

    %% Classical IBVS
    %adaptive gain Lambda
    %Lambda = Adaptive_gain(Jp,4,0.4,30);
    %Lambda = 10;
    Lambda = 1.5;

    %Control Law for IBVS
    %dw = Lambda*pinv(Jp(:,4:6))*error;
    dw = Lambda*pinv(Jp(:,4:6))*(error-Jp(:,1:3)*v_c(:,ii));
    
%     %% Model Predictive Control:
%     
%     %Tuning Parameters:
%     n = length(image_features(:));
%     Q = 10*eye(n);
%     R = eye(3);
    dT = 1/30;
%     Np = 10;
%     
%     %Cost Function:
%     %fun = @(x)1+x(1)/(1+x(2)) - 3*x(1)*x(2) + x(2)*(1+x(1));
%     fun = @(u)CostFunction(u,p,Jw,Jt,v_c(:,ii),Q,R,center,region_radius,Np,dT);
%     dw0 = zeros(3,Np);
%     % Input Bounds
%     Vmax = 0.05;
%     lb = -Vmax*ones(3,Np);
%     ub = Vmax*ones(3,Np);
%     A = [];
%     b = [];
%     Aeq = [];
%     beq = [];
%     
%     OPTIONS = optimoptions('fmincon','Algorithm','sqp');
%     %x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options)
%     dwNp = fmincon(fun,dw0,A,b,Aeq,beq,lb,ub,[],OPTIONS);
%     dw = dwNp(:,1);
    
    %% Continue:
    Vmax = 0.05;
    %limit the magnitude:
    for j=1:3
        if abs(dw(j))>Vmax
            dw(j) = Vmax*(dw(j)/abs(dw(j)));
        end
    end
    
    %Noise:
    dx = [v_c(:,ii); dw];
%     noise = 0.05*randn(6,1);
%     dx = dx + noise;   

    %Update Camera Motion
    W_T_c = Tnorm(W_T_c * dx2trans(dx));
    
    %Compute New Endeffector Pose for inverse kinematics:
    %[success,S] = S.IK(W_T_c * c_T_g); %Do IK to that target pose (works)
    S = S.IK_update(W_T_c * c_T_g); %Just do one iteration of IK towards
    %S = S.IK_update(S.Tend * Tnorm(dx2trans(dx))); %Just do one iteration of IK towards
    
    W_T_c = S.Tend*Tgc;
    
    %Approximate new depth Z:
%     v = dx(1:3); w = dx(4:6);
%     J = Jt*v;
%     d_inv_Z = J\(P_desired -Jw*w);
%     dZ = 1/d_inv_Z;
%     Z = Z + Lambda*dZ;
%     inv_Z = 1/Z;
    
    %Record update:
    feat_traj(:,ii) = p;
    Cam_traj(:,:,ii) = W_T_c;
    error_traj(:,ii) = error;
    
%     %Stop command %dx
%     if norm(error)<200
%         break;
%     end
end

%% PLOT RESULTS
%3D PLOT OF VISUAL SERVO TRAJECTORY!!!!!
%figure;
figure('Name','Task Space','units','normalized','outerposition',[0 0 1 1])
%figure(4)
subplot(1,2,1)
plot3(target_points(:,1),target_points(:,2),target_points(:,3),'ro')
hold on
for ii = 1:iterations
    plotcoord3(Cam_traj(:,:,ii),0.5,'b-','g-','r-');
end
title('3D View of Camera Trajectory for Orientation Partition-IBVS')
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')
axis equal
grid on
view([30 10])
%axis([-305 -295 55 65 -45 -20])

%Feature Trajectory
subplot(2,2,2)
%figure(5)
grid on; axis equal; 
hold on
plotcircle(center(1),center(2),region_radius);
hold on
plot(desired_image_points(:,1),desired_image_points(:,2),'b-')
for ii=1:2:8
    hold on
    plot(feat_traj(ii,1:iterations),feat_traj(ii+1,1:iterations),'g--')
    hold on
    plot(feat_traj(ii,1),feat_traj(ii+1,1),'ko')
    hold on
    plot(feat_traj(ii,iterations),feat_traj(ii+1,iterations),'b+')
end
title('Orientation Partition-IBVS Feature Motion on Image Plane')
xlabel('x - pixels')
ylabel('y - pixels')
legend('Desired Region','Image Boundary','Feature Trajectory','Initial Points','Final Points','Location','Eastoutside')

%Error Figure
%figure(3)
subplot(2,2,4)
plot(dT*(1:iterations),error_traj)
title('Error over Time for Orientation Partition-IBVS')
xlabel('Time [s]')
ylabel('Error [Pixels]')
