%% Orientation Partition IBVS demo in matlab, SnakeRaven Cascade Method
% + region reaching to a circular region
% + orientation control only whilst translation control for camera
%Written by Andrew Razjigaev rewritten in 2021, 
% rewritten for Tmech revision 2023

clc
clear all
close all

addpath('IBVS_demo')
addpath('Math_functions')
addpath('Plotting_functions')
addpath('SnakeRaven_kinematics')

%% Settings:
IBVSOP_algorithm = 1; %activates IBVS OP if true, VPC for false
translation_feed = 1; %activates knowledge of teleop command

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

%number of features
ff = 4;

%3D position [x y z] the four target dots in T frame:
switch ff
    case 4
        target_points = [ 1, 1, 0;
                     -1, 1, 0;
                     -1,-1, 0;
                      1,-1, 0];
    case 3
        target_points = [ 0, 1, 0;
                     -1,-1, 0;
                      1,-1, 0];
    case 2
        target_points = [ 0, 1, 0;
                       0,-1, 0];
    case 1
        target_points = [ 0, 0, 0];
        region_radius = 0;
    otherwise
        target_points = [ 1, 1, 0;
                     -1, 1, 0;
                     -1,-1, 0;
                      1,-1, 0];
end

Tg = S.Tend; 
%T = S.Tend * txyz(0,0,3) * [Rx(0.2)*Ry(0.2) [5 0 0]'; 0 0 0 1];
T = S.Tend * txyz(0,0,3) * [Rx(0)*Ry(0) [0 0 0]'; 0 0 0 1];
target_points = TransformPoints(T,target_points);
              
%Desired image [u,v] position of the four target dots:
side = width/2;
desired_image_points = [ side,-side;
                         side, side;
                         -side, side;
                         -side -side;
                         side,-side];
                     
%Camera initial pose (looking Down at target rather than up)
Tgc = txyz(0.04, 0.05, -4); Tgc(1:3,1:3) = Rx(0.01); %Gripper to Camera
W_T_c = Tg*Tgc;
c_T_g = inv(Tgc);

%Record Camera trajectory over time:
Cam_traj = zeros(4,4,iterations);
N = size(target_points,1);
error_traj = zeros(2*N,iterations);
feat_traj = zeros(2*N,iterations);
vc_traj = zeros(6,iterations);

%Frequency of Camera frame rate:
dT = 1/30;

%Z initial approximation distance to target
Z = 5;
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

    %Feature Correspondance:
    nn = size(image_features,1);
    p = zeros(nn*2,1);
    for jj=1:size(image_features,1)
        p(2*jj-1) = image_features(jj,1);
        p(2*jj) = image_features(jj,2);
    end
 
%     %Pixel noise
%     noise = 10*randn(length(p),1);
%     p = p + noise;   
  
    %Control Error
    error = circle_region_error(p,center,region_radius);

    %Image Jacobian With Depth estimate:
    Jt = image_Jacobian(:,1:3); %translation partition
    Jw = image_Jacobian(:,4:6); %rotation partition
    Jp = [(Jt*inv_Z) Jw];

    if(IBVSOP_algorithm)
        %% Classical IBVS
        %adaptive gain Lambda from visp
        %Lambda = Adaptive_gain(Jp,4,0.4,30);
        %Lambda = 10;
        %Lambda = 1.5;
    
        %Control Law for IBVS
        if(translation_feed)
            Lambda = 1.5;
            dw = pinv(Jp(:,4:6))*(Lambda*error-Jp(:,1:3)*v_c(:,ii));
        else
            Lambda = 10;
            dw = Lambda*pinv(Jp(:,4:6))*error;
        end
  
    else

        %% Model Predictive Control:
    
        %Tuning Parameters:
        n = length(image_features(:));
        Q = 10*eye(n);
        R = eye(3);
        Np = 10;
    
        %Cost Function:
        %fun = @(x)1+x(1)/(1+x(2)) - 3*x(1)*x(2) + x(2)*(1+x(1));
        fun = @(u)CostFunction(u,p,Jw,Jt,v_c(:,ii),Q,R,center,region_radius,Np,dT,translation_feed);
        dw0 = zeros(3,Np);
        % Input Bounds
        Vmax = 0.05;
        lb = -Vmax*ones(3,Np);
        ub = Vmax*ones(3,Np);
        A = [];
        b = [];
        Aeq = [];
        beq = [];
    
        OPTIONS = optimoptions('fmincon','Algorithm','sqp');
        dwNp = fmincon(fun,dw0,A,b,Aeq,beq,lb,ub,[],OPTIONS);
        dw = dwNp(:,1);

    end
    
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
    %[success,S] = S.IK(W_T_c * c_T_g); %Do IK to that target pose 
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
    vc_traj(:,ii) = dx/dT;
    
    % %Stop command %dx
    % if norm(error)<200
    %     break;
    % end
end

%% PLOT RESULTS
%3D PLOT OF VISUAL SERVO TRAJECTORY!!!!!
f1 = figure;
%subplot(1,4,1)

%Full screen
%figure('Name','Task Space','units','normalized','outerposition',[0 0 1 1])
%figure(4)
%subplot(1,2,1)
plot3(target_points(:,1),target_points(:,2),target_points(:,3),'ro')
hold on
for ii = 1:iterations
    plotcoord3(Cam_traj(:,:,ii),0.5,'r-','g-','b-');
end
%title('3D View of Camera Trajectory for Orientation Partition-IBVS')
title('Camera Pose Trajectory')
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')
axis equal
grid on
view([30 10])
%axis([-305 -295 55 65 -45 -20])
f1.Position(3) = 300;
f1.Position(4) = 300;

%Feature Trajectory
f2 = figure;
%subplot(2,2,2)
%subplot(1,4,2)
%figure('Name','Task Space','units','normalized','outerposition',[0 0 1 1])
grid on; axis equal; 
hold on
plotcircle(center(1),center(2),region_radius);
hold on
plot(desired_image_points(:,1),desired_image_points(:,2),'b-','HandleVisibility','off')
for ii=1:2:2*nn
    hold on
    plot(feat_traj(ii,1:iterations),feat_traj(ii+1,1:iterations),'g--')
    hold on
    plot(feat_traj(ii,1),feat_traj(ii+1,1),'ko')
    hold on
    plot(feat_traj(ii,iterations),feat_traj(ii+1,iterations),'b+')
end
%title('Orientation Partition-IBVS Feature Motion on Image Plane')
title('Feature Motion on Image Plane')
xlabel('x - pixels')
ylabel('y - pixels')
axis([-side side -side side])
%legend('Central FOV','Boundary','Trajectory','Initial Points','Final Points','Location','southeast')
legend('Central FOV','Trajectory','Initial Points','Final Points','Location','southeast')
f2.Position(3) = 300;
f2.Position(4) = 300;

%Error Figure
f3 = figure;
%subplot(1,4,3)
%figure('Name','Task Space','units','normalized','outerposition',[0 0 1 1])
%subplot(2,2,4)
plot(dT*(1:iterations),error_traj)
title('Feature Error over Time')
xlabel('Time [s]')
ylabel('Error [Pixels]')
axis([0 5.5 -40 40])
switch ff
    case 4
        lgd = legend('e_{1x}','e_{1y}','e_{2x}','e_{2y}','e_{3x}','e_{3y}','e_{4x}','e_{4y}','Location','east','Orientation','horizontal'); lgd.NumColumns = 1;
    case 3
        lgd = legend('e_{1x}','e_{1y}','e_{2x}','e_{2y}','e_{3x}','e_{3y}','Location','east','Orientation','horizontal'); lgd.NumColumns = 1;
    case 2
        lgd = legend('e_{1x}','e_{1y}','e_{2x}','e_{2y}','Location','east','Orientation','horizontal'); lgd.NumColumns = 1;
    case 1
        lgd = legend('e_{1x}','e_{1y}','Location','east','Orientation','horizontal'); lgd.NumColumns = 1;
end
f3.Position(3) = 300;
f3.Position(4) = 300;
%lgd.Position(1) = 300/2 - lgd.Position(3)/2;
%lgd.Position(2) = 300/2 - lgd.Position(4)/2;


%Input Figure
f4 = figure;
%subplot(1,4,4)
%figure('Name','Task Space','units','normalized','outerposition',[0 0 1 1])
%subplot(2,2,4)
plot(dT*(1:iterations),vc_traj)
title('Camera Velocity over Time')
xlabel('Time [s]')
ylabel('Camera Velocity [mm/s] or [rad/s]')
axis([0 5.5 -2 3])
%lgd2 = legend('v_x','v_y','v_z','\omega_x','\omega_y','\omega_z','Location','north','Orientation','horizontal'); lgd2.NumColumns = 3;
lgd2 = legend('v_x','v_y','v_z','\omega_x','\omega_y','\omega_z','Location','east','Orientation','horizontal'); lgd2.NumColumns = 1;
f4.Position(3) = 300;
f4.Position(4) = 300;
