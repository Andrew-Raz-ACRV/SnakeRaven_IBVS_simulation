function visual_servoing_simulation_visp_improvements()
% IBVS demo in matlab, approximates depth Z
% + has guassian noise for motion + uses visp ideas:
%Written by Andrew Razjigaev 2017

clc
close all

%initialise simulator
%position the four target dots:
target_points = [ 1, 1, 0;
    -1, 1, 0;
    -1,-1, 0;
    1,-1, 0];

%Camera initial pose
x = 0.3; y = 0.3; z = 2.3; thr = -pi/10; thp = pi/10; thy = 0.8*pi;

%x = -0.3; y = -0.3; z = 2.5; thr = pi/10; thp = -pi/10; thy = 0.4*pi;
%World_T_Camera = txyz(-0.2,0.2,1);
%World_T_Camera(1:3,1:3) = Rz(-0.6)*Ry(0.3)*Rx(-0.1);

%x = -0.2; y = 0.2; z = 1; thr = -0.1; thp = 0.3; thy = -0.6;

Camera_pose = [x,y,z,thr,thp,thy];

%Z approximator: notice measured in negative axis
Z = -1.5;
inv_Z = 1/Z;

%desired image position of the four target dots:
desired_image_points = [ 1000, 1000;
    -1000, 1000;
    -1000,-1000;
    1000,-1000];
P_desired = [desired_image_points(1,:)'; desired_image_points(2,:)';
    desired_image_points(3,:)'; desired_image_points(4,:)'];

%Initial Picture:
%Take picture:
[image_features,~] = take_picture_perspective(Camera_pose,target_points);
figure(1);
clf
grid on; axis equal; hold on
plot(desired_image_points(:,1),desired_image_points(:,2),'bo')
hold on
plot(image_features(:,1),image_features(:,2),'ro')
title('Initial Image Plane')
xlabel('u')
ylabel('v')
legend('Desired Image feature positions','Actual Image feature positions','Location','southoutside')

%Vectors to record data:
error_graph =[];
x_graph = x; y_graph = y; z_graph = z; roll_graph = thr; pitch_graph = thp;
yaw_graph = thy;

%Control Loop
time_end = 150;
for i = 1:time_end
    
    %Take picture:
    [image_features,Jt,Jw] = take_picture_perspective(Camera_pose,target_points);
    figure(2);
    grid on; axis equal; hold on
    plot(desired_image_points(:,1),desired_image_points(:,2),'bo')
    hold on
    plot(image_features(:,1),image_features(:,2),'ro')
    title('Image Plane - During Control Loop')
    xlabel('u')
    ylabel('v')
    
    %Control Law
    p = [image_features(1,:)'; image_features(2,:)';
        image_features(3,:)'; image_features(4,:)'];
    %Lambda = 0.02; % Without adaptive gain
    error = (P_desired - p);
    
    %Unite Jacobian
    Jp = [(Jt*inv_Z) Jw];
    
    %adaptive gain
    Lambda = Adaptive_gain(Jp,4,0.4,30);
    Jp_pseudo = pinv(Jp);
    vel = Lambda*Jp_pseudo*error;
    Vmax = 0.05;
    
    %limit the magnitude:
    for j=1:6
        if abs(vel(j))>Vmax
            vel(j) = Vmax*(vel(j)/abs(vel(j)));
        end
    end

    %limit the magnitude
%    if norm(vel)>Vmax
%        vel = Vmax*vel/norm(vel);
%    end
    
    %Update the Camera movement:
    %Get the change
    %Note the image flow reversion x-y needs to be negative for converging
    dx = -vel(1); dy = -vel(2); dz = vel(3); dthr = -vel(4);
    dthp = -vel(5); dthy = vel(6);
    
    %Get the current:
    x = Camera_pose(1); y = Camera_pose(2); z = Camera_pose(3);
    thr = Camera_pose(4); thp = Camera_pose(5); thy = Camera_pose(6);
    
    %Movement noise 1 cm of error
    noise = 0.01*randn(1,6);
    
    %%Integrate the position
    Camera_pose = [x+dx,y+dy,z+dz,thr+dthr,thp+dthp,thy+dthy]+noise;
    
    %Approximate new Z:
    v = [dx; dy; dz]; w = [dthr; dthp; dthy];
    J = Jt*v;
    d_inv_Z = J\(P_desired -Jw*w);
    dZ = 1/d_inv_Z;
    Z = Z + Lambda*dZ;
    inv_Z = 1/Z;
    
    %Store error
    error_graph = [error_graph, error];
    
    %Store the position
    x_graph = [x_graph; Camera_pose(1)]; y_graph = [y_graph; Camera_pose(2)];
    z_graph = [z_graph; Camera_pose(3)]; roll_graph = [roll_graph; Camera_pose(4)];
    pitch_graph = [pitch_graph; Camera_pose(5)];
    yaw_graph = [yaw_graph; Camera_pose(6)];
end

%FIGURES******
%Error Figure
figure(3)
plot(1:time_end,error_graph)
title('Error in IBVS')
xlabel('Time in iterations')
ylabel('Error')

% %X-Y movement figure
% figure(4)
% grid on; axis equal; hold on
% for i = 1:length(x_graph)
%     plot_coord(x_graph(i),y_graph(i),yaw_graph(i),0.1,'b-','g-')
% end
% %plot(x_graph,y_graph,'k+')
% hold on
% plot(target_points(:,1),target_points(:,2),'ro')
% title('Above View (X-Y)')
% xlabel('X displacement')
% ylabel('Y displacement')
% legend('X axis','Y axis')
% 
% %X-Z movement figure
% figure(5)
% grid on; axis equal; hold on
% for i = 1:length(x_graph)
%     plot_coord(x_graph(i),z_graph(i),pitch_graph(i),0.1,'b-','r-')
% end
% %plot(x_graph,z_graph,'k*')
% hold on
% plot(target_points(:,1),target_points(:,3),'ro')
% title('Front View (X-Z)')
% xlabel('X displacement')
% ylabel('Z displacement')
% legend('X axis','Z axis')
% 
% %Y-Z movement figure
% figure(6)
% grid on; axis equal; hold on
% for i = 1:length(y_graph)
%     plot_coord(y_graph(i),z_graph(i),roll_graph(i),0.1,'g-','r-')
% end
% %plot(y_graph,z_graph,'k*')
% hold on
% plot(target_points(:,1),target_points(:,3),'ro')
% title('Side View (Y-Z)')
% xlabel('Y displacement')
% ylabel('Z displacement')
% legend('Y axis','Z axis')

%3D PLOT OF VISUAL SERVO TRAJECTORY!!!!!
figure('Name','Task Space','units','normalized','outerposition',[0 0 1 1])
%figure(7)
plot3(target_points(:,1),target_points(:,2),target_points(:,3),'ro')
hold on
%plot3(x_graph,y_graph,z_graph,'k+')
for i = 1:length(x_graph)
    plot_coord3(x_graph(i),y_graph(i),z_graph(i),roll_graph(i),pitch_graph(i),yaw_graph(i),0.1,'b-','g-','r-');
end
title('3D View (X-Y-Z)')
xlabel('X displacement')
ylabel('Y displacement')
zlabel('Z displacement')
axis equal
grid on

end

%Adaptive Gain
function Lambda = Adaptive_gain(Jp,L0,Linf,L_dot0)
x = norm(Jp,inf);
a = L0 - Linf;
b = L_dot0/a;
c = Linf;
Lambda = a*exp(-b*x)+c;
end

%PLOTTING FUNCTIONS
function plot_coord(x,y,theta,axis_length,X_colour,Y_colour)
%Axis Lengths of the axes on the robot coordinate frame
px = [axis_length; 0; 1]; py = [0; axis_length; 1];
%Homogeneous transform of the axes
transform = [ cos(theta) -sin(theta) x;
    sin(theta) cos(theta) y;
    0 0 1];
%Computed points of axes:
px_point = transform*px;
py_point = transform*py;
%X axis
plot([x; px_point(1)], [y; px_point(2)], X_colour)
hold on
%Y axis
plot([x; py_point(1)], [y; py_point(2)], Y_colour)
hold on
end

%TRANSFORMATION FUNCTIONS:
%Rotation matrices
function [R_z] = Rotz(th)
R_z = [cos(th) -sin(th) 0;
    sin(th) cos(th) 0;
    0 0 1];
end
function [R_y] = Roty(th)
R_y = [cos(th) 0 sin(th) ;
    0 1 0;
    -sin(th) 0 cos(th)];
end
function [R_x] = Rotx(th)
R_x = [1 0 0;
    0 cos(th) -sin(th);
    0 sin(th) cos(th)];
end

function [transformation] = pose_transform(pose_vector)
%Extract pose elements
x = pose_vector(1); y = pose_vector(2); z = pose_vector(3);
thr = pose_vector(4); thp = pose_vector(5); thy = pose_vector(6);
%Compute the Rotation:
Rx = Rotx(thr); Ry = Roty(thp); Rz = Rotz(thy);
R = Rx*Ry*Rz;
% Compute the t distance
t = [x; y; z];
%Compute the transform
transformation = [ R, t;
    zeros(1,3) 1];
end

%3D plot coordinate system function
function plot_coord3(x,y,z,roll,pitch,yaw,axis_length,X_colour,Y_colour,Z_colour)
%Axis Lengths of the axes on the robot coordinate frame
px = [axis_length; 0; 0; 1]; py = [0; axis_length; 0; 1]; pz = [0; 0; axis_length; 1];
%Homogeneous transform of the axes
transform = [Rotx(roll)*Roty(pitch)*Rotz(yaw) [x , y, z]'; zeros(1,3) 1];
%Computed points of axes:
px_point = transform*px; py_point = transform*py; pz_point = transform*pz;
%X axis
plot3([x; px_point(1)], [y; px_point(2)], [z; px_point(3)], X_colour)
hold on
%Y axis
plot3([x; py_point(1)], [y; py_point(2)], [z; py_point(3)], Y_colour)
hold on
%Z axis
plot3([x; pz_point(1)], [y; pz_point(2)], [z; pz_point(3)], Z_colour)
hold on
end

%CAMERA MODEL
function [image_features,Jt,Jw] = take_picture_perspective(camera_pose,points)
%Camera model, perspective
% provided that points [x y z] are in world frame,
% this find the points relative to camera pose [x,y,z,th...]
%Find World to Camera transform
W_T_c = pose_transform(camera_pose);
%Focal and pixel dimensions
f = 0.015; pu =10*10^-6; pv = pu; %pw = 10^-6; ph = 10^-6;
%f = 1; pw = 1; ph = 1;
uo = 0; vo = 0;
%intrinsic features
%camera matrix
cam_mat = [f 0 0 0;
    0 f 0 0;
    0 0 1 0];
%Image plane K
K = [(1/pu) 0 uo;
    0 (1/pv) vo;
    0 0 1];
%initialise output
image_features = zeros(4,2);
image_Jacobian = zeros(8,6);
for i = 1:length(points)
    %Extract homogenous
    W_P = [points(i,:) 1]';
    %Find camera to point distance
    C_P = (W_T_c) \ W_P;
    %Z = C_P(3);
    %Camera Model
    p_ = K*cam_mat*C_P;
    %convert out of homogeneous
    p = p_/p_(3);
    u = p(1);
    v = p(2);
    %image_features = [image_features; u, v];
    image_features(i,:) = [u,v];
    %Compute the image jacobian for that point:
    image_Jacobian(((2*i)-1):(2*i),:) = [-f/(pu) 0 u (pu*u*v)/f -(f^2 + pu^2 * u^2)/(pu*f) v;
        0 -f/(pv) v (f^2 + pv^2 * v^2)/(pv*f) -(pv*u*v)/f -u];
end

%Split the Jacobian:
Jt = image_Jacobian(:,1:3);
Jw = image_Jacobian(:,4:6);
end