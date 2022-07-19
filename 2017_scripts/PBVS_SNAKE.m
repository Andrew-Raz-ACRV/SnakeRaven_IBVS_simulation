function PBVS_SNAKE()
% This Function Runs the PBVS Control of the Snake Robot
% This is a simulator of the robot following a ball point
clc, close all
%Initial motor value test values
q1 = 50; q2 = 50; q6 = pi/3; q3 = 30;
r = 175.73; %mm
%Point for the camera to visual servo about
point = [60 60 200];
rad = 25/2;
%Control Loop
time_end = 25;
for k = 1:time_end
%GENERATE SNAKE ROBOT
[Base_2_Camera,Base_2_tool] = plot_snake_simulation(q1,q2,q3,q6,point,rad);
pause(0.0001)
%Take picture:
[image_features,depth] = take_picture_perspective(Base_2_Camera,point);
%POSITON COMPUTATION
%Estimate pose of the ball as a point:
[X,Y,Z] = estimate_pose(image_features,rad,depth);
%Put that pose into the base frame
Cam_2_point= [X, Y, Z, 1]';
P_desired = Base_2_Camera*Cam_2_point;
display(P_desired) % point
P_desired = [P_desired(1:3)' deg2rad(45)]; %desired grasping angle P_desired(1:3,1);
%Solve the Forward Kinematics
Pitch = atan(sin(q6)*tan(q2/r));
p = [Base_2_tool(1:3,4); Pitch];
%CONTROL LAW
%disp(q1), disp(q2), disp(q3), disp(rad2deg(q6))
dX = (P_desired') - (p);
%Break case close to target
if norm(dX)<1
break
end
%limit the magnitude:
Vmax = 25;
if norm(dX)>Vmax
dX = Vmax*(dX/norm(dX));
end
%Manipulator Jacobian
J = toolpoint_Jacobian(q2,q3,q6);
%Solve the motor rate of change:
dQ = J'/(J*J' + eye(4)) * dX;
%Update the joints:
dq1 = dQ(1); dq2 = dQ(2); dq3 = dQ(3); dq6 = dQ(4);
%Integrate to find new joint values:
q1 = q1 + dq1;
q2 = q2 + dq2;
q3 = q3 + dq3;
q6 = q6 + dq6;
end
end

function [Base_2_Camera,Base_2_tool] = plot_snake_simulation(q1,q2,q3,q6,pts,rad)
% This function generates the figures of the Snake Robot and the Camera
% view
% It also computes the base to camera transformation matrix
%CONSTANTS
r = 175.73; %mm
b = 5;
e = 0; %radians
q6 = q6-pi/2; %offset
%Forward Kinematics Transforms:
% q1 is a prismatic joint acting in the z-axis
Base_2_q1 = [1 0 0 0;
0 1 0 0;
0 0 1 q1;
0 0 0 1];
% q6 is a revolute joint acting about the z-axis
q1_2_q6 = [cos(q6) -sin(q6) 0 0;
sin(q6) cos(q6) 0 0;
0 0 1 0;
0 0 0 1];
% q2 is a curved tube(induces rotation about the X-axis and translation in
% the y and z-axes)
q6_2_q2 = [1 0 0 0;
0 cos(-q2/r) sin(-q2/r) r*(1-cos(q2/r));
0 -sin(-q2/r) cos(-q2/r) r*sin(q2/r);
0 0 0 1];
% The camera is -b mm offset underneath the curved tube and e is the z-axis
% rotation offset of the camera in its holder attached to the curved tube
q2_2_Camera = [cos(e) -sin(e) 0 0;
sin(e) cos(e) 0 -b;
0 0 1 0;
0 0 0 1];
Base_2_Camera = Base_2_q1*q1_2_q6*q6_2_q2*q2_2_Camera;
% q3 is the prismatic joint of the third tube
q2_2_q3 = [1 0 0 0;
0 1 0 0;
0 0 1 q3;
0 0 0 1];
%PLOT TUBE 1 LINE
pb = [0 0 0 1]';
p1 = Base_2_q1*pb;
figure(1), clf
x = [pb(1) p1(1)];
y = [pb(2) p1(2)];
z = [pb(3) p1(3)];
plot3(x,y,z,'r-','LineWidth',3);
%PLOT TUBE 2 LINE
pii2 = p1;
for ii = linspace(0,q2,100)
q6_2_q2 = [1 0 0 0;
0 cos(-ii/r) -sin(-ii/r) r*(1-cos(ii/r));
0 sin(-ii/r) cos(-ii/r) r*sin(ii/r);
0 0 0 1];
p2 = Base_2_q1*q1_2_q6*q6_2_q2*pb;
hold on
x = [pii2(1) p2(1)];
y = [pii2(2) p2(2)];
z = [pii2(3) p2(3)];
plot3(x,y,z,'b-','LineWidth',3);
pii2 = p2;
end
p2 = Base_2_q1*q1_2_q6*q6_2_q2*pb;
%PLOT CAMERA POINT
pC = Base_2_Camera*pb;
hold on
plot3(pC(1),pC(2),pC(3),'b*','LineWidth',3);
%PLOT TUBE 3 LINE
Base_2_tool = Base_2_q1*q1_2_q6*q6_2_q2*q2_2_q3;
p3 = Base_2_q1*q1_2_q6*q6_2_q2*q2_2_q3*pb;
hold on
x = [p2(1) p3(1)]; y = [p2(2) p3(2)]; z = [p2(3) p3(3)];
plot3(x,y,z,'g-','LineWidth',3);
hold on
%PLOT the target point
hold on
[xs,ys,zs] = sphere;
surf(xs*rad+pts(1), ys*rad+pts(2), zs*rad+pts(3));
title('3D View (X-Y-Z)'), xlabel('X displacement')
ylabel('Y displacement'), zlabel('Z displacement')
view(3)
daspect([1 1 1])
%Initial Picture:
%Take picture:
[image_features,~] = take_picture_perspective(Base_2_Camera,pts);
figure(2);
grid on; axis equal; hold on
plot(image_features(:,1),image_features(:,2),'ro')
title('Image Plane')
xlabel('u')
ylabel('v')
axis([-12000 12000 -12000 12000])
legend('centroid of sphere trajectory','Location','southoutside')
end
%CAMERA MODEL
function [image_features,depth] = take_picture_perspective(Base_2_Camera,point)
%Camera model, perspective
% provided that points [x y z] are in world frame,
% this find the points relative to camera pose [x,y,z,th...]
%Find World to Camera transform
W_T_c = Base_2_Camera;
%intrinsic features
%Focal and pixel dimensions
f = 0.015; pu =10^-5; pv = pu;
uo = 0; vo = 0;
%camera matrix
cam_mat = [f 0 0 0;
0 f 0 0;
0 0 1 0];
%Image plane K
K = [(1/pu) 0 uo;
0 (1/pv) vo;
0 0 1];
%Extract homogenous
W_P = [point 1]';
%Find camera to point distance
C_P = (W_T_c) \ W_P;
%Camera Model
p_ = K*cam_mat*C_P;
%depth
depth = C_P(3);
%convert out of homogeneous
p = p_/p_(3);
u = p(1);
v = p(2);
%image_features = [image_features; u, v];
image_features = [u,v];
end
function [X,Y,Z] = estimate_pose(image_features,rad,depth)
%intrinsic features
%Focal and pixel dimensions
f = 0.015; pu =10^-5;
%Extract data from image features
centre = image_features(1,:);
%Disparity
%Z = f_*Actual_width/pixel_width;
f_ = f/pu;
Z = depth;
%Output
X = centre(1)*Z/f_;
Y = centre(2)*Z/f_;
%Z = Z - rad; %surface of ball
end
function [J] = toolpoint_Jacobian(Q2_current,Q3_current,Q6_current)
% This function is for:
% Quickly computing the Jacobian generated from the symbolic toolbox
%CONSTANTS
r = 175.73; %mm
%Manipulator Jacobian
J = [ 0, cos(Q6_current)*(sin(Q2_current/r)+(Q3_current*cos(Q2_current/r))/r), sin(Q2_current/r)*cos(Q6_current), sin(Q6_current)*(r*(cos(Q2_current/r)-1)-Q3_current*sin(Q2_current/r))
0, sin(Q6_current)*(sin(Q2_current/r)+(Q3_current*cos(Q2_current/r))/r), sin(Q2_current/r)*sin(Q6_current), -cos(Q6_current)*(r*(cos(Q2_current/r)-1)-Q3_current*sin(Q2_current/r))
1, cos(Q2_current/r)-(Q3_current*sin(Q2_current/r))/r, cos(Q2_current/r), 0
0, (sin(Q6_current)*(tan(Q2_current/r)^2+1))/(r*(tan(Q2_current/r)^2*sin(Q6_current)^2+1)), 0, (tan(Q2_current/r)*cos(Q6_current))/(tan(Q2_current/r)^2*sin(Q6_current)^2+1)];
end