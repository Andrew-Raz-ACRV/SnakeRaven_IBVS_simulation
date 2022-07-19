function [Base_T_tool] = plot_snake_robot(q1,q2,q3,q4,q5,q6,target)
%Plots a Snake robot given the joint values.
% [Base_T_tool] = plot_snake_robot(q1,q2,q3,q4,q5,q6)
%It plots the endeffector coordinate (x is red, y is green and z is blue)
%as well as returns the
%forward kinematics of the robot (The transform from base to endeffector)
%
%Assumptions: its a straight-curved-straight concentric tube robot
%joint values are in mm for q1,q2,q3 the outer to inner tubes,
%joint values are in radians for q4,q5,q6 the rotation joints for the
%outer to inner tubes respectively.
%written by Andrew Razjigaev 31/05/2018

%CONSTANTS
r = 175.73; %mm

%Forward Kinematics Transforms:
% q1 is a prismatic joint acting in the z-axis
Base_T_q1 = [1 0 0 0;
             0 1 0 0;
             0 0 1 q1;
             0 0 0 1];
% q5 is a revolute joint acting about the z-axis
q1_T_q5 = [cos(q5) -sin(q5) 0 0;
            sin(q5) cos(q5) 0 0;
                        0 0 1 0;
                        0 0 0 1];
% q2 is a curved tube(induces rotation about the X-axis and translation in
% the y and z-axes)
q5_T_q2 = [ 1 0 0 0;
            0 cos(q2/r) sin(q2/r) r*(1-cos(q2/r));
            0 -sin(q2/r) cos(q2/r) r*sin(q2/r);
            0 0 0 1];
% q3 is the prismatic joint of the third tube
q2_T_q3 = [1 0 0 0;
           0 1 0 0;
           0 0 1 q3;
           0 0 0 1];
% q6 is a revolute joint of the tool
q3_T_q6 = [cos(q6) -sin(q6) 0 0;
            sin(q6) cos(q6) 0 0;
                        0 0 1 0;
                        0 0 0 1];

                    
%PLOT BASE FRAME:
plotcoord3(eye(4),20,'r-','g-','b-')
hold on

%PLOT TUBE 1 LINE
pb = [0 0 0 1]';
p1 = Base_T_q1*pb;
x = [pb(1) p1(1)];
y = [pb(2) p1(2)];
z = [pb(3) p1(3)];
plot3(x,y,z,'k-','LineWidth',6);

%PLOT TUBE 2 CURVE
pii2 = p1;
for ii = linspace(0,q2,100)
q5_2_q2 = [1 0 0 0;
0 cos(ii/r) -sin(ii/r) r*(1-cos(ii/r));
0 sin(ii/r) cos(ii/r) r*sin(ii/r);
0 0 0 1];
p2 = Base_T_q1*q1_T_q5*q5_2_q2*pb;
hold on
x = [pii2(1) p2(1)];
y = [pii2(2) p2(2)];
z = [pii2(3) p2(3)];
plot3(x,y,z,'k-','LineWidth',4);
pii2 = p2;
end
p2 = Base_T_q1*q1_T_q5*q5_T_q2*pb;


%PLOT TUBE 3 LINE
Base_T_tool = Base_T_q1*q1_T_q5*q5_T_q2*q2_T_q3*q3_T_q6;
p3 = Base_T_tool*pb;

hold on
x = [p2(1) p3(1)]; y = [p2(2) p3(2)]; z = [p2(3) p3(3)];
plot3(x,y,z,'k-','LineWidth',2);
hold on
plotcoord3(Base_T_tool,20,'r-','g-','b-')

%PLOT the target point
hold on
[xs,ys,zs] = sphere;
rad = 2;
surf(xs*rad+target(1), ys*rad+target(2), zs*rad+target(3));


title('3D View of Snake Robot'), xlabel('X displacement')
ylabel('Y displacement'), zlabel('Z displacement')
view(3)
daspect([1 1 1])
grid on
hold off
end

function plotcoord3(transform,axis_length,X_colour,Y_colour,Z_colour)
%Plots a 3D coordinate frame given a transform, the axis length and colours
%for each axis
%e.g. 
% R = eye(3); t = [0 0 0]';
%plotcoord3([R, t; 0 0 0 1],1,'r','g','b')
%
%this plots a coordinate frame at the origin with a red x-axis, green
%y-axis and a blue z-axis
%
%Tips: other useful functions related to this after plotting...
%
% title('3D View (X-Y-Z)'), xlabel('X displacement')
% ylabel('Y displacement'), zlabel('Z displacement')
% view(3)
% daspect([1 1 1]) % makes axes equal in 3D
% grid on
%
%written by Andrew Razjigaev

    %Axis Lengths of the axes on the robot coordinate frame
    px = [axis_length; 0; 0; 1]; py = [0; axis_length; 0; 1]; pz = [0; 0; axis_length; 1];
    %Homogeneous transform of the axes
    x = transform(1,4); y = transform(2,4); z = transform(3,4);
    %Computed points of axes:
    px_point = transform*px;
    py_point = transform*py;
    pz_point = transform*pz;
    %disp(transform)
    
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