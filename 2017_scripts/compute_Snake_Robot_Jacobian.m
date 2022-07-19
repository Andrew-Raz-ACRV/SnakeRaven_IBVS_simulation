function compute_Snake_Robot_Jacobian()
% This Function symbolically solves the Forward Kinematics equations,
% rotation matrix and the resulting Jacobian matrix for Position Control
% This outputs nothing but a display of these equations.
%
% Assuming the straight curved straight configuration of concentric tubes
% The Jacobian is a 3 x 4 matrix relating x,y,z to q1,q2,q3,q5 joints where
% q1, q2, q3 are prismatic actuation of the tubes (q2 is curved) and q5 is
% the rotation of the curved tube
%
% Written by Andrew Razjigaev 01/06/2018

clc
%symbols for joint valuas and radius of curvature
syms q1; syms q2; syms q3; syms q4; syms q5; syms q6; syms r;

%Solve and test the Forward Kinematics:
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
                    
% Display the forward kinematics equations
Base_T_tool = Base_T_q1*q1_T_q5*q5_T_q2*q2_T_q3*q3_T_q6;

X = Base_T_tool(1,4); Y = Base_T_tool(2,4); Z = Base_T_tool(3,4);

Rotation_matrix = Base_T_tool(1:3,1:3);

display(X), display(Y), display(Z);
display(Rotation_matrix);

%BUILD THE POSITION JACOBIAN:
Q = [q1; q2; q3; q5]; %xyz only depends on these three
%X = f(Q);
F1 = f1(Q); F2 = f2(Q); F3 = f3(Q);
F = [F1; F2; F3];

disp('Jacobian symbolically is:');
J = jacobian(F,Q);
display(J); %Display symbolic Jacobian
end

function [X] = f1(Q)
syms r;
q1 = Q(1); q2 = Q(2); q3 = Q(3); q5 = Q(4);
X = r*sin(q5)*(cos(q2/r) - 1) - q3*sin(q2/r)*sin(q5);
end

function [Y] = f2(Q)
syms r;
q1 = Q(1); q2 = Q(2); q3 = Q(3); q5 = Q(4);
Y = q3*sin(q2/r)*cos(q5) - r*cos(q5)*(cos(q2/r) - 1);
end

function [Z] = f3(Q)
syms r;
q1 = Q(1); q2 = Q(2); q3 = Q(3); q5 = Q(4);
Z = q1 + q3*cos(q2/r) + r*sin(q2/r);
end