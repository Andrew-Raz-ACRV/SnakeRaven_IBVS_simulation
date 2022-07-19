function compute_camera_Jacobian()
clc
syms q1; syms q2; syms q6; syms r; syms b; syms e;
%Solve and test the Forward Kinematics:
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
% Display the forward kinematics equations
X = Base_2_Camera(1,4); Y = Base_2_Camera(2,4); Z = Base_2_Camera(3,4);
Rotation_matrix = Base_2_Camera(1:3,1:3);
display(X), display(Y), display(Z);
display(Rotation_matrix);
%BUILD THE JACOBIAN:
Q = [q1; q2; q6];
%X = f(Q);
F1 = f1(Q); F2 = f2(Q); F3 = f3(Q); F4 = f4(Q); F5 = f5(Q); F6 = f6(Q);
F = [F1; F2; F3; F4; F5; F6];
display('Jacobian symbolically is:');
J = jacobian(F,Q);
display(J); %Display symbolic Jacobian
end
function [X] = f1(Q)
syms r; syms b; syms e;
q1 = Q(1); q2 = Q(2); q6 = Q(3);
X = (r-(b-r)*cos(q2/r))*cos(q6);
end
function [Y] = f2(Q)
syms r; syms b; syms e;
q1 = Q(1); q2 = Q(2); q6 = Q(3);
Y = (r-(b-r)*cos(q2/r))*sin(q6);
end
function [Z] = f3(Q)
syms r; syms b; syms e;
q1 = Q(1); q2 = Q(2); q6 = Q(3);
Z = (r-b)*sin(q2/r) + q1;
end
function [Rx] = f4(Q)
syms r; syms b; syms e;
q1 = Q(1); q2 = Q(2); q6 = Q(3);
Rx = atan(sin(q6)*tan(q2/r));
end
function [Ry] = f5(Q)
syms r; syms b; syms e;
q1 = Q(1); q2 = Q(2); q6 = Q(3);
Ry = atan(cos(q6)*tan(q2/r));
end
function [Rz] = f6(Q)
syms r; syms b; syms e;
q1 = Q(1); q2 = Q(2); q6 = Q(3);
Rz = q6 + e;
end
