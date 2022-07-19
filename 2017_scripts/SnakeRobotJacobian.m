function [J] = SnakeRobotJacobian(q2,q3,q5,r)
% This function outputs the 3 x 4 Jacobian matrix for position control of
% a concentric tube robot in a striaght curved straight configuration
% The Jacobian only needs q2, q3, the prismatic joints for the curved nd
% endeffector tube and q5 the rotation of the curved tube.
%
% Written by Andrew Razjigaev 01/06/2018

J = [ 0,  -sin(q2/r)*sin(q5)-(q3*cos(q2/r)*sin(q5))/r,  -sin(q2/r)*sin(q5), r*cos(q5)*(cos(q2/r)-1)-q3*sin(q2/r)*cos(q5);
      0,   sin(q2/r)*cos(q5)+(q3*cos(q2/r)*cos(q5))/r,   sin(q2/r)*cos(q5), r*sin(q5)*(cos(q2/r)-1)-q3*sin(q2/r)*sin(q5);
      1,                   cos(q2/r)-(q3*sin(q2/r))/r,           cos(q2/r),                                           0];
 


end