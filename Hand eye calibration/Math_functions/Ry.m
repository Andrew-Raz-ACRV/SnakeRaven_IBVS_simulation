function R = Ry(theta)
%Returns the 3 by 3 rotation matrix of theta (radians) about the y axis
% y-axis rotation matrix  = Ry(theta)

R = [cos(theta)  0 sin(theta);
         0       1      0;
     -sin(theta) 0 cos(theta)];
end