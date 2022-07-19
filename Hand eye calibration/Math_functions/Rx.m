function R = Rx(theta)
%Returns the 3 by 3 rotation matrix of theta (radians) about the x axis
% x-axis rotation matrix  = Rx(theta)

R = [1      0           0;
     0 cos(theta) -sin(theta);
     0 sin(theta) cos(theta)];

end