function R = Rz(theta)
%Returns the 3 by 3 rotation matrix of theta (radians) about the z axis
% z-axis rotation matrix  = Ry(theta)

R = [cos(theta) -sin(theta) 0;
     sin(theta)  cos(theta) 0;
        0           0       1];
    
end