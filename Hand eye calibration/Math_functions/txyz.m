function T = txyz(x,y,z)
%Gives Transformation matrix for an x, y, z translation
% T = txyz(x,y,z)

T = [eye(3) [x y z]';
     zeros(1,3) 1];
end