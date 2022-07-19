function v = vex(S)
% Convert skew symmetric matrix S back into a vector w
% assuming for 3D space
% V (1x3) then S =
%
%           |  0  -vz   vy |
%           | vz    0  -vx |
%           |-vy   vx    0 |
%
% returns V = [vx vy vz]'

v = [S(3,2) S(1,3) S(2,1)]';

end