function TR = Tnorm(T)
%Tnorm normalises transform T so that the rotation matrix is a true one
%   Outputs another 4x4 transform

    o = T(1:3,2); a = T(1:3,3);
    n = cross(o, a);         % N = O x A
    o = cross(a, n);         % O = A x N
    R = [cap_mag(n,1) cap_mag(o,1) cap_mag(a,1)];

    TR = [R, T(1:3,4); 0 0 0 1];
end

