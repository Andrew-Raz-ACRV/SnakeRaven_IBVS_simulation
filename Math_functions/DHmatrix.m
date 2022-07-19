function T = DHmatrix(alpha,a,d,theta)
% Computes a DH matrix 
% 
% input: 
% alpha = link angle from frame N-1
% a = link length from frame N-1;
% d = joint distance to frame N 
% theta = joint angle to frame N
%
% ouput: T = [4x4] matrix from frame N-1 to N

%Terms for DH matrix
c0 = cos(theta);
s0 = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

%DH matrix
T = [c0    -s0   0   a;
     s0*ca c0*ca -sa -d*sa;
     s0*sa c0*sa  ca d*ca;
     0     0     0   1];

end