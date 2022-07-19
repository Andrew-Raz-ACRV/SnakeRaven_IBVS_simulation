function dT = dx2trans(dx)
% computes a commutable dT transform matrix given an infinitesimally small
% update where dx = [dtx,dty,dtz,drx,dry,drz];
%
% For example: We have T0 and T1 are frames relative to w
% if we use trans2dx(T0,T1) we get an update step called dx
%
% To solve T1 given a motion dx and start frame T0 we do this:
% T1 = dx2trans(dx) * T0
%
% Computes equation 3.12 on page 53 Peter corke textbook delta2tr
    dx = dx(:);
    dT = eye(4,4) + [skew_symmetric(dx(4:6)) dx(1:3); 0 0 0 0];