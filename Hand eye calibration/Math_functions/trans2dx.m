function dx = trans2dx(T0,T1)
% Computes the change position and angles dx = [dtx,dty,dtz,drx,dry,drz];
% Given two pose transforms moving from T0 to T1
% refer to equation 3.10 on page 53 Peter Corke textbook

t1 = T1(1:3,4);
t0 = T0(1:3,4);

R1 = T1(1:3,1:3);
R0 = T0(1:3,1:3);

dx = [t1-t0; vex(R1*R0' - [1 0 0; 0 1 0; 0 0 1])];

end