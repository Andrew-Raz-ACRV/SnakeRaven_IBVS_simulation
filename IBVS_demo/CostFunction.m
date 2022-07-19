function [output] = CostFunction(u,s,Lu,Lv,v,Q,R,center,radius,Np,dT)

%Compute prediction horizon:
output = 0;
for ii=1:Np
    s = s + Lu*u(:,ii)*dT; % + Lv*v*dT; % teleop update known or unknown?
    e = circle_region_error(s,center,radius);
    output = output + e'*Q*e + u(:,ii)'*R*u(:,ii);
end
end