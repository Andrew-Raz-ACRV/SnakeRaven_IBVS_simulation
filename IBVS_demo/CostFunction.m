function [output] = CostFunction(u,s,Lu,Lv,v,Q,R,center,radius,Np,dT,translation_feed)

%Compute prediction horizon:
output = 0;
for ii=1:Np
    % teleop update known or unknown?
    if(translation_feed)
        s = s + Lu*u(:,ii)*dT + Lv*v*dT;
    else
        s = s + Lu*u(:,ii)*dT;
    end
    e = circle_region_error(s,center,radius);
    output = output + e'*Q*e + u(:,ii)'*R*u(:,ii);
end
end