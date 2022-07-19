function [q_,hit] = applyJointLimits(q,qL,qU)
%Given the lower and upper joint limits in qL and qU respectively, this
%restricts the values of q to be in that range i.e. qL(i) < q(i) < qU(i)
% hit is a boolean that says if the collision occured or not

%by default
q_ = q; hit = false;

%Check limits in each value q
for ii = 1:length(q)
    if q(ii)<qL(ii)
        q_(ii)=qL(ii);
        hit = true;
    elseif q(ii)>qU(ii)
        q_(ii)=qU(ii);
        hit = true;
    end
end

end