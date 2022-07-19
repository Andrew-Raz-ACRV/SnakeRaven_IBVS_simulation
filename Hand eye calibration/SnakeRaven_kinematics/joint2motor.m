function [mj] = joint2motor(q,design,calibration)
% A function that uses a calibration model to map from SnakeRaven Joints to
% Raven Motor values:
% - This includes tendon calculation for the modules
% - Calibration is a structure with a rate value and an offset value for
%   linear mapping from joint to motor space

%Initialise motor values as q values
mj = q;
da = 10; %mm diameter of actuation on adaptor

%Get the change in tendon lengths
dl0 = GetTendonLengths(design,zeros(length(q),1));
dl1 = GetTendonLengths(design,q);
ddl = dl1 - dl0;

%Tendon lengths:
%mj = 
%disp(rad2deg(ddl/(da/2)));

%Get left tendon 
for ii=1:2*length(design.alpha)
    %Left Tendon is controlled
    mj(3+ii) = ddl(ii,1)/(da/2);
end

%Linear Calibration to Raven Motor space:
for ii=1:length(q)
    mj(ii) = calibration.rate(ii)*mj(ii) + calibration.offset(ii);
end

end