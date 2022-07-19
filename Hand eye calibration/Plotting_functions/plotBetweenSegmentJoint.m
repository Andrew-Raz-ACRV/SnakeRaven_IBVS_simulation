function plotBetweenSegmentJoint(T,w,sigma,zeta,alpha1,alpha2)
% Plots the joint for a variable neutral line manipulator at coordinate frame
% T{1'} where the bottom end is the proximal segment and the upper end is
% the distal segment
%Parameters are:
% T is the transfrom to draw it at the tip of the lower segment
% w = the width of the disk that is the diameter of the simulated one
% sigma = the constant height offset between segments
% zeta = the z-axis rotation from one segment to the next. This is
% necessary so that no cable overlaps
% alpha1 = half the contact angle for Segment 1 (lower)
% alpha2 = half the contact angle for Segment 2 (upper)
% n_proximal = the number of disks in the lower segment

%DATA FOR PLOTTING

%The radius of the tube:
rad = w/2;
%radius of contact surface depends on w and alpha %alpha = asin(rad/r);
r1 = rad / sin(alpha1);
r2 = rad / sin(alpha2);

%default colour
C = [0.6 0.6 0.6];
%points for patch generation
pts = 100;

% Curved Top and Bottom Surfaces

%Plot the top arc contact part
arcTheta = linspace(-alpha2+pi/2,alpha2+pi/2,pts);
x = r2*cos(arcTheta); x = [x -rad]; 
z = r2*sin(arcTheta)+(sigma-r2*cos(alpha2)); ztop = [z sigma];

%plot arc top surface with patches
for ii = 1:length(arcTheta)
    X = [x(ii) x(ii) x(ii+1) x(ii+1)];
    y1 = real(sqrt((rad^2)-(x(ii))^2));
    y2 = real(sqrt((rad^2)-(x(ii+1))^2));
    Y = [-y1 y1 y2 -y2];
    Z = [ztop(ii) ztop(ii) ztop(ii+1) ztop(ii+1)];   
    %Transfrom coordinates of the patch
    P = TransformPoints(T*txyz(0,0,-sigma),[X', Y', Z']);
    X = P(:,1); Y = P(:,2); Z = P(:,3);
    patch(X,Y,Z,C,         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud', ...
         'DiffuseStrength', 1,...
         'BackFaceLighting',    'reverselit', ...
         'AmbientStrength', 0.2);
    hold on
end

%Plot bottom arc contact part
arcTheta = linspace(-alpha1-pi/2,alpha1-pi/2,pts);
y = r1*cos(arcTheta); y = [y rad];
z = r1*sin(arcTheta)+(r1*cos(alpha1)); zbot = [z 0];

%plot arc bottom surface
for ii = 1:length(arcTheta)
    Y = [y(ii) y(ii) y(ii+1) y(ii+1)];  
    x1 = real(sqrt((rad^2)-(y(ii))^2));
    x2 = real(sqrt((rad^2)-(y(ii+1))^2));
    X = [-x1 x1 x2 -x2];
    Z = [zbot(ii) zbot(ii) zbot(ii+1) zbot(ii+1)];
    %Transfrom coordinates of the patch
    P = TransformPoints(T*[Rz(-zeta) [0 0 0]'; 0 0 0 1]*txyz(0,0,-sigma),[X', Y', Z']);
    X = P(:,1); Y = P(:,2); Z = P(:,3);
    patch(X,Y,Z,C,         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud', ...
         'DiffuseStrength', 1,...
         'BackFaceLighting',    'reverselit', ...
         'AmbientStrength', 0.2);
    hold on
end

%plot cylinder:
theta = linspace(-2*pi,2*pi,pts);
x = rad*cos(theta);
y = rad*sin(theta);

%Make relative to

for ii = 1:(length(theta)-1)
    X1 = [x(ii) x(ii+1) x(ii+1) x(ii)];
    Y1 = [y(ii) y(ii+1) y(ii+1) y(ii)];
    
    %Section under the Top Curve
    %Get Z height points of the arc based on function of arc:
    z1 = sqrt(r2^2 - x(ii)^2) - r2*cos(alpha2);
    z2 = sqrt(r2^2 - x(ii+1)^2) - r2*cos(alpha2);
    
    %Add section under curve to d
    Z1 = [0 0 sigma+z2 sigma+z1];
    %Z = [0 0 sigma sigma];
    
    %Transfrom coordinates of the patch
    P = TransformPoints(T*txyz(0,0,-sigma),[X1', Y1', Z1']);
    X = P(:,1); Y = P(:,2); Z = P(:,3);
    patch(X,Y,Z,C,         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud', ...
         'DiffuseStrength', 1,...
         'BackFaceLighting',    'reverselit', ...
         'AmbientStrength', 0.2);
    hold on
    
    %Section under the Bottom Curve
    %Get Z bottom points of the arc based on function of arc:
    z3 = sqrt(r1^2 - y(ii)^2) - r1*cos(alpha1);
    z4 = sqrt(r1^2 - y(ii+1)^2) - r1*cos(alpha1);  
    
    %Add section under curve to d
    Z2 = [-z3 -z4 0 0];  
    
    %Transfrom coordinates of the patch
    P = TransformPoints(T*[Rz(-zeta) [0 0 0]'; 0 0 0 1]*txyz(0,0,-sigma),[X1', Y1', Z2']);
    X2 = P(:,1); Y2 = P(:,2); Z2 = P(:,3);
    patch(X2,Y2,Z2,C,         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud', ...
         'DiffuseStrength', 1,...
         'BackFaceLighting',    'reverselit', ...
         'AmbientStrength', 0.2);
    hold on
    
end

%Finished plotting
end
