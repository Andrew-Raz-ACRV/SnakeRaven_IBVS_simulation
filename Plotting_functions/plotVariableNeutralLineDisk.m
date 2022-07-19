function plotVariableNeutralLineDisk(T,w,alpha,d)
% Plots the disk of a variable neutral line manipulator at coordinate frame
% T{1'} based on the paper "Modelling and control of Robotic Surgical
% Platform...
%Parameters are:
% w = the width of the disk that is the diameter of the simulated one
% alpha = half the contact angle of the curved surface on the disk
% d = the constant height within the disk between each curved surface

%DATA FOR PLOTTING

%The radius of the tube:
rad = w/2;
%radius of contact surface depends on w and alpha %alpha = asin(rad/r);
r = rad / sin(alpha);

%default colour
C = [0.6 0.6 0.6];
%points for patch generation
pts = 100;



% Curved Top and Bottom Surfaces

%Plot the top arc contact part
arcTheta = linspace(-alpha+pi/2,alpha+pi/2,pts);
x = r*cos(arcTheta); x = [x -rad]; 
z = r*sin(arcTheta)+(d-r*cos(alpha)); ztop = [z d];

%plot arc top surface with patches
for ii = 1:length(arcTheta)
    X = [x(ii) x(ii) x(ii+1) x(ii+1)];
    y1 = real(sqrt((rad^2)-(x(ii))^2));
    y2 = real(sqrt((rad^2)-(x(ii+1))^2));
    Y = [-y1 y1 y2 -y2];
    Z = [ztop(ii) ztop(ii) ztop(ii+1) ztop(ii+1)];   
    %Transfrom coordinates of the patch
    P = TransformPoints(T*txyz(0,0,-d),[X', Y', Z']);
    X = P(:,1); Y = P(:,2); Z = P(:,3);
    patch(X,Y,Z,C,         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud', ...
         'DiffuseStrength', 1,...
         'BackFaceLighting',    'reverselit', ...
         'AmbientStrength', 0.2);
    hold on
end

%Plot bottom arc contact part
arcTheta = linspace(-alpha-pi/2,alpha-pi/2,pts);
y = r*cos(arcTheta); y = [y rad];
z = r*sin(arcTheta)+(r*cos(alpha)); zbot = [z 0];

%plot arc bottom surface
for ii = 1:length(arcTheta)
    Y = [y(ii) y(ii) y(ii+1) y(ii+1)];  
    x1 = real(sqrt((rad^2)-(y(ii))^2));
    x2 = real(sqrt((rad^2)-(y(ii+1))^2));
    X = [-x1 x1 x2 -x2];
    Z = [zbot(ii) zbot(ii) zbot(ii+1) zbot(ii+1)];
    %Transfrom coordinates of the patch
    P = TransformPoints(T*txyz(0,0,-d),[X', Y', Z']);
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
    X = [x(ii) x(ii+1) x(ii+1) x(ii)];
    Y = [y(ii) y(ii+1) y(ii+1) y(ii)];
    %Section under the Curve
    
    %Get Z height points of the arc based on function of arc:
    z1 = sqrt(r^2 - x(ii)^2) - r*cos(alpha);
    z2 = sqrt(r^2 - x(ii+1)^2) - r*cos(alpha);
    
    %Get Z bottom points of the arc based on function of arc:
    z3 = sqrt(r^2 - y(ii)^2) - r*cos(alpha);
    z4 = sqrt(r^2 - y(ii+1)^2) - r*cos(alpha);
    
    %Add section under curve to d
    Z = [0-z3 0-z4 d+z2 d+z1];
    
    %Z = [0 0 d d];
    
    %Transfrom coordinates of the patch
    P = TransformPoints(T*txyz(0,0,-d),[X', Y', Z']);
    X = P(:,1); Y = P(:,2); Z = P(:,3);
    patch(X,Y,Z,C,         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud', ...
         'DiffuseStrength', 1,...
         'BackFaceLighting',    'reverselit', ...
         'AmbientStrength', 0.2);
    hold on
end

%Finished plotting
end




%{
%DATA FOR PLOTTING
%Extract the origin:
x0 = T(1,4); y0 = T(1,4); z0 = T(3,4);
%Extract orientation
R = T(1:3,1:3);

%The radius of the tube:
rad = w/2;
%radius of contact surface depends on w and alpha %alpha = asin(rad/r);
r = rad / sin(alpha);

%default colour
C = [0.5 0.5 0.5];
%points for patch generation
pts = 60;



% Curved Top and Bottom Surfaces

%Plot the top arc contact part
arcTheta = linspace(-alpha+pi/2,alpha+pi/2,pts);
x = r*cos(arcTheta)+x0; x = [x -rad]; 
z = r*sin(arcTheta)+(z0+d-r*cos(alpha)); ztop = [z z0+d];

%plot arc top surface with patches
for ii = 1:length(arcTheta)
    X = [x(ii) x(ii) x(ii+1) x(ii+1)];
    y1 = real(sqrt((rad^2)-(x(ii)-x0)^2));
    y2 = real(sqrt((rad^2)-(x(ii+1)-x0)^2));
    Y = [-y1 y1 y2 -y2];
    Z = [ztop(ii) ztop(ii) ztop(ii+1) ztop(ii+1)];    
    patch(X,Y,Z,C);
    hold on
end

%Plot bottom arc contact part
arcTheta = linspace(-alpha-pi/2,alpha-pi/2,pts);
y = r*cos(arcTheta)+y0; y = [y rad];
z = r*sin(arcTheta)+(z0+r*cos(alpha)); zbot = [z z0];

%plot arc bottom surface
for ii = 1:length(arcTheta)
    Y = [y(ii) y(ii) y(ii+1) y(ii+1)];  
    x1 = real(sqrt((rad^2)-(y(ii)-y0)^2));
    x2 = real(sqrt((rad^2)-(y(ii+1)-y0)^2));
    X = [-x1 x1 x2 -x2];
    Z = [zbot(ii) zbot(ii) zbot(ii+1) zbot(ii+1)];
    patch(X,Y,Z,C);
    hold on
end

%plot cylinder:
theta = linspace(-2*pi,2*pi,pts);
x = rad*cos(theta)+x0;
y = rad*sin(theta)+y0;
zd = z0+d;

for ii = 1:(length(theta)-1)
    X = [x(ii) x(ii+1) x(ii+1) x(ii)];
    Y = [y(ii) y(ii+1) y(ii+1) y(ii)];
    Z = [z0 z0 zd zd];
    patch(X,Y,Z,C);
    hold on
end

%}