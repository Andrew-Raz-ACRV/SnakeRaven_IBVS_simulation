function plotInsertionTube(Baseframe,w,alpha,L,pan_first)
% Plots the insertion tube base for of a variable neutral line manipulator 
% at the base frame of the snake robot
%Parameters are:
% w = the diameter of the simulated tube
% alpha = the contact angle of the curved surface on the top of the tube
% L = the constant height within the disk between each curved surface

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
z = r*sin(arcTheta)-r*cos(alpha); ztop = [z 0];

%plot arc top surface with patches
for ii = 1:length(arcTheta)
    X = [x(ii) x(ii) x(ii+1) x(ii+1)];
    y1 = real(sqrt((rad^2)-(x(ii))^2));
    y2 = real(sqrt((rad^2)-(x(ii+1))^2));
    Y = [-y1 y1 y2 -y2];
    Z = [ztop(ii) ztop(ii) ztop(ii+1) ztop(ii+1)];   
    %Transfrom coordinates of the patch
    if pan_first==true
        P = TransformPoints(Baseframe*[Rz(pi/2) [0 0 0]'; 0 0 0 1],[X', Y', Z']);
    else
        %tilt first
        P = TransformPoints(Baseframe,[X', Y', Z']);
    end
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

% for ii = 1:(length(theta)-1)
%     X = [x(ii) x(ii+1) x(ii+1) x(ii)];
%     Y = [y(ii) y(ii+1) y(ii+1) y(ii)];
%     Z = [-L -L 0 0];
%     %Transfrom coordinates of the patch
%     P = TransformPoints(Baseframe,[X', Y', Z']);
%     X = P(:,1); Y = P(:,2); Z = P(:,3);
%     patch(X,Y,Z,C,         'EdgeColor',       'none',        ...
%          'FaceLighting',    'gouraud', ...
%          'DiffuseStrength', 1,...
%          'BackFaceLighting',    'reverselit', ...
%          'AmbientStrength', 0.2);
%     hold on
% end

%Plot The section under the curve with the cylinder

for ii = 1:(length(theta)-1)
    X = [x(ii) x(ii+1) x(ii+1) x(ii)];
    Y = [y(ii) y(ii+1) y(ii+1) y(ii)];    

    %Get Z height points of the arc based on function of arc:
    z1 = sqrt(r^2 - y(ii)^2) - r*cos(alpha);
    z2 = sqrt(r^2 - y(ii+1)^2) - r*cos(alpha);
    
    %Z is height from -L to the curve of the arc
    Z = [-L -L z2 z1];
    
    %Transfrom coordinates of the patch
    P = TransformPoints(Baseframe,[X', Y', Z']);
    X = P(:,1); Y = P(:,2); Z = P(:,3);
    patch(X,Y,Z,C,         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud', ...
         'DiffuseStrength', 1,...
         'BackFaceLighting',    'reverselit', ...
         'AmbientStrength', 0.2);
    hold on
end

end