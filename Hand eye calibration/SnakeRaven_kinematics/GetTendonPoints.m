function P = GetTendonPoints(T,w,alpha,d,B,pan,k,M)
% T is the transform
% w is the width
% alpha is the angle
% d is the distance between curved surfaces
% B is a logic true for end or false for start
% pan is a logic for pan or tilt
% col is a colour scheme vector for the tendons
% k is the current segment/module e.g. 1 or 2
% M is the total number of segments in system e.g. 2

%Get Point A from Pan
rad = w/2;
r = rad / sin(alpha);

%Initialise points for tendons:
% p = [1pl; 1pr; 1tl; 1tr;   2pl... ];

p = zeros(4*M,3); %Four tendons per segment x,y,z columns
%Use radius w/2 and the angle for tendon placement theta to find x,y,z

%Tendon Placement Angles:
%Tendons:   pl pr tl tr
angles = [-pi/2 pi/2 0 pi];

if pan==1
    if B==0 %A beginning rolling joint frame
        for ii = k:M
            %Go around the circle and get points for each of the 4 tendons in each
            %segment
            for jj = 1:4
                theta = angles(jj) + (ii-k)*deg2rad(-90/M);
                p((ii-1)*4+jj,:) = [rad*cos(theta), rad*sin(theta), sqrt(r^2 - (rad*sin(theta))^2) - r*cos(alpha)];
            end
         %E.g. For one segment these formulas give:
        %pl = [0,-rad,0];
        %pr = [0,rad,0];
        %tl = [rad,0,h];
        %tr = [-rad,0,h];   where   h = r*(1-cos(alpha));      
        end
    elseif B==1 %B end rolling joint frame
        for ii = k:M
            %Go around the circle and get points for each tendon in each
            %segment
            for jj = 1:4
                theta = angles(jj) + (ii-k)*deg2rad(-90/M);
                p((ii-1)*4+jj,:) = [rad*cos(theta), rad*sin(theta), -sqrt(r^2 - (rad*sin(theta))^2) + r*cos(alpha) - d];
            end
        end        
    end
elseif pan==0
    if B==0 %A
        for ii = k:M
            %Go around the circle and get points for each of the 4 tendons in each
            %segment
            for jj = 1:4
                theta = angles(jj) + (ii-k)*deg2rad(-90/M);
                p((ii-1)*4+jj,:) = [rad*cos(theta), rad*sin(theta), sqrt(r^2 - (rad*cos(theta))^2) - r*cos(alpha)];
            end
        end
    elseif B==1 %B
        for ii = k:M
            %Go around the circle and get points for each tendon in each
            %segment
            for jj = 1:4
                theta = angles(jj) + (ii-k)*deg2rad(-90/M);
                p((ii-1)*4+jj,:) = [rad*cos(theta), rad*sin(theta), -sqrt(r^2 - (rad*cos(theta))^2) + r*cos(alpha) - d];
            end
        end 
    end
end

%Determine null set of tendons that is case k=2 M=2 p(1:4,:) is 0 0 0
%Where proximal tendons have finished in the distal section they are no
%longer used in the calculation and are set to 0 0 0 points:
Null_set = all(p==0,2);

P = TransformPoints(T,p);

%Mask null points as zeros:
P = P.*~Null_set;

end
