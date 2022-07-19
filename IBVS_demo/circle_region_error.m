function [error] = circle_region_error(p,center,radius)
% p is points [u v u v ..]'
% center i s point [x y]
% radius is the distance of edge of circle to center

%by default error is 0
error = zeros(size(p));

pts = length(p)/2;
for ii = 1:pts
    x = p(2*ii - 1); y = p(2*ii);
    R = sqrt((x - center(1))^2 + (y - center(2))^2);
    %If outside the circle:
    if R>radius
        %Compute error
        magnitude = R - radius;
        deltaX = -magnitude*(x - center(1))/R;
        deltaY = -magnitude*(y - center(2))/R;
        
        %Insert:
        error(2*ii - 1) = deltaX;
        error(2*ii) = deltaY;
    end
end
end