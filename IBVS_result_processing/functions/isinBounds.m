function [logic] = isinBounds(point,xbounds,ybounds,zbounds)
%Checks if point = x,y,z is within bounds [x_min x_max] etc...
%in each dimension. It also works for multiple bounded areas
%i.e. xbounds = [x1_min x1_max; x2_min x2_max...]

%Number of bounded areas
N = size(xbounds,1);
logic = false; %By default

%For each bounded area check if the point is in it:
for ii = 1:N
    if (point(1)<xbounds(ii,2))&&(point(1)>xbounds(ii,1))...
      &&(point(2)<ybounds(ii,2))&&(point(2)>ybounds(ii,1))...
      &&(point(3)<zbounds(ii,2))&&(point(3)>zbounds(ii,1))
        %If any point is within the bounds its all true
        logic = true;
    end
end

end