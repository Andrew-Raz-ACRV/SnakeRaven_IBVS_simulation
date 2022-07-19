function P = TransformPoints(T,p)
% Makes column vectors of (x,y,z) coordinates become those vectors in 
% reference to coordinate frame T 
% take p as a matrix p = [x, y, z;    
%                        x2, y2, z2; 
%                               ...];
% and do the transform P = T*p

n = size(p,1);
P = zeros(size(p));

for ii = 1:n
    Pv = T*[p(ii,1) p(ii,2) p(ii,3) 1]';
    
    P(ii,1) = Pv(1);
    P(ii,2) = Pv(2);
    P(ii,3) = Pv(3);
    
end

end
