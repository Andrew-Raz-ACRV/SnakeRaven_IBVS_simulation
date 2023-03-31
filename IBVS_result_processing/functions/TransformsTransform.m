function Tb = TransformsTransform(A,Tab)
% Given a single Transform Tab (4x4) (from frame A to B)
% transform it to an array of transforms A (4x4xN) (transform from 0 to A)
% Giving you Transforms Ta (4x4) (from 0 to B)
% i.e. Tb = A*Tab
% i.e. 0_T_B(i=1:N) = 0_T_A(i=1:N) * A_T_B
% Not to be confused with TransformTransforms!
%
% % author: Andrew Razjigaev 2020

N = size(A,3);
Tb = zeros(size(A));

for ii = 1:N
    Tb(:,:,ii) = A(:,:,ii)*Tab;
end

end