function Tb = TransformTransforms(A,Tab)
% Given Transforms Tb (4x4xN) (from frame A to B)
% transform it to transform A (4x4) (transform from 0 to A)
% Giving you Transforms Ta (4x4xN) (from 0 to B)
% i.e. Tb = A*Tab
% i.e. 0_T_B(i=1:N) = 0_T_A * A_T_B(i=1:N) 
% 
% % author: Andrew Razjigaev 2020

N = size(Tab,3);
Tb = zeros(size(Tab));

for ii = 1:N
    Tb(:,:,ii) = A*Tab(:,:,ii);
end

end
