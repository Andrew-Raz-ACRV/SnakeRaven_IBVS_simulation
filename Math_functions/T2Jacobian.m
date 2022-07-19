function B_J_A = T2Jacobian(A_T_B)
% Creates the Jacobian matrix to transform a velocity vector from frame A
% to frame B given transform matrix 4x4
% i.e. B_v = B_J_A * A_v
% 
% Input: A_T_B 4x4 Transform matrix
% Output: A_J_B 6x6 Jacobian matrix

%Get Rotation and translation
B_R_A = A_T_B(1:3,1:3)';
A_t_B = A_T_B(1:3,4);


B_J_A = [B_R_A      -B_R_A*skew_symmetric(A_t_B); 
         zeros(3,3)             B_R_A];
end