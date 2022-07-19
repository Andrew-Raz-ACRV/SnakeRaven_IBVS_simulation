function [image_features,image_Jacobian] = take_picture_perspective(W_T_c,points,fig_frame,desired_image_points)
%Creates a picture extracting visual information using perspective camera model
% Inputs:
% W_T_c: 4x4 Transformation Matrix from World frame to camera frame to take
% picture from
% points: 3D coordinates of visual features
% i.e. [x y z; x y z ...] are in world frame,
% N points = number of rows of points
% fig_frame = Figure number 0 to disable figure 1 or more refers to which
% figure
%
% Output:
% image_features: pixel coordinates [u,v; u,v...]
% image_Jacobian is a 2N x 6 matrix for N features
% But has no depth value in the image jacobian
%
%Split the Jacobian to get partitions:
%Jt = image_Jacobian(:,1:3);
%Jw = image_Jacobian(:,4:6);
%
% Andrew 2021

%% Focal and pixel dimensions Parameters:
%f = 0.015; 
%pu =10*10^-6; pv = pu; %Pixel Size
uo = 0; vo = 0; %Principal point

pu = 0.9/384;
pv = pu;
f = 252.5 * pu;
%uo = 207.3;
%vo = 205.65;


%% Intrinsic Camera Matrix:
%camera matrix
cam_mat = [f 0 0 0;
    0 f 0 0;
    0 0 1 0];
%Image plane K
K = [(1/pu) 0 uo;
    0 (1/pv) vo;
    0 0 1];

%% Convert points to image plane:
N = size(points,1);

%initialise output
image_features = zeros(N,2);
image_Jacobian = zeros(2*N,6);

for ii = 1:N
    %Extract homogenous 3D coordinate point [x y z 1]'
    W_P = [points(ii,:) 1]';
    %Find camera to point translation
    C_P = (W_T_c) \ W_P; %C_P =[x y z 1]' relative to camera
    Z = C_P(3); % true depth
    %Z = 1; 

    %Camera Model to Image plane
    p_ = K*cam_mat*C_P; % p_ = u,v,w]
    %convert out of homogeneous
    p = p_/p_(3);
    u = p(1) - uo;
    v = p(2) - vo;
    %image_features = [image_features; u, v];
    image_features(ii,:) = [u,v];
    %Compute the image jacobian for that point:
    image_Jacobian(((2*ii)-1):(2*ii),:) = ...
    [-f/(pu*Z) 0 u/Z (pu*u*v)/f -(f^2 + pu^2 * u^2)/(pu*f) v;
        0 -f/(pv*Z) v/Z (f^2 + pv^2 * v^2)/(pv*f) -(pv*u*v)/f -u];
end

%% Plot Image:
if fig_frame>0
    figure(fig_frame);
    grid on; axis equal; hold on
    plot(desired_image_points(:,1),desired_image_points(:,2),'b-')
    hold on
    plot(image_features(:,1),image_features(:,2),'ro')
    title('Image Plane')
    xlabel('u')
    ylabel('v')
    legend('Desired','Actual','Location','southoutside')
end

end