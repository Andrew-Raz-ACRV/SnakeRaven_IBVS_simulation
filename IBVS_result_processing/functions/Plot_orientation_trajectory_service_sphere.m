function Plot_orientation_trajectory_service_sphere(R_ndi,R_cam,R_tar)
% Given rotation matrices over time i.e. R(:,:,i) ith sample
% Plot orientation as a point on a service sphere:

%% Parameters
%service sphere params
%ss_params = [18,9,0.349065850398866,0.222222222222222];
ss_params = [50,25,0.349065850398866,0.222222222222222];

%Get coordinates in on sphere surface from ss_params
Ntheta = ss_params(1); Nh = ss_params(2); 
%dtheta = ss_params(3);  dh = ss_params(4);

%% Plot Empty Sphere:
%CREATE FIGURE AND PLOT SPHERE:

%Create blank white sphere
theta = linspace(-pi,pi,Ntheta+1);
h = linspace(-1,1,Nh+1);
phi = asin(h); %%angle phi
[theta,phi] = meshgrid(theta,phi);
[xs,ys,zs] = sph2cart(theta,phi,1);


%% Get trajectory point and plot
r = -1.0001; %radius arm of sphere
p = [0 0 r]';

%NDI points p1
Nndi = length(R_ndi);
p1 = zeros(Nndi,3);
for ii=1:Nndi
    %R_ndi,R_cam_R_tar    
    p1_ = R_ndi(:,:,ii)*p;   
    %p1_ = Rz(-pi/2)*p1_;
    p1(ii,:) = p1_';   
end

%Cam pose points:
Ncam = length(R_cam);
p2 = zeros(Ncam,3);
for ii=1:Ncam
    %R_ndi,R_cam_R_tar    
    p2_ = R_cam(:,:,ii)*p;
    %p2_ = Rz(-pi/2)*p2_;
    p2(ii,:) = p2_';   
end

% %Target Camera points
% Ntar = length(R_tar);
% p3 = zeros(Ntar,3);
% for ii=1:Ntar
%     %R_ndi,R_cam_R_tar    
%     p3_ = R_tar(:,:,ii)*p;   
%     p3(ii,:) = p3_';   
% end

%% Plot

p2 = [medfilt1(p2(:,1),200) medfilt1(p2(:,2),200) medfilt1(p2(:,3),200)];

plot3(p1(:,1), p1(:,2), p1(:,3), 'b.'), hold on
plot3(p2(:,1), p2(:,2), p2(:,3), 'r.'), hold on
%plot3(p3(:,1), p3(:,2), p3(:,3), 'g.')

hold on
surf(xs,ys,zs);
colormap([1,1,1]);

hold on
plotcoord3(eye(4),1.5,'r','g','b');


end