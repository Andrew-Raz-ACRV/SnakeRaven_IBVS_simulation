function [dexterity] = GenerateServicesphere(T,s)
% Given transformations in T size (4,4,N) N transforms
% Generate the binary surface matrix
% That is Ntheta by Nh with (1,1) corresponding to
% the singular corner (-pi,1), and given the servicesphere parameters
% [Ntheta, Nh, dtheta, dh] this function plots the unit service sphere with
% green patches for the regions that are '1' in the surface matrix and
% blank region for regions that are '0' in the surface matrix
% s is a structure with x y and z bounds in form [xmin xmax]
%made in 24 6 20

%service sphere params
ss_params = [18,9,0.349065850398866,0.222222222222222];

%Get coordinates in on sphere surface from ss_params
Ntheta = ss_params(1); Nh = ss_params(2); 
dtheta = ss_params(3);  dh = ss_params(4);

%Generate Service Sphere
N = length(T);
surface_matrix = false(Ntheta,Nh);

for kk = 1:N
    %extract rotation
    R = T(1:3,1:3,kk);
    
    %extract translation:
    t = T(1:3,4,kk);
    
    %Check spatial_bounds
    if ~any(isnan(R(:))) %&&isinBounds(t,s.xbounds,s.ybounds,s.zbounds)
        %Compute the coordinates on sphere:
        z = R*[0 0 1]';
        %Get unit vector version
        z = z/norm(z);
        %longitudinal angles
        theta = atan2(z(2),z(1));
        %latitudinal angles
        h = z(3);

        %Do update

        %Find relative coordinate to the top left corner of the matrix which 
        %corresponds to point (-pi,1) rather than to the centre (0,0):
        th_r = theta - (-pi); h_r = - (h - 1);
        %Round the relative angles to the nearest grid:
        ii = ceil(th_r/dtheta); %column
        jj = ceil(h_r/dh); %row

        %Singular point handling 0=2*pi therefore
        if (ii == 0)
            ii = Ntheta; %make -pi become +pi position
        end

        if (jj == 0) %h = -1 goes to 1st grid (-1,-1+dh)
            jj = 1;
        end

        %Check bounds
        if (jj<1)||(jj>Nh)||(ii<1)||(ii>Ntheta)
            %do nothing something is wrong with the calculation
            disp('Failure to update service sphere');
        else
            %Everything is okay, update the surface of the matrix
            surface_matrix(ii,jj) = true;
            %a = Vcoord(1); b = Vcoord(2); c = Vcoord(3); 
            %Find corresponding region in sphere maps
            %update map
            %new_map((a-1)*Ntheta + ii, (b-1)*Nh + jj,c) = true; %ii is theta, jj is h
        end
    end
end

%Comput Dexterity from Sphere
dexterity = sum(surface_matrix(:))/(Ntheta*Nh);

%CREATE FIGURE AND PLOT SPHERE:

%Create blank white sphere
theta = linspace(-pi,pi,Ntheta+1);
h = linspace(-1,1,Nh+1);
phi = asin(h); %%angle phi
[theta,phi] = meshgrid(theta,phi);
[xs,ys,zs] = sph2cart(theta,phi,1);
surf(xs,ys,zs);
colormap([1,1,1]);

%Set up the figure showing the update
C = [0,1,0]; %rgb color make it green for the patches
hold on
plotcoord3(eye(4),1.5,'r','g','b');
title('Service Sphere')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
axis equal
grid on
                
%Go through the surface_matrix and put the patches of green on sphere
for jj = 1:Nh
    for ii = 1:Ntheta
        %Check the value in the matrix:
        if surface_matrix(ii,jj)==1
            %Draw a green patch
            %get theta and h %upper left corner and bottom right corner
            th1 = -pi + (ii-1)*dtheta;
            th2 = -pi + (ii)*dtheta;
            h1 = h(Nh+2-jj); 
            h2 = h((Nh+1)-jj); 

            %Get x-y axis projection of the unit vector using pythagoras
            r1 = sqrt(1-(h1^2)) + 0.001; %added to ensure it overlays the white patch
            r2 = sqrt(1-(h2^2)) + 0.001;
            
            if any(isreal([r1 r2])==false)
                disp('Warning: Imaginary parts something went wrong...')
                display(r1)
                display(r2)
                display(h1)
                display(h2)
                disp('end')
            end
            
            %Convert the region to 4 3D points:
            %1:topL,2:bottomL,3:bottomR,4:bottomL
            x1 = r1*cos(th1); y1 = r1*sin(th1); z1 = h1;
            x2 = r2*cos(th1); y2 = r2*sin(th1); z2 = h2;
            x3 = r2*cos(th2); y3 = r2*sin(th2); z3 = h2;
            x4 = r1*cos(th2); y4 = r1*sin(th2); z4 = h1;
            
            %make coordinates of the square patch
            X = [x1 x2 x3 x4];
            Y = [y1 y2 y3 y4];
            Z = [z1 z2 z3 z4];
            
            %plot patch
            hold on
            patch(X,Y,Z,C);
        end
    end
end

end