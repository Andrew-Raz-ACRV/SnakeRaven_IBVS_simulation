function [Traj,Rend,tend] = ForwardKinematicsVariableSegmentTraj(EntranceFrame,tooltransform,design,q,Voxelsize,Traj_initial)
%Computes forward Kinematics for a variable segment continuum on raven insertion
%tube. tend = [x y z]', Rend = rotation matrix. 
%The design alpha = [a1 a2 a3... aN] that is N segment robot
% Traj = [P1, P2, ... tend] where P(i) is each position of disk in robot

%Extract Design values
alphas = design.alpha; %[a1 a2 a3 a4...]
ws = design.w;
ns = design.n;
ds = design.d;

%Size of the voxel for trajectory measurement
dz = Voxelsize(3);

%Compute The Raven Insertion
nr = ceil(q(3)/dz); %discretisation based on voxel

%Get RCM Base Frame from Anatomy frame:
isright = true;
RCM_frame = EntranceFrame; %*[Ry(deg2rad(90)) [0 0 0]'; 0 0 0 1];
%Traven = EntranceFrame*[Rx(q(1))*Ry(q(2)) [0 0 0]'; 0 0 0 1]; 

%Compute The Raven Insertion Transform:
La12 = deg2rad(75); La23 = deg2rad(52);

%Select appropriate arm kinematics
if isright==true
    %Right arm:
    T1 = DHmatrix(pi,0,0,q(1));
    T2 = DHmatrix(La12,0,0,q(2));
    T3 = DHmatrix(La23,0,0,-pi/2);
else
    %Left arm:
    T1 = DHmatrix(0,0,0,q(1));
    T2 = DHmatrix(La12,0,0,q(2));
    T3 = DHmatrix(pi-La23,0,0,pi/2);
end

%Start Kinematic Chain:
Traven = RCM_frame*T1*T2*T3;

%Trajectory
TrajRaven = zeros(3,nr);
adz = q(3)/nr; %apparent delta in trajectory
for ii = 1:nr
Traven = Traven*txyz(0,0,adz);
TrajRaven(:,ii) = Traven(1:3,4); 
end
Tend = Traven;

%Without discretising the Raven straight section:
% Traven = [Rx(q(1))*Ry(q(2)) [0 0 0]'; 0 0 0 1]*txyz(0,0,q(3));
% baseframe = EntranceFrame*Traven;
% Tend = baseframe;

%Default rules: if this is true then the initial disk is a pan then a tilt
pan_first = true;

%Compute Trajectory while getting forward Kinematics

%Number of segments is length of alpha or w or n or d variables:
segs = length(alphas);

%Prediction of trajectory length is involves 
%taking a straight configuration and slicing it


%Initialise the trajectory vector and positioner:
Traj = zeros(3,Traj_initial);
positioner = 1;
%Variables for disk counting
first_disk = 1;

for k = 1:segs
    %Get the design parameters for the first segment:
    alpha = alphas(k);
    w = ws(k);
    n = ns(k);
    d = ds(k);
    
    %Compute other paramters:
    rad = w/2;
    r = rad / sin(alpha);
    
    %Decide how many pan and tilt disks are needed
    if pan_first == true
        %if pan first then split n into np and nt like so       
        np = round(n/2);
        nt = n - np;        
    else
        %if tilt first then split n into np and nt like so       
        nt = round(n/2);
        np = n - nt; 
    end
    
    %Extract Joint angles pan and tilt for kth segment
    theta_p = q(4+(k-1)*2); %i.e k =1 q4, k=2 its q6
    theta_t = q(5+(k-1)*2);
    
    %Compute Transform Matrices
    %Pan transform  0 to 1
    r01 = [0 -2*r*(1 - cos(alpha)*cos(theta_p/(2*np)))*sin(theta_p/(2*np)) ...
        2*r*(1 - cos(alpha)*cos(theta_p/(2*np)))*cos(theta_p/(2*np))]';
    r11 = [0 0 d]';

    %Breaking it down into Rotation and discretised steps:
    Tpan01 = [Rx(theta_p/np) [0 0 0]'; %r01;
            zeros(1,3)     1];    

    %Tilt  transform 1 to 2
    r12 = [2*r*(1 - cos(alpha)*cos(theta_t/(2*nt)))*sin(theta_t/(2*nt)) 0 ...
        2*r*(1 - cos(alpha)*cos(theta_t/(2*nt)))*cos(theta_t/(2*nt))]';

    Ttilt12 = [Ry(theta_t/nt) [0 0 0]'; %r12;
            zeros(1,3)     1];    

    %Discretise the pan translations:
    distancep = norm(r01);
    samples_pan = ceil(distancep/dz);
    dr01 = r01/samples_pan; %(norm/samples)*(vector/norm)

    %Discretise the tilt translations:
    distancet = norm(r12);
    samples_tilt = ceil(distancet/dz);
    dr12 = r12/samples_tilt; %(norm/samples)*(vector/norm)

    %Discretise the straight translations:
    distances = norm(r11); %same as r22
    samples_straight = ceil(distances/dz);
    dr11_22 = r11/samples_straight; %(norm/samples)*(vector/norm)

    %Plot all disks in the segment in a for loop based on total disk length:
    for ii = first_disk:(first_disk+n-1)
        if pan_first==true %Pan first case
            if isodd(ii-(first_disk-1)) %pan disk for ii = 1,3,5,7... relative to first disk
                
                %Translation Discretisation:
                for dv = 1:samples_pan
                    Tend = Tend*[eye(3) dr01; 0 0 0 1];
                    Traj(:,positioner) = Tend(1:3,4);
                    positioner = positioner + 1;
                end
                Tend = Tend*Tpan01; %Rotation only
                for dv = 1:samples_straight
                    Tend = Tend*[eye(3) dr11_22; 0 0 0 1];
                    Traj(:,positioner) = Tend(1:3,4);
                    positioner = positioner + 1;
                end
            else %tilt disk for ii = 2,4,6,8...
                
                %Translation Discretisation:
                for dv = 1:samples_tilt
                    Tend = Tend*[eye(3) dr12; 0 0 0 1];
                    Traj(:,positioner) = Tend(1:3,4);
                    positioner = positioner + 1;
                end
                Tend = Tend*Ttilt12;%Rotation only
                for dv = 1:samples_straight
                    Tend = Tend*[eye(3) dr11_22; 0 0 0 1];
                    Traj(:,positioner) = Tend(1:3,4);
                    positioner = positioner + 1;
                end 
            end           
        else %tilt first case:
            if isodd(ii-(first_disk-1)) %tilt disk for ii = 1,3,5,7... relative to first disk
                
                %Translation Discretisation:
                for dv = 1:samples_tilt
                    Tend = Tend*[eye(3) dr12; 0 0 0 1];
                    Traj(:,positioner) = Tend(1:3,4);
                    positioner = positioner + 1;
                end
                Tend = Tend*Ttilt12;%Rotation only
                for dv = 1:samples_straight
                    Tend = Tend*[eye(3) dr11_22; 0 0 0 1];
                    Traj(:,positioner) = Tend(1:3,4);
                    positioner = positioner + 1;
                end  
            else %pan disk for ii = 2,4,6,8...
                
                %Translation Discretisation:
                for dv = 1:samples_pan
                    Tend = Tend*[eye(3) dr01; 0 0 0 1];
                    Traj(:,positioner) = Tend(1:3,4);
                    positioner = positioner + 1;
                end
                Tend = Tend*Tpan01; %Rotation only
                for dv = 1:samples_straight
                    Tend = Tend*[eye(3) dr11_22; 0 0 0 1];
                    Traj(:,positioner) = Tend(1:3,4);
                    positioner = positioner + 1;
                end
            end    
        end
    end
    
    
    %Transition disk addition is embedded into the nth disk of the segment
    if k~=segs 
        %Embed the inbetween joint with the previous segment
        zeta = deg2rad(-90/segs); %make rotation given cables of equal spacing
        %Transform proximal to distal segment 
        Tpd = [Rz(zeta) [0 0 0]'; 0 0 0 1]; 
        Tend = Tend*Tpd; %Transition transform after nth dik
    end
      
    %Decide if the next segment should be pan first or not based on the pan
    %tilt pattern being preserved.
    if (pan_first==true)&&(isodd(n))
        %e.g. n=3 : p t p -t
        pan_first = false;
    elseif (pan_first==false)&&(isodd(n))
        %e.g. n=3 : t p t -p
        pan_first = true;
    elseif (pan_first==true)&&(isodd(n)==false)
        %e.g. n=4 : p t p t -p
        pan_first = true;
    elseif (pan_first==false)&&(isodd(n)==false)
        %e.g. n=4 : t p t p -t
        pan_first = false;     
    end
    
    %Update first_disk for next segment
    first_disk = first_disk + n;
end



    %Append tool tip to trajectory remember the tool is 5mm
    
    %Toolframe is a z axis distance:
    distance_tool = norm(tooltransform(1:3,4));
    samples_tool = ceil(distance_tool/dz);
    dtool = tooltransform(1:3,4)/samples_tool;
    for dv = 1:samples_tool
        Tend = Tend*[eye(3) dtool; 0 0 0 1];
        Traj(:,positioner) = Tend(1:3,4);
        positioner = positioner + 1;        
    end
    
    %Extract Endeffector final Rotation and position
    %Tend = Tend*tooltransform; done
    Rend = Tend(1:3,1:3);
    tend = Tend(1:3,4);
    
    %Combine Raven and trajectory snake
    Traj = [TrajRaven, Traj];
end