function [tend] = FastForwardKinematicsSnake(EntranceFrame,tooltransform,design,q)
% Computes the Forward kinematics of a snake-like manipulator to get just
% the position quickly for faster configuration rejection in the dexterity
% measure
% Andrew Razjigaev 2020

%Compute The Raven Insertion Transform:
La12 = deg2rad(75); La23 = deg2rad(52);

%Select appropriate arm kinematics

%Right arm:
T1 = DHmatrix(pi,0,0,q(1));
T2 = DHmatrix(La12,0,0,q(2));
T3 = DHmatrix(La23,0,q(3),-pi/2);

% %Left arm:
% T1 = DHmatrix(0,0,0,q(1));
% T2 = DHmatrix(La12,0,0,q(2));
% T3 = DHmatrix(pi-La23,0,q(3),pi/2);

%Start Kinematic Chain:
Tend = EntranceFrame*T1*T2*T3;

%Snakebot rule: if this is true then the initial disk is a pan then a tilt
pan_first = true;

%Number of segments is length of alpha or w or n or d variables:
segs = length(design.alpha);
%Variables for disk counting
first_disk = 1;

for k = 1:segs
    %Get the design parameters for the first segment:
    alpha = design.alpha(k);
    n = design.n(k);
    d = design.d(k);
    w = design.w(k);
    
    %Compute other paramters:
    r = (w/2) / sin(alpha);
    
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
    %Pan trasform  0 to 1
    r01 = [0 -2*r*(1 - cos(alpha)*cos(theta_p/(2*np)))*sin(theta_p/(2*np)) ...
        2*r*(1 - cos(alpha)*cos(theta_p/(2*np)))*cos(theta_p/(2*np))]';
    r11 = [0 0 d]';

    Tpan = [Rx(theta_p/np) r01+Rx(theta_p/np)*r11;
            zeros(1,3)                         1];

    %Tilt  transform 1 to 2
    r12 = [2*r*(1 - cos(alpha)*cos(theta_t/(2*nt)))*sin(theta_t/(2*nt)) 0 ...
        2*r*(1 - cos(alpha)*cos(theta_t/(2*nt)))*cos(theta_t/(2*nt))]';
    r22 = [0 0 d]';

    Ttilt = [Ry(theta_t/nt) r12+Ry(theta_t/nt)*r22;
            zeros(1,3)                         1];
    
    
    %Plot all disks in the segment in a for loop based on total disk length:
    for ii = first_disk:(first_disk+n-1)
        if pan_first==true %Pan first case
            if isodd(ii-(first_disk-1)) %pan disk for ii = 1,3,5,7... relative to first disk
                Tend = Tend*Tpan;
            else %tilt disk for ii = 2,4,6,8...
                Tend = Tend*Ttilt;
            end           
        else %tilt first case:
            if isodd(ii-(first_disk-1)) %tilt disk for ii = 1,3,5,7... relative to first disk
                Tend = Tend*Ttilt;  
            else %pan disk for ii = 2,4,6,8...
                Tend = Tend*Tpan;
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

    %Extract Endeffector final Rotation and position
    Tend = Tend*tooltransform;
    tend = Tend(1:3,4);
end