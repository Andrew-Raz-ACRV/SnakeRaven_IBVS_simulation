function [Tendon_lengths] = GetTendonLengths(design,q)
% Computes the tendon lengths for a snake like robot
%
% Inputs:
% design = is a structure with the parameter vector and toolframe
%
% design.alpha = [a1 a2 a3... aM] the alpha rolling joint angle
% design.n = [n1 n2 n3... nM] number of rolling joints
% design.d = [d1 d2 d3... dM] the distance between rolling joint curves
% design.w = w           the width of the manipulator tendon to tendon
%                             which should all be 4mm for consistency
% design.tool = [4x4] transform matrix from the end of the final joint 
%               transform to the tip of the endeffector
%
% q = is the joint vector
% q = [q1 q2 q3 q4 q5 q6 q7]
% where q1-3 are RAVEN RCM joints theat1, theta2 and insertion axis
% q4-5 are the proximal module pan and tilt theta angles
% q6-7 are the distal module pan and tilt theta angles
%
% Output: Tendon_lengths = [2*Mx2] matrix with tendon lengths in order: 
% 2x2 blocks:
% [pan Left (pl), pan right (pr);
% tilt Left (tl), tilt right (tr)];
% per segment
%
% written by Andrew Razjigaev 2019


%Initialise Tendon Length Matrix:
L = zeros(length(q)-3,2);

Tend = eye(4); %Start Doesn't matter because tendon lengths are independent of this

%Snakebot rule: if this is true then the initial disk is a pan then a tilt
pan_first = true;

%Number of segments is length of alpha or w or n or d variables:
segs = design.M;

%Variables for disk counting
first_disk = 1;

for k = 1:segs
    %Get the design parameters for the first segment:
    alpha = design.alpha(k);
    n = design.n(k);
    d = design.d(k);
    w = design.w;
    
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
                %Measure Start Pa
                Pa = GetTendonPoints(Tend,w,alpha,d,0,1,k,segs);
                Tend = Tend*Tpan;
                %Measure End Pb
                Pb = GetTendonPoints(Tend,w,alpha,d,1,1,k,segs);
                L = AppendTendonMeasurement(Pa,Pb,L);
            else %tilt disk for ii = 2,4,6,8...
                %Measure Start Pa
                Pa = GetTendonPoints(Tend,w,alpha,d,0,0,k,segs);
                Tend = Tend*Ttilt;
                %Measure End Pb
                Pb = GetTendonPoints(Tend,w,alpha,d,1,0,k,segs);
                L = AppendTendonMeasurement(Pa,Pb,L);
            end           
        else %tilt first case:
            if isodd(ii-(first_disk-1)) %tilt disk for ii = 1,3,5,7... relative to first disk
                %Measure Start Pa
                Pa = GetTendonPoints(Tend,w,alpha,d,0,0,k,segs);
                Tend = Tend*Ttilt;  
                %Measure End Pb
                Pb = GetTendonPoints(Tend,w,alpha,d,1,0,k,segs);
                L = AppendTendonMeasurement(Pa,Pb,L);
            else %pan disk for ii = 2,4,6,8...
                %Measure Start Pa
                Pa = GetTendonPoints(Tend,w,alpha,d,0,1,k,segs);
                Tend = Tend*Tpan;
                Pb = GetTendonPoints(Tend,w,alpha,d,1,1,k,segs);
                L = AppendTendonMeasurement(Pa,Pb,L);
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
%Output Tendon Lengths
Tendon_lengths = L;
end