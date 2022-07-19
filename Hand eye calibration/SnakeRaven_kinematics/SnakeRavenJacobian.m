function [J] = SnakeRavenJacobian(Tendeffector,isright,design,q)
% Computes Jacobian for the snake manipulator on the RAVEN as a
% 6xN matrix where N is the number of DOF in q
%
% The calculation is similar to the forward kinematics but takes positions
% of each joint and the rotation axes to create the 6xN Jacobian matrix
% where N is the number of joints in vector q.
%
% Inputs:
% Tendeffector = [4x4] endeffector matrix in reference to the world frame
%        Solved via Forward Kinematics Function SnakeRavenFK
%
% isright is a bool that chooses if its left or right 0 = left;
% 1 = right
%
% This sets the RAVEN base frame in reference to the world frame
% These options lead to base frames: 
% B_TLeft = [0 0 1 300.71; 0 -1 0 61; 1 0 0 -7; 0 0 0 1];
% B_TRight = [0 0 -1 -300.71; 0 1 0 61; 1 0 0 -7; 0 0 0 1];
%
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
% where M is the number of modules
% 
% q = is the joint vector
% q = [q1 q2 q3 q4 q5 q6 q7]
% where q1-3 are RAVEN RCM joints theat1, theta2 and insertion axis
% q4-5 are the proximal module pan and tilt theta angles
% q6-7 are the distal module pan and tilt theta angles
%
% Output: J = [6xN] Jacobian matrix relating the endeffector 6DOF to the
%             joint vector q
% 
% written by Andrew Razjigaev 2019

%Initialise Jacobian matrix
N = length(q);
J = zeros(6,N);

%Rotation axes for x,y,z motion
ax = [1 0 0]';
ay = [0 1 0]';
az = [0 0 1]';

%Endeffector position vector:
Pg = Tendeffector(1:3,4);

%Compute The Raven Insertion Transforms:
La12 = deg2rad(75); La23 = deg2rad(52);

%Select appropriate arm kinematics
if isright==1
    %Right arm:
    RCM_frame = [0 0 -1 -300.71; 0 1 0 61; 1 0 0 -7; 0 0 0 1];
    T1 = DHmatrix(pi,0,0,q(1));
    T2 = DHmatrix(La12,0,0,q(2));
    T3 = DHmatrix(La23,0,q(3),-pi/2);
else
    %Left arm:
    RCM_frame = [0 0 1 300.71; 0 -1 0 61; 1 0 0 -7; 0 0 0 1];
    T1 = DHmatrix(0,0,0,q(1));
    T2 = DHmatrix(La12,0,0,q(2));
    T3 = DHmatrix(pi-La23,0,q(3),pi/2);
end

%Solving the Raven Jacobians Jq1 Jq2 Jq3
%Jq1:
Trx = RCM_frame*T1;
Prx = Trx(1:3,4);
Rrx = Trx(1:3,1:3);
wz = Rrx*az; %get axis relative to base 
J(:,1) = [cross(wz,Pg - Prx); wz];

%Jq2:
Try = Trx*T2;
Pry = Try(1:3,4);
Rry = Try(1:3,1:3);
wz = Rry*az;
J(:,2) = [cross(wz,Pg - Pry); wz];

%Jq3: z insertion
Tend = Try*T3;
wz = Tend(1:3,1:3)*az;
J(:,3) = [wz; zeros(3,1)];

%Now continue from baseframe of segment to the end:
%Snakebot rule: if this is true then the initial disk is a pan then a tilt
pan_first = true;

%Number of segments is length of alpha or w or n or d variables:
segs = design.M;
%Variables for disk counting
first_disk = 1;

for k = 1:segs
    %Get the design parameters for the first segment:
    alpha = design.alpha(k);
    w = design.w;
    n = design.n(k);
    d = design.d(k);
    
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
    
    %Initialise the Jacobians for the pan and tilt components
    Ja_pan = zeros(6,np);  
    Ja_tilt = zeros(6,nt);
    
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
                %Concatenate to Jacobian Pan
                Ppan = Tend(1:3,4); Rpan = Tend(1:3,1:3); wx = Rpan*ax;
                Ja_pan(:,ceil((ii-(first_disk-1))/2)) = [cross(wx,Pg - Ppan); wx];
                
            else %tilt disk for ii = 2,4,6,8...
                Tend = Tend*Ttilt;
                %Concatenate to Jacobian tilt
                Ptilt = Tend(1:3,4); Rtilt = Tend(1:3,1:3); wy = Rtilt*ay;
                Ja_tilt(:,(ii-(first_disk-1))/2) = [cross(wy,Pg - Ptilt); wy];
                
            end           
        else %tilt first case:
            if isodd(ii-(first_disk-1)) %tilt disk for ii = 1,3,5,7... relative to first disk
                Tend = Tend*Ttilt;          
                %Concatenate to Jacobian tilt
                Ptilt = Tend(1:3,4); Rtilt = Tend(1:3,1:3); wy = Rtilt*ay;                
                Ja_tilt(:,ceil((ii-(first_disk-1))/2)) = [cross(wy,Pg - Ptilt); wy];
                
            else %pan disk for ii = 2,4,6,8...
                Tend = Tend*Tpan;            
                %Concatenate to Jacobian Pan
                Ppan = Tend(1:3,4); Rpan = Tend(1:3,1:3); wx = Rpan*ax;
                Ja_pan(:,(ii-(first_disk-1))/2) = [cross(wx,Pg - Ppan); wx];
                
            end    
        end
    end
    
    %Reduce the apparent Jacobians as one pan and one tilt joint column:
    %Jq4,6... component pan
    if np~=0
        J(:,4+(k-1)*2) = sum(Ja_pan,2)/np;
    else
        J(:,4+(k-1)*2) = zeros(6,1); %if no pan disks
    end
    %Jq5,7... component tilt
    if nt~=0
        J(:,5+(k-1)*2) = sum(Ja_tilt,2)/nt;
    else
        J(:,5+(k-1)*2) = zeros(6,1); %if no tilt disks
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
end