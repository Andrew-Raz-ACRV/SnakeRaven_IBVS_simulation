function Tend = PlotSnakeRaven(isright,design,q)
%Plots a rendering of the snake like robot using Raven Kinematics
%in 3D plots
%
% Inputs:
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
% Output: Tend = [4x4] endeffector matrix in reference to the world frame
% 
% written by Andrew Razjigaev 2019

%Compute The Raven Insertion Transform:
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

%Plot the Raven Frames
% plotcoord3(eye(4),10,'r','g','b')
% text(0,0,0,'WORLD')

hold on
plotcoord3(RCM_frame,10,'r','g','b')
text(RCM_frame(1,4)+10,RCM_frame(2,4),RCM_frame(3,4),'RCM')
% hold on
% plotcoord3(RCM_frame*T1,10,'r','g','b')
% 
% hold on
% plotcoord3(RCM_frame*T1*T2,10,'r','g','b')
% % hold on
% % plotcoord3(RCM_frame*T1*T2*T3,10,'r','g','b')
% axis equal
%Start Kinematic Chain:
Tend = RCM_frame*T1*T2*T3;

%Default rules: if this is true then the initial disk is a pan then a tilt
pan_first = true;

%Plot insertion tube
hold on
plotInsertionTube(Tend,design.w,design.alpha(1),30,pan_first);
hold on

%Plot while doing Forward Kinematics

%Number of modules is length of alpha or w or n or d variables:
segs = design.M;

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
    
    %Extract Joint angles pan and tilt for kth segment
    theta_p = q(4+(k-1)*2); %i.e k =1 q4, k=2 its q6
    theta_t = q(5+(k-1)*2); %i.e k =1 q5, k =2 its q7
    
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
    
    %Plotting correction: I only plot pan disks I just rotate them to make them tilts  
    Ttiltrot = [Rz(pi/2) [0 0 0]';
            zeros(1,3) 1];
        
    %Decide if a transition disk is needed for this segment.
    if k == segs
        m = n; %i.e. final segment all disks are ordinary
    else
        m = n-1; %i.e. intermediate segment one disk is a transition
    end
    
    %Plot all disks in a for loop:
    for ii = 1:m
        
        if pan_first==true %Pan first case
            if isodd(ii) %pan disk for ii = 1,3,5,7...
                Tend = Tend*Tpan;
                plotVariableNeutralLineDisk(Tend,w,alpha,d)   
            else %tilt disk for ii = 2,4,6,8...
                Tend = Tend*Ttilt;
                plotVariableNeutralLineDisk(Tend*Ttiltrot,w,alpha,d)
            end           
        else %tilt first case:
            if isodd(ii) %tilt disk for ii = 1,3,5,7...
                Tend = Tend*Ttilt;
                plotVariableNeutralLineDisk(Tend*Ttiltrot,w,alpha,d)   
            else %pan disk for ii = 2,4,6,8...
                Tend = Tend*Tpan;
                plotVariableNeutralLineDisk(Tend,w,alpha,d)
            end    
        end
        hold on
    end
    
    %Transition disk plotting for nth disk
    if k~=segs 
        %Embed the inbetween joint with the previous segment
        zeta = deg2rad(-90/segs); %make rotation given cables of equal spacing
        %Transform proximal to distal segment 
        Tpd = [Rz(zeta) [0 0 0]'; 0 0 0 1]; 
        
        %Plotting the transition disk
        if pan_first==true
            if isodd(n)
                %must be a pan e.g. n=3 p t (p)-final transition disk
                Tend = Tend*Tpan;     
                Tend = Tend*Tpd; %Transition transform after transiton disk
                plotBetweenSegmentJoint(Tend,w,d,zeta,alpha,design.alpha(k+1))
            else
                %must be a tilt e.g. n=4 p t p (t)-final transition disk
                Tend = Tend*Ttilt;
                Tend = Tend*Tpd; %Transition transform after transiton disk
                plotBetweenSegmentJoint(Tend*Ttiltrot,w,d,zeta,alpha,design.alpha(k+1))               
            end
        else
            %tilt first
            if isodd(n)
                %must be a tilt e.g. n=3 t p (t)-final transition disk                
                Tend = Tend*Ttilt;
                Tend = Tend*Tpd; %Transition transform after transiton disk
                plotBetweenSegmentJoint(Tend*Ttiltrot,w,d,zeta,alpha,design.alpha(k+1))  
            else
                %must be a pan e.g. n=4 t p t (p)-final transition disk
                Tend = Tend*Tpan;     
                Tend = Tend*Tpd; %Transition transform after transiton disk
                plotBetweenSegmentJoint(Tend,w,d,zeta,alpha,design.alpha(k+1))               
            end
        end
        hold on
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
end

    %Extract Endeffector final Rotation and position
    hold on
    L = design.tooltransform(3,4);
    Tend = plotEndeffector(Tend,L);
    hold on
    axis equal
end