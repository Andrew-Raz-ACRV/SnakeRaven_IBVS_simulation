function PlotRenderVariableSegment(EntranceFrame,tooltransform,design,q)
%Plots a rendering of a variable segment snake like robot
%in 3D plots

%Extract Design values
alphas = design.alpha; %[a1 a2 a3 a4...]
ws = design.w;
ns = design.n;
ds = design.d;

%Compute The Raven Insertion
Traven = [Rx(q(1))*Ry(q(2)) [0 0 0]'; 0 0 0 1]*txyz(0,0,q(3));
baseframe = EntranceFrame*Traven;
Tend = baseframe;

%Plot the Entrance Frame
%plotcoord3(baseframe,2,'r','g','b')

%Default rules: if this is true then the initial disk is a pan then a tilt
pan_first = true;

%Plot insertion tube
hold on
plotInsertionTube(baseframe,ws(1),alphas(1),30,pan_first);
hold on

%Compute Trajectory while getting forward Kinematics

%Number of segments is length of alpha or w or n or d variables:
segs = length(alphas);

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
                plotBetweenSegmentJoint(Tend,w,d,zeta,alpha,alphas(k+1))
            else
                %must be a tilt e.g. n=4 p t p (t)-final transition disk
                Tend = Tend*Ttilt;
                Tend = Tend*Tpd; %Transition transform after transiton disk
                plotBetweenSegmentJoint(Tend*Ttiltrot,w,d,zeta,alpha,alphas(k+1))               
            end
        else
            %tilt first
            if isodd(n)
                %must be a tilt e.g. n=3 t p (t)-final transition disk                
                Tend = Tend*Ttilt;
                Tend = Tend*Tpd; %Transition transform after transiton disk
                plotBetweenSegmentJoint(Tend*Ttiltrot,w,d,zeta,alpha,alphas(k+1))  
            else
                %must be a pan e.g. n=4 t p t (p)-final transition disk
                Tend = Tend*Tpan;     
                Tend = Tend*Tpd; %Transition transform after transiton disk
                plotBetweenSegmentJoint(Tend,w,d,zeta,alpha,alphas(k+1))               
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
    %plotcoord3(Tend,w,'r','g','b')
    hold on
    L = tooltransform(3,4);
    Tendeffector = plotEndeffector(Tend,L);
    hold on
    %plotcoord3(Tendeffector,w,'r','g','b');
    hold on
    %disp(Tendeffector)
end