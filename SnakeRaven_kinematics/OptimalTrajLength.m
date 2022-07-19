function [traj_length,Tend] = OptimalTrajLength(EntranceFrame,design,toolframe,voxelsize)
    % The trajectory needs to be at an appropriate size before being used 
    % in the algorithm. This is based on a straight configuration precomputed
    % before the loop.
    
    %extract design params
    alphas = design.alpha;
    ds = design.d;
    ns = design.n;
    ws = design.w;
    segs = length(alphas);
    
    %straight configuration variable segs
    q = zeros(1,3+2*segs);
    
    %discretisation
    dz = voxelsize(3);
    
    %Compute length of robot from base to tip
    Tend = EntranceFrame;
    traj_length = 0;
    
    %Forward kinematics
    
    %Default rules: if this is true then the initial disk is a pan then a tilt
    pan_first = true;
    %Number of segments is length of alpha or w or n or d variables:
    segs = length(alphas); 
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

        %Tpan = [Rx(theta_p/np) r01+Rx(theta_p/np)*r11;
        %        zeros(1,3)                         1];
        
        %Breaking it down into Rotation and discretised steps:
        Tpan01 = [Rx(theta_p/np) [0 0 0]'; %r01;
                zeros(1,3)     1];    

%         Tpan11 = [eye(3)         [0 0 0]'; %r11;
%                 zeros(1,3)     1]; 


        %Tilt  transform 1 to 2
        r12 = [2*r*(1 - cos(alpha)*cos(theta_t/(2*nt)))*sin(theta_t/(2*nt)) 0 ...
            2*r*(1 - cos(alpha)*cos(theta_t/(2*nt)))*cos(theta_t/(2*nt))]';
        %r22 = [0 0 d]';

        %Ttilt = [Ry(theta_t/nt) r12+Ry(theta_t/nt)*r22;
        %        zeros(1,3)                         1];
        
        Ttilt12 = [Ry(theta_t/nt) [0 0 0]'; %r12;
                zeros(1,3)     1];    

%         Ttilt22 = [eye(3)         [0 0 0]'; %r22;
%                 zeros(1,3)     1]; 
        
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
                    Tend = Tend*Tpan01; %Rotation only
                    %Translation Discretisation:
                    for dv = 1:samples_pan
                        Tend = Tend*[eye(3) dr01; 0 0 0 1];
                        traj_length = traj_length+1;
                    end
                    for dv = 1:samples_straight
                        Tend = Tend*[eye(3) dr11_22; 0 0 0 1];
                        traj_length = traj_length+1;
                    end
                else %tilt disk for ii = 2,4,6,8...
                    Tend = Tend*Ttilt12;%Rotation only
                    %Translation Discretisation:
                    for dv = 1:samples_tilt
                        Tend = Tend*[eye(3) dr12; 0 0 0 1];
                        traj_length = traj_length + 1;
                    end
                    for dv = 1:samples_straight
                        Tend = Tend*[eye(3) dr11_22; 0 0 0 1];
                        traj_length = traj_length + 1;
                    end 
                end           
            else %tilt first case:
                if isodd(ii-(first_disk-1)) %tilt disk for ii = 1,3,5,7... relative to first disk
                    Tend = Tend*Ttilt12;%Rotation only
                    %Translation Discretisation:
                    for dv = 1:samples_tilt
                        Tend = Tend*[eye(3) dr12; 0 0 0 1];
                        traj_length = traj_length+1;
                    end
                    for dv = 1:samples_straight
                        Tend = Tend*[eye(3) dr11_22; 0 0 0 1];
                        traj_length = traj_length+1;
                    end  
                else %pan disk for ii = 2,4,6,8...
                    Tend = Tend*Tpan01; %Rotation only
                    %Translation Discretisation:
                    for dv = 1:samples_pan
                        Tend = Tend*[eye(3) dr01; 0 0 0 1];
                        traj_length = traj_length+1;
                    end
                    for dv = 1:samples_straight
                        Tend = Tend*[eye(3) dr11_22; 0 0 0 1];
                        traj_length = traj_length+1;
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
    
    %Toolframe is a z axis distance:
    distance_tool = norm(toolframe(1:3,4));
    samples_tool = ceil(distance_tool/dz);
    dtool = toolframe(1:3,4)/samples_tool;
    for dv = 1:samples_tool
        Tend = Tend*[eye(3) dtool; 0 0 0 1];
        traj_length = traj_length+1;
    end
    %Output:
end