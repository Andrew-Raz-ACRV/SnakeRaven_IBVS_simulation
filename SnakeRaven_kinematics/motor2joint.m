function [q] = motor2joint(mv,design,calibration)
% Compute q given motor value m
%

%Assume correspondance m/q = [rx ry rtz pan1 tilt1 pan2 tilt2 ...]

%Initialise joint vector as 0
m = length(design.alpha);
q = zeros(3+2*m,1);

%Reverse Linear Calibration to Raven Motor space:
for ii=1:length(q)
    mv(ii) = (mv(ii) - calibration.offset(ii))/calibration.rate(ii);
end

%Raven joints are direct mapping
for ii=1:3
    q(ii) = mv(ii);
end

%Snake joints
pan_first = true;
%mm diameter of actuation on adaptor
da = 10;
ra = da/2;
    
for k=1:m
    
    %Define Variables for problem
    %Snakebot section
    n = design.n(k);
    w = design.w;
    alpha = design.alpha(k);
    r = (w/2)/(sin(alpha));
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
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if k~=1
        %Not first segment compute coupling of tendons up to that module
        q_isolated = q;
    
        %proximal independent of distal (i.e. make distal 0 estimate the
        %tendon lengths for distal given that it is isolated)
        q_isolated(4+(k-1)*2) = 0;
        q_isolated(5+(k-1)*2) = 0;

        %Get the change in tendon lengths of the isolated version
        dl0 = GetTendonLengths(design,zeros(length(q),1));
        dl1 = GetTendonLengths(design,q_isolated);
        ddl = dl1 - dl0;

        %Get motor component from coupling
        mv_coupling = zeros(size(mv));
        mv_coupling(4+(k-1)*2) = ddl(1+(k-1)*2,1)/(da/2); %Pan coupling
        mv_coupling(5+(k-1)*2) = ddl(2+(k-1)*2,1)/(da/2); %tilt coupling
       
        %Subtract coupling component from mv
        mv = mv - mv_coupling;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Now treat the problem as if its just the one module
    %Initial guess
    q_ = zeros(2,1);
    m_actual = [mv(4+(k-1)*2),  mv(5+(k-1)*2)]'; %Extract current mv for the segment
    
    if np==0
        %No pan no coupling
        q_(2) = 2*nt*(alpha - acos(cos(alpha) - (m_actual(2)*ra/(nt*w))*sin(alpha)));
    elseif nt==0
        %No tilt no coupling
        q_(1) = 2*np*(alpha - acos(cos(alpha) - (m_actual(1)*ra/(np*w))*sin(alpha)));
    else
        %Pan and tilt both couple
        %estimate minimising q error
        %Compute m given q_estimate
        m_estimate = [(2*r*np*(cos(alpha) - cos(alpha - (q_(1)/(2*np)))) + 2*r*nt*(1 - cos(q_(2)/(2*nt))))/ra; ...
               (2*r*nt*(cos(alpha) - cos(alpha - (q_(2)/(2*nt)))) + 2*r*np*(1 - cos(q_(1)/(2*np))))/ra];

        error = m_actual - m_estimate;

        while(norm(error)>1.0e-6)
            %Compute actuation jacobian
            Ja = [(-r/ra)*sin(alpha - q_(1)/(2*np)), (r/ra)*sin(q_(2)/(2*nt));
                (r/ra)*sin(q_(1)/(2*np)), (-r/ra)*sin(alpha - q_(2)/(2*nt))];

            %Compute Update
            dq = Ja\error;

            %Update estimate for q
            q_ = q_ + dq;

            %Compute resulting m_estimate
            m_estimate = [(2*r*np*(cos(alpha) - cos(alpha - (q_(1)/(2*np)))) + 2*r*nt*(1 - cos(q_(2)/(2*nt))))/ra; ...
                    (2*r*nt*(cos(alpha) - cos(alpha - (q_(2)/(2*nt)))) + 2*r*np*(1 - cos(q_(1)/(2*np))))/ra];

            %Compute new error
            error = m_actual - m_estimate; 
        end  
    end
    
    %Append solution q_ to the q vector
    q(4+(k-1)*2) = q_(1); %pan
    q(5+(k-1)*2) = q_(2); %tilt
    
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

end