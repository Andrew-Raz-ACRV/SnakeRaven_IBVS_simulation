%% Snakebot Design Initialisation Script:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Snakebot Raven Design Variables:

%One Module Design
design1 = struct('alpha',1.24,'n',3,'d',1.62,'w',4,...
    'M',1,'tooltransform',txyz(0,0,5),'qL',0,'qU',0);

%Two Module Design
design2 = struct('alpha',[0.2 0.88],'n',[3 3],'d',[1 1],'w',4,...
    'M',2,'tooltransform',txyz(0,0,5),'qL',0,'qU',0);
%old 'alpha',[1.39 1.18],'n',[1 3],'d',[6 0.41]

%Choose design to simulate:
design = design2;

%Raven arm variable 
Right = 1; Left = 0;

%Compute Design Joint Limits:
%Raven x rotation
qrxL = -2*pi; qrxU = 2*pi;%radians
%Raven y rotation
qryL = -2*pi; qryU = 2*pi;
%Raven z translation
qrzL = -300; qrzU = 300; %mm

%Joint Limits RAVEN level:
qL = [qrxL,qryL,qrzL];
qU = [qrxU,qryU,qrzU];

%Segment Pan Tilt Joint Limit calculation:
for ii = 1:design.M
    %Continuum Joints: Lower Upper
    theta_max = (design.alpha(ii)*design.n(ii))/2; %Maximum bending
    %pan
    qipL = -theta_max;    qipU = theta_max;
    %tilt
    qitL = -theta_max;    qitU = theta_max;
    %Append segment configurations to whole configuration space:
    qL(:,(4+(ii-1)*2):(5+(ii-1)*2)) = [qipL, qitL];
    qU(:,(4+(ii-1)*2):(5+(ii-1)*2)) = [qipU, qitU];
end
%Save joint limits into the design structure
design.qL = qL; design.qU = qU;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initial Configuration:
    % -39.5967  -77.9160 %The solution to the homing problem (get
    % snakeRaven to be vertical see Solving_home_position
    %Left side it is 39.5967 -102.0840
    %deg2rad(-36),deg2rad(-85)

if design.M>1
    if Right==1
        %q0 = [deg2rad(-39.5967),deg2rad(-77.9160),0, deg2rad(-5),deg2rad(0),deg2rad(1),deg2rad(-45)]';
        %q0 = [ 0, 0, 25, 0.3, 0.2, 0.3, 0.2];
        q0 = [deg2rad(-39.5967),deg2rad(-77.9160),0, deg2rad(0),deg2rad(0),deg2rad(0),deg2rad(0)]';
    else
        q0 = [deg2rad(39.5967),deg2rad(-102.0840),0, 0,0,0,0]';
    end
    calibration = struct('rate',ones(7,1),'offset',zeros(7,1));
else
    if Right==1
        q0 = [deg2rad(-39.5967),deg2rad(-77.9160),0 ,deg2rad(1),deg2rad(-30)]';
    else
        q0 = [deg2rad(39.5967),deg2rad(-102.0840),0 ,deg2rad(0),deg2rad(0)]';
    end
    calibration = struct('rate',ones(5,1),'offset',zeros(5,1));
end

%Starting joint angles
q = q0;
mj = joint2motor(q,design,calibration);
