%% Raven Snakebot Control Simulator
%Andrew Razjigaev
%March 2020

clear all
close all
clc

%add functions into the path
addpath('Math_functions');
addpath('Plotting_functions');
addpath('SnakeRaven_kinematics');

%recording options
filename = 'RavenSnakeBot_Control.gif';
record = false;

%% Snakebot Design Initialisation:
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

%% Simulation Initialisation
disp('STARTING SNAKEBOT SIMULATION')

%Controller Speed:
dx_limit = 1; %mm/iteration
tol_error = 0.2; %Error magnitude threshold tol_error = 0.1;

%Initial Configuration:
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

%Defining a Target pose:
Tend = SnakeRavenFK(Right,design,q0);
R0 = Tend(1:3,1:3); t0 = Tend(1:3,4);
target_pose = [R0*Rx(deg2rad(-30)), [t0(1)-2 t0(2)-2 t0(3)-5]'; 0 0 0 1];
%target_pose = [R0*Rx(deg2rad(-45)), [t0(1) t0(2) t0(3)+7]'; 0 0 0 1]; %hard

%Create Figure
h = figure('Name','Task Space','units','normalized','outerposition',[0 0 1 1]);
clf
drawfigure(target_pose,Right,design,q0);

%% SnakeBot Control Loop:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Start moving from initial
q = q0;
mj = joint2motor(q,design,calibration);

%Control Loop to Target
moving = true; frame_count = 1; %iteration counter
Max_frames = 30;

%Gain
W = diag([1 1 1 5 5 5]);

while(moving)
    
    %Display Commands robot arm and targets
    disp('Moving...')
    clf
    drawfigure(target_pose,Right,design,q)
    pause(0.001) %frame rate 0.001 
    %pause()
    
    %Record figure frame to gif
    if record == true
        recordFrame2gif(h,filename,frame_count);
    end 
    
    %frame count 
    frame_count = frame_count + 1;
    if frame_count == Max_frames
        moving = false;
    end
    
    %Measure Toolpoint
    
    %Input mj ...
    q = motor2joint(mj,design,calibration);
    
    %Forward Kinematics
    Tend = SnakeRavenFK(Right,design,q);  
    
    %Measure the Error between transforms 
    dx = trans2dx(Tend,target_pose);
    
    %Measure error velocity dx, times it by proportional gain
    error = norm(dx);

    disp('Current Error:')
    disp(error)   
    
    %Check Target if within some pose error:
    if error<tol_error
        moving = false;
        disp('Target was reached')
    else
        %Measure error and apply speed limit:
        if error>dx_limit
            dx = cap_mag(dx,dx_limit);
        end

        %Calculate Jacobian
        J = SnakeRavenJacobian(Tend,Right,design,q);
        
        %Calculate psuedo-inverse Jacobian avoiding joint limits:        
        %inv_J = dampedLeastSquaresInverse(J,q,design.qL,design.qU);
        inv_J = (J'*J + eye(length(q))^2)\J'; %damped least squares
        
        %Calculate the update step Weighted Damped Least Squares:
        dq = inv_J*W*dx;
        
        %Integrate the joint step
        q = q + dq;
        
        %Ensure joint limits are satisfied and send motor command
        [q,hit] = applyJointLimits(q,qL,qU); 
        
        %Display error when joint limit is reached:
        if hit==true
            disp('Joint-Limit Saturation')
        end
        
        %Compute Motor Values for those joint values:
        mj = joint2motor(q,design,calibration);
        
        %Output mj ...
        
        %if time runs out
        if moving == false
            disp('Time for simulation expired');
        end
    end
end

function drawfigure(target_pose,Right,design,q)
% Draws the figure for the simulation
%Tend = PlotSnakeRavenTendons(Right,design,q); %plot with tendons
Tend = PlotSnakeRaven(Right,design,q); %plot without tendons
light('Position',[-1 -1 0.5],'Style','infinite')
plotcoord3(target_pose,5,'r','g','b')
hold on
plotcoord3(Tend,5,'r','g','b') 
grid on
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
view(3)
%view([90,10])
view([20,10])
axis equal
end

%     %Forward Kinematic speed calc:
%     disp('FK speed run...')
%     trials = 1000000;
%     disp('SnakeRavenFK time:')
%     tic
%     for kk = 1:trials
%         Tend = SnakeRavenFK(Right,design,q);
%     end
%     disp(toc/trials)
%     EntranceFrame = [0 0 -1 -300.71; 0 1 0 61; 1 0 0 -7; 0 0 0 1]; 
%     Voxelsize = [2 2 2];
%     design.w = [4 4];
%     [traj_length,~] = OptimalTrajLength(EntranceFrame,design,design.tooltransform,Voxelsize); 
%     disp('FastFK time:')    
%     tic
%     for kk = 1:trials
%         tend = FastForwardKinematicsSnake(EntranceFrame,design.tooltransform,design,q);
%     end  
%     time1 = toc/trials;
%     disp(time1)
%     disp('FK_traj time:') 
%     tic
%     for kk = 1:trials
%         [Traj,Rend,tend] = ForwardKinematicsVariableSegmentTraj(EntranceFrame,design.tooltransform,design,q,Voxelsize,traj_length);
%     end  
%     time2 = toc/trials;
%     disp(time2)    
%     disp('Advantage of: ')
%     disp(time2/time1)