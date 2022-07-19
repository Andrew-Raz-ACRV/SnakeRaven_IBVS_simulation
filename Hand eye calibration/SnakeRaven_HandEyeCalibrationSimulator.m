%% SNAKERAVEN HAND EYE CALIBRATOR
clear all,
close all,
clc

%add snake functions into the path
% addpath('Math_functions');
% addpath('Plotting_functions');
% addpath('SnakeRaven_kinematics');

%Number of images:
Frames = 5;

%% *****INITIAISATION SNAKERAVEN********%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%INITAL CONFIGURATION
if design.M>1
    if Right==1
        q0 = [deg2rad(-39.5967),deg2rad(-77.9160),0, deg2rad(0),deg2rad(0),deg2rad(0),deg2rad(0)]';
    else
        q0 = [deg2rad(39.5967),deg2rad(-102.0840),0, 0,0,0,0]';
    end
else
    if Right==1
        q0 = [deg2rad(-39.5967),deg2rad(-77.9160),0 ,deg2rad(1),deg2rad(-30)]';
    else
        q0 = [deg2rad(39.5967),deg2rad(-102.0840),0 ,deg2rad(0),deg2rad(0)]';
    end
end
%***********Joint Limits**********%
%Raven x rotation
qrxL = q0(1)-pi/4; qrxU = q0(1)+pi/4;%radians
%Raven y rotation
qryL = q0(2)-pi/4; qryU = q0(2)+pi/4;
%Raven z translation
qrzL = q0(3)-50; qrzU = q0(3)+100; %mm

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

%% *********HAND EYE CALIBRATION DATA MOVEMENT********%
%Series of Joint Angles for HAND EYE
q = [q0';
     q0(1)+0.1, q0(2)+0.1, q0(3)+5, q0(4)+0.1, q0(5)+0.1, q0(6)+0.1, q0(7)+0.1;
     q0(1)-0.1, q0(2)-0.1, q0(3)+10, q0(4)-0.1, q0(5)-0.1, q0(6)-0.1, q0(7)-0.1;
     q0(1)+0.1, q0(2)+0.1, q0(3)+5, q0(4)+0.2, q0(5)+0.2, q0(6)+0.2, q0(7)+0.2;
     q0(1)-0.1, q0(2)-0.1, q0(3)+10, q0(4)-0.2, q0(5)-0.2, q0(6)-0.2, q0(7)-0.2;
     q0(1)-0.1, q0(2)-0.1, q0(3)+10, q0(4)-0.2, q0(5)-0.2, q0(6)-0.2, q0(7)-0.2];
    %qL + (qU-qL).*rand(Frames-1,1)]; % random configurations
%r = qL + (qU-qL).*rand(100,1);

%Determine the number 
Number_of_Views = size(q,1);

%Data pool from movements

%Hmarker2world      a 4x4xNumber_of_Views Matrix of the form
%                   Hmarker2world(:,:,i) = [Ri_3x3 ti_3x1;[ 0 0 0 1]] 
%                   with 
%                   i = number of the view, 
%                   Ri_3x3 the rotation matrix 
%                   ti_3x1 the translation vector.
%                   Defining the transformation of the robot hand / marker
%                   to the robot base / external tracking device
%Hgrid2cam          a 4x4xNumber_of_Views Matrix (like above)
%                   Defining the transformation of the grid to the camera
%
Hmarker2world = zeros(4,4,Number_of_Views);
Hgrid2cam = zeros(4,4,Number_of_Views);


%% **************INITIALISE HAND EYE ENVIRONMENT************%
%World to Base of the Robot:
Twb = eye(4);
%Twb = transl(0, 0, 0) * trotx(pi);
%bot = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'Kinova Arm','base',Twb);

%Calibration Target:
Twt = SnakeRavenFK(Right,design,q0)*txyz(0,0,100)*[Rz(-pi/3) [0 0 0]'; 0 0 0 1];
P = mkgrid(3, 10, 'pose', Twt);
Tta = [Rz(pi)*Rx(pi) [0 0 0]'; 0 0 0 1]; %Target to Aruco
Twa = Twt*Tta;
%P = mkcube(0.2,'pose',Tcali);

%Hand to Eye answer GROUND TRUTH:
%Tgc = transl(0.04, 0.05, -4) * trotz(deg2rad(118)); %Gripper to Camera
% Tgc = [0.751393 0.656326 -0.0681615 -3.45668;
%        -0.634118 0.74679 0.200497 0.132972;
%        0.182493 -0.107429 0.97732 -10.2099;
%         0 0 0 1];
% Tgc = [0.985052 -0.16286 -0.0561147 0.202904;
%        0.141502 0.950814 -0.275555 0.202904; %3.45743;
%        0.0982315 0.263495 0.959646 -3.45668; %-9.67357;
%         0 0 0 1];

%Best
%Tgc = [0.983941 0.153045 0.0918522 0.202904;
%       -0.172836 0.945449 0.27614 0.202904; %3.45743;
%       -0.0445798 -0.287581 0.956718 -3.45668; %-9.67357;
%        0 0 0 1];

Tgc = [Rz(deg2rad(118 + 180 + 22)) [0 0 -4]'; 0 0 0 1]; %228

%Tgc = Tnorm(Tgc);

%Initialise the Eye
cam = CentralCamera('focal', 0.015, 'pixel', 10e-6, ...
'resolution', [1280 1024], 'centre', [640 512], 'name', 'Eye');

%% ********VISUALISE HAND EYE CALIBRATION PROCEDURE*******%

for ii = 1:Number_of_Views

%Ai - World to gripper Transform - Forward Kinematics
Twg = SnakeRavenFK(Right,design,q(ii,:));         % bot.fkine(q(ii,:)); 

%Defining the transformation of the robot hand / marker
%to the robot base / external tracking device
Hmarker2world(:,:,ii) = Twg;


%Bi - Camera to Target Transform
Twc = Twg*Tgc;
Cam = cam.move(Twc);
Tct = Twc\Twt;

%Defining the transformation of the grid to the camera
Hgrid2cam(:,:,ii) = Tct;

%Plot Environment

    figure(1)
    %Plot Robot
    plotcoord3(eye(4),10,'r','g','b')
    Tend = PlotSnakeRaven(Right,design,q(ii,:)); %plot without tendons
    light('Position',[-1 -1 0.5],'Style','infinite')
    hold on
    plotcoord3(Twa,10,'r','g','b')
    hold on
    grid on
    xlabel('x-axis')
    ylabel('y-axis')
    zlabel('z-axis')
    view(3)
    %view([90,10])
    view([20,10])
    axis equal
    hold on
    
    plot_sphere(P, 1, 'r'); %Plot Calibration Target:
    hold on
    Cam.plot_camera('scale',3); %Plot eye
    axis equal
    Cam.plot(P); %plot what the eye sees in image plane
    

    disp('running')
    disp(ii)
    pause()
    clc
end

disp('Done movements')

%% ********HAND EYE CALIBRATION COMPARISON*******%

disp('Ground truth:')
display(Tgc)

disp('Testing Hourads quaternion method:')
[Hcam2marker_, err] = inria_calibration(Hmarker2world, Hgrid2cam);
display(Hcam2marker_)
display(err)

% disp('Testing TSAI method:')
% [Hcam2marker_, err] = TSAIleastSquareCalibration(Hmarker2world, Hgrid2cam);
% display(Hcam2marker_)
% display(err)

% disp('Testing Euclidean method:')
% [Hcam2marker_, err] = navy_calibration(Hmarker2world, Hgrid2cam);
% display(Hcam2marker_)
% display(err)
% 
% disp('Testing Dual Quaternion method:')
% [Hcam2marker_, err] = hand_eye_dual_quaternion(Hmarker2world, Hgrid2cam);
% display(Hcam2marker_)
% display(err)

display('Done Testing Hand Eye in Simulation')


