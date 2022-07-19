%Kinova ARM
clear all,
close all,
clc

%*****INITIAISATION of Robot Environment********%

%Link lengths
D1 = 0.2755; D2 = 0.2050; D3 = 0.2050; D4 = 0.2073; D5 = 0.1038;
D6 = 0.1038; D7 = 0.1600; e2 = 0.0098;

%Inertial Parameters:
m0 = 0.64; m1 = 0.6; m2 = 0.57; m3 = 0.6; m4 = 0.37; m5 = 0.37; m6 = 0.37;
m7 = 0.37; mH = 0.68;

%Joint limits: limits_i = [min_deg, max_deg];
limits_1 = [-10000, 10000]; limits_2 = [47, 313];
limits_3 = [-10000, 10000]; limits_4 = [30, 330];
limits_5 = [-10000, 10000]; limits_6 = [65, 295];
limits_7 = [-10000, 10000];

%Robot arm model
L1 = Link('d', -D1, 'a', 0, 'alpha', pi/2, 'qlim', limits_1, 'm', m1, 'offset', pi);
L2 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', limits_2, 'm', m2);
L3 = Link('d', -(D2+D3), 'a', 0, 'alpha', pi/2, 'qlim', limits_3, 'm', m3);
L4 = Link('d', -e2, 'a', 0, 'alpha', pi/2, 'qlim', limits_4, 'm', m4);
L5 = Link('d', -(D4+D5), 'a', 0, 'alpha', pi/2, 'qlim', limits_5, 'm', m5);
L6 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', limits_6, 'm', m6);
L7 = Link('d', -(D6+D7), 'a', 0, 'alpha', 0, 'qlim', limits_7, 'm', m7, 'offset', pi/2);

%World to Base of the Robot:
Twb = transl(0, 0, 0) * trotx(pi);
bot = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'Kinova Arm','base',Twb);

%Calibration Target:
Twt = transl(1,1,1) * trotx(pi/2);
P = mkgrid(3, 0.2, 'pose', Twt);
%P = mkcube(0.2,'pose',Tcali);

%Hand to Eye answer:
Tgc = transl(0.04, 0.05, 0.05) * trotx(pi); %Gripper to Camera

%Initialise the Eye
cam = CentralCamera('focal', 0.015, 'pixel', 10e-6, ...
'resolution', [1280 1024], 'centre', [640 512], 'name', 'Eye');


%*********MOVEMENTS********%
%Series of Joint Angles for HAND EYE
q = [deg2rad([180 180 0 100 90 120 0]);
    deg2rad([180 180 0 100 90 120 30]);
    deg2rad([180 180 0 100 90 120 40]);
    deg2rad([180 180 0 100 90 120 50]);
    deg2rad([180 180 0 95  90 120 50]);
    deg2rad([180 180 0 105 90 120 50]);
    deg2rad([180 180 0 110 90 120 50]);
    deg2rad([180 180 0 115 90 120 50]);
    deg2rad([180 180 0 120 90 120 50]);
    deg2rad([180 180 0 125 90 120 0]);
    deg2rad([180 175 0 100 90 120 50]);
    deg2rad([180 170 0 100 90 120 50]);
    deg2rad([180 165 0 100 90 120 50]);
    deg2rad([180 160 0 100 90 120 50]);
    deg2rad([180 160 0 100 90 125 50]);
    deg2rad([180 180 0 100 90 115 50]); %16
    deg2rad([180 180 0 100 80 115 50]); %17
    deg2rad([185 180 0 100 90 120 50]); %18
    deg2rad([190 180 0 100 90 120 50]); %19
    deg2rad([190 180 0 100 80 120 50]); %20
    deg2rad([190 180 0 100 80 125 50]); %21
    deg2rad([200 180 0 100 80 85 50]); %22
    deg2rad([205 180 0 100 80 95 50]); %23
    deg2rad([200 180 30 100 80 85 50]); %24
    deg2rad([190 180 50 100 80 75 50]); %25
    deg2rad([190 180 45 100 80 75 50]); %26
    deg2rad([190 180 40 100 80 75 50]); %27
    deg2rad([190 180 35 100 80 75 50]); %28
    deg2rad([190 180 25 100 80 75 50]); %29
    deg2rad([190 180 20 100 80 75 50])]; %30

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

%********HAND EYE CALIBRATION DATA*******%

for ii = 1:Number_of_Views

%Ai - World to gripper Transform - Forward Kinematics
Twg = bot.fkine(q(ii,:)); 
Twg = [Twg.n, Twg.o, Twg.a, Twg.t; 0, 0, 0, 1];

%Defining the transformation of the robot hand / marker
%to the robot base / external tracking device
Hmarker2world(:,:,ii) = Twg;


%Bi - Camera to Target Transform
Twc = Twg*Tgc;
Cam = cam.move(Twc);
Tct = Twc\Twt;

%Defining the transformation of the grid to the camera
Hgrid2cam(:,:,ii) = Tct;

%Plot Environment once

    figure(1)
    bot.plot(q(ii,:)); %Plot Robot
    hold on
    plot_sphere(P, 0.03, 'r'); %Plot Calibration Target:
    hold on
    Cam.plot_camera; %Plot eye
    axis equal
    Cam.plot(P); %plot what the eye sees in image plane

    disp('running')
    disp(ii)
    pause()

end

disp('done movements')

%********HAND EYE CALIBRATION*******%

disp('Ground truth:')
display(Tgc)

disp('Testing Hourads quaternion method:')
[Hcam2marker_, err] = inria_calibration(Hmarker2world, Hgrid2cam);
display(Hcam2marker_)
display(err)

disp('Testing TSAI method:')
[Hcam2marker_, err] = TSAIleastSquareCalibration(Hmarker2world, Hgrid2cam);
display(Hcam2marker_)
display(err)

disp('Testing Euclidean method:')
[Hcam2marker_, err] = navy_calibration(Hmarker2world, Hgrid2cam);
display(Hcam2marker_)
display(err)

disp('Testing Dual Quaternion method:')
[Hcam2marker_, err] = hand_eye_dual_quaternion(Hmarker2world, Hgrid2cam);
display(Hcam2marker_)
display(err)

display('Done Testing Hand Eye in Simulation')


