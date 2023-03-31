%% Inputs to fminsearch
options = optimset('Display','iter','PlotFcns',@optimplotfval);
%x0 = [deg2rad(90) deg2rad(-60) deg2rad(25)];
%x0 = [deg2rad(0) deg2rad(90) deg2rad(-30)]; %0.0002    1.6032   -0.5144
%x0 = [deg2rad(-85) deg2rad(120) deg2rad(0)]; %VPC A
x0 = [-1.29043267827014,1.93415347464714,0.000859331748205085]; %VPC D
fun = @Offset_cameraNDI_calibration;

%% Run fminsearch

[x,fval,exitflag,output] = fminsearch(fun,x0,options)
