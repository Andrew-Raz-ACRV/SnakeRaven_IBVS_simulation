function [NDI_data] = NDIcsv2struct(filename,plotting)
%Given a csv file from the NDI combined sample, Convert the data into a
%structure. Assumes the data is consistently being read with no missing
%except during the beginning
%[NDI_data] = NDIcsv2struct(filename,plotting)

%% Find the start and end of the file:
fid = fopen(filename);
for ii = 1:5
    fgetl(fid); %beginning part
end
searching_start = true;
searching_end = true; %the data part
samples = 0; start = 0;
while(searching_end)
    line_ex = fgetl(fid);
    samples = samples +1;
    %Check if its start:
    if (isempty(strfind(line_ex,'0, 0, 0, 0, 0, 0, 0,')) && searching_start)
        %Found start sample:
        start = samples;
        searching_start = false;
    end
    %Check if its the end
    if line_ex==-1
        searching_end = false;
        samples = samples - 2;
    end
end
fclose(fid);

%% Extract data from csv:

%Todo: filter out the lost data 0s manually adjust start row
r = 4 + start; %first row
f = 4 + samples; %Last row of transform data
c = 5; %start column of transform data
NDI.Test_date = csvread(filename,2,0,[2 0 2 6]);
NDI.sample = csvread(filename,r,0,[r 0 f 0]); 
NDI.time = csvread(filename,r,1,[r 1 f 4]);
%A
NDI.A.x = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.A.y = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.A.z = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.A.qo = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.A.qx = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.A.qy = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.A.qz = csvread(filename,r,c,[r c f c]); c=c+1;
%B
NDI.B.x = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.B.y = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.B.z = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.B.qo = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.B.qx = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.B.qy = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.B.qz = csvread(filename,r,c,[r c f c]); c=c+1;
%C
NDI.C.x = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.C.y = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.C.z = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.C.qo = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.C.qx = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.C.qy = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.C.qz = csvread(filename,r,c,[r c f c]); c=c+1;
%D
NDI.D.x = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.D.y = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.D.z = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.D.qo = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.D.qx = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.D.qy = csvread(filename,r,c,[r c f c]); c=c+1;
NDI.D.qz = csvread(filename,r,c,[r c f c]); c=c+1;

%% Create Transformation matrices
%Initialisation
NDI.N = samples-r+5; %number of valid samples
NDI.A.T = zeros(4,4,NDI.N);
NDI.B.T = zeros(4,4,NDI.N);
NDI.C.T = zeros(4,4,NDI.N);
NDI.D.T = zeros(4,4,NDI.N);

for ii=1:NDI.N
    %Rotation
    Ra = quat2rotm([NDI.A.qo(ii) NDI.A.qx(ii) NDI.A.qy(ii) NDI.A.qz(ii)]);
    Rb = quat2rotm([NDI.B.qo(ii) NDI.B.qx(ii) NDI.B.qy(ii) NDI.B.qz(ii)]);
    Rc = quat2rotm([NDI.C.qo(ii) NDI.C.qx(ii) NDI.C.qy(ii) NDI.C.qz(ii)]);
    Rd = quat2rotm([NDI.D.qo(ii) NDI.D.qx(ii) NDI.D.qy(ii) NDI.D.qz(ii)]);
    %Translation
    ta = [NDI.A.x(ii) NDI.A.y(ii) NDI.A.z(ii)]';
    tb = [NDI.B.x(ii) NDI.B.y(ii) NDI.B.z(ii)]';
    tc = [NDI.C.x(ii) NDI.C.y(ii) NDI.C.z(ii)]';
    td = [NDI.D.x(ii) NDI.D.y(ii) NDI.D.z(ii)]';
    %Put together
    NDI.A.T(:,:,ii) = [Ra, ta; 0 0 0 1];
    NDI.B.T(:,:,ii) = [Rb, tb; 0 0 0 1];
    NDI.C.T(:,:,ii) = [Rc, tc; 0 0 0 1];
    NDI.D.T(:,:,ii) = [Rd, td; 0 0 0 1];
end

%% Extra Plot Trajectory of Sensor A relative to NDI frame
NDI_data = NDI;

if (plotting)
    figure;
    %Plot Trajectory
    % plot3(NDI_data.A.x,NDI_data.A.y,NDI_data.A.z,'bo'), hold on
    % plot3(NDI_data.B.x,NDI_data.B.y,NDI_data.B.z,'ro'), hold on
    % plot3(NDI_data.C.x,NDI_data.C.y,NDI_data.C.z,'go'), hold on
    % plot3(NDI_data.D.x,NDI_data.D.y,NDI_data.D.z,'mo'), hold on
    for ii=1:NDI_data.N
        %Plot the sensor frames
        plotcoord3(NDI_data.A.T(:,:,ii),5,'r','g','b')
        plotcoord3(NDI_data.B.T(:,:,ii),5,'r','g','b')
        plotcoord3(NDI_data.C.T(:,:,ii),5,'r','g','b')
        plotcoord3(NDI_data.D.T(:,:,ii),5,'r','g','b')
    end
    text(NDI_data.A.T(1,4,1)+1,NDI_data.A.T(2,4,1)+1,NDI_data.A.T(3,4,1)+1,'A')
    text(NDI_data.B.T(1,4,1)+1,NDI_data.B.T(2,4,1)+1,NDI_data.B.T(3,4,1)+1,'B')
    text(NDI_data.C.T(1,4,1)+1,NDI_data.C.T(2,4,1)+1,NDI_data.C.T(3,4,1)+1,'C')
    text(NDI_data.D.T(1,4,1),NDI_data.D.T(2,4,1),NDI_data.D.T(3,4,1),'D')

    %Plot coordinate frame 100 mm axis length
    % plotcoord3(eye(4),100,'r','g','b')
    % text(0,0,0,'NDI')
    %Figure style
    title('3D View (X-Y-Z) of NDI tracker and Sensors'), xlabel('X displacement')
    ylabel('Y displacement'), zlabel('Z displacement')
    view(3)
    axis equal
    grid on
end

end