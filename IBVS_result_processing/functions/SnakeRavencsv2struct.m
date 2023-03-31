function [SRdata] = SnakeRavencsv2struct(filename)
%Given a csv frile from SnakeRaven Controller, It converts it into
%structure containing the target and forward kinematic trajectory

%% Find the end of the file:
fid = fopen(filename);
fgetl(fid); %title
searching_end = true; %the data part
samples = 0; 
while(searching_end)
    line_ex = fgetl(fid);
    samples = samples +1;
    %Check if its the end
    if line_ex==-1
        searching_end = false;
        samples = samples - 1;
    end
end
fclose(fid);

%% Now extract data:

r = 1; %first row
f = samples; %Last row of transform data
c = 2; %start column of transform data
SRdata.seconds = csvread(filename,r,0,[r 0 f 0]); 
SRdata.nsec = csvread(filename,r,1,[r 1 f 1]);
%Transform Data
Tend1_1 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar1_1 = csvread(filename,r,c,[r c f c]); c = c + 1;
Tend2_1 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar2_1 = csvread(filename,r,c,[r c f c]); c = c + 1;
Tend3_1 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar3_1 = csvread(filename,r,c,[r c f c]); c = c + 1;

Tend1_2 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar1_2 = csvread(filename,r,c,[r c f c]); c = c + 1;
Tend2_2 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar2_2 = csvread(filename,r,c,[r c f c]); c = c + 1;
Tend3_2 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar3_2 = csvread(filename,r,c,[r c f c]); c = c + 1;

Tend1_3 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar1_3 = csvread(filename,r,c,[r c f c]); c = c + 1;
Tend2_3 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar2_3 = csvread(filename,r,c,[r c f c]); c = c + 1;
Tend3_3 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar3_3 = csvread(filename,r,c,[r c f c]); c = c + 1;

Tend1_4 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar1_4 = csvread(filename,r,c,[r c f c]); c = c + 1;
Tend2_4 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar2_4 = csvread(filename,r,c,[r c f c]); c = c + 1;
Tend3_4 = csvread(filename,r,c,[r c f c]); c = c + 1;
Ttar3_4 = csvread(filename,r,c,[r c f c]);

%% Convert into Transformation matrices
SRdata.N = samples; %number of valid samples
SRdata.Tend = zeros(4,4,samples);
SRdata.Ttarget = zeros(4,4,samples);
SRdata.x_end = zeros(samples,1);
SRdata.y_end = zeros(samples,1);
SRdata.z_end = zeros(samples,1);
SRdata.x_tar = zeros(samples,1);
SRdata.y_tar = zeros(samples,1);
SRdata.z_tar = zeros(samples,1);

for ii=1:samples
    SRdata.Tend(:,:,ii) = [Tend1_1(ii) Tend1_2(ii) Tend1_3(ii) Tend1_4(ii);
                           Tend2_1(ii) Tend2_2(ii) Tend2_3(ii) Tend2_4(ii);
                           Tend3_1(ii) Tend3_2(ii) Tend3_3(ii) Tend3_4(ii);
                           0    0   0   1];
    SRdata.x_end(ii) = Tend1_4(ii); 
    SRdata.y_end(ii) = Tend2_4(ii); 
    SRdata.z_end(ii) = Tend3_4(ii); 
    SRdata.Ttarget(:,:,ii) = [Ttar1_1(ii) Ttar1_2(ii) Ttar1_3(ii) Ttar1_4(ii);
                           Ttar2_1(ii) Ttar2_2(ii) Ttar2_3(ii) Ttar2_4(ii);
                           Ttar3_1(ii) Ttar3_2(ii) Ttar3_3(ii) Ttar3_4(ii);
                           0    0   0   1];                       
    SRdata.x_tar(ii) = Ttar1_4(ii); 
    SRdata.y_tar(ii) = Ttar2_4(ii); 
    SRdata.z_tar(ii) = Ttar3_4(ii);       
end

end