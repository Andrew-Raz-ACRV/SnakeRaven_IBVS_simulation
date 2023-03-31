function [feature_error] = featuretrackercsv2struct(filename)
%Given a csv frile from the feature tracker, It converts it into
%structure containing the feature error over time

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
feature_error.seconds = csvread(filename,r,0,[r 0 f 0]); 
feature_error.nsec = csvread(filename,r,1,[r 1 f 1]);
%Feature Error Data
feature_error.Error = zeros(f,20);
for i=c:c+20
feature_error.Error(:,i) = csvread(filename,r,i,[r i f i]);
end
end