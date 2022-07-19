% SnakeBot Vision Processing
clc
clear all
close all

%% Read images 
files = {'2021-03-08-162432.jpg','2021-03-08-162503.jpg',...
    '2021-03-08-162530.jpg', '2021-03-08-162557.jpg'};

I1 = rgb2gray(imread(files{2}));
I2 = rgb2gray(imread(files{1}));

%% Test Feature Detection in Phantom
pts1 = {detectKAZEFeatures(I1), ...
    detectSURFFeatures(I1), ...
    detectBRISKFeatures(I1,'MinQuality',0.1,'MinContrast',0.1), ...
    detectFASTFeatures(I1)};
pts2 = {detectKAZEFeatures(I2), ...
    detectSURFFeatures(I2), ...
    detectBRISKFeatures(I1,'MinQuality',0.1,'MinContrast',0.1), ...
    detectFASTFeatures(I1)};

for ii = 1:4
    %% Detect Feature Points
    Points1 = pts1{ii};
    Points2 = pts2{ii};

    figure;
    imshow(I1);
    title('300 Strongest Feature Points from Image 1');
    hold on;
    plot(selectStrongest(Points1, 300));

    figure;
    imshow(I2);
    title('500 Strongest Feature Points from Image 2');
    hold on;
    plot(selectStrongest(Points2, 500));

    %% Extract Feature Descriptors

    [Features1, Points1] = extractFeatures(I1, Points1);
    [Features2, Points2] = extractFeatures(I2, Points2);

    %% Find Putative Point Matches

    Pairs = matchFeatures(Features1, Features2,'MatchThreshold',25);

    matchedPoints1 = Points1(Pairs(:, 1), :);
    matchedPoints2 = Points2(Pairs(:, 2), :);
    figure;
    showMatchedFeatures(I1, I2, matchedPoints1, ...
        matchedPoints2, 'montage');
    title('Putatively Matched Points (Including Outliers)');
    
    %% Locate the Object in the Scene Using Putative Matches
%     [tform, inlierIdx] = ...
%     estimateGeometricTransform2D(matchedPoints1, matchedPoints2, 'affine');
%         inlierPoints1 = matchedPoints1(inlierIdx, :);
%         inlierPoints2 = matchedPoints2(inlierIdx, :);
%         
%     figure;
%     showMatchedFeatures(I1, I2, inlierPoints1, ...
%     inlierPoints2, 'montage');
%     title('Matched Points (Inliers Only)');
end
