function [matchedPoints1,matchedPoints2] = featureDetection(image1,image2)
%FEATUREDETECTION 关联特征点
%   此处显示详细说明
    
    I1 = im2gray(image1);
    I2 = im2gray(image2);
    
    % find corner (寻找特征点)
    points1 = detectHarrisFeatures(im2gray(I1));
    points2 = detectHarrisFeatures(im2gray(I2));

    % extract features 提取特征
    [features1, valid_points1] = extractFeatures(I1,points1);
    [features2, valid_points2] = extractFeatures(I2,points2);
    
    % match features 匹配特征
    indexPairs = matchFeatures(features1,features2);

    % retrieve the locations of corr points for each image
    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);

%   %% Visualization
    figure;
    showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2);
end

