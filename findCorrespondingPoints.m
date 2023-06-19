function [matchedPoints1, matchedPoints2] = findCorrespondingPoints(image1, image2, featureDetector)
% findCorrespondingPoints 寻找两张RGB图像的关联点
%   输入两张RGB图像和特征点检测器类型，输出对应的关联点坐标

    % 转换为灰度图像
    grayImage1 = rgb2gray(image1);
    grayImage2 = rgb2gray(image2);

    % 检查特征点检测器类型
    if nargin < 3 || isempty(featureDetector)
        featureDetector = 'Harris'; % 默认使用 Harris 特征点检测器
    end
    
    % 特征点检测
    switch featureDetector
        case 'Harris'
            points1 = detectHarrisFeatures(grayImage1);
            points2 = detectHarrisFeatures(grayImage2);
        case 'SURF'
            points1 = detectSURFFeatures(grayImage1);
            points2 = detectSURFFeatures(grayImage2);
        case 'SIFT'
            points1 = detectSIFTFeatures(grayImage1);
            points2 = detectSIFTFeatures(grayImage2);
        case 'ORB'
            points1 = detectORBFeatures(grayImage1);
            points2 = detectORBFeatures(grayImage2);
        otherwise
            error('不支持的特征点检测器类型');
    end

    % 提取特征描述子
    [features1, validPoints1] = extractFeatures(grayImage1, points1);
    [features2, validPoints2] = extractFeatures(grayImage2, points2);
    
    % 特征点匹配
    indexPairs = matchFeatures(features1, features2);

    % 设置Metric的阈值
    metricThreshold = 2e-4; % 这里假设阈值为0.5，你可以根据实际需求进行调整

    % 获取匹配点的坐标和Metric值
    matchedPoints1 = validPoints1(indexPairs(:, 1));
    matchedPoints2 = validPoints2(indexPairs(:, 2));

    % 根据Metric的阈值筛选特征点
    metrics = [matchedPoints1.Metric]; % 获取所有特征点的度量值
    validIndices = metrics >= metricThreshold;
    matchedPoints1 = matchedPoints1(validIndices);
    matchedPoints2 = matchedPoints2(validIndices);


    % 显示关联点
    %figure;
    %ax = axes;
    %showMatchedFeatures(image1, image2, matchedPoints1, matchedPoints2, 'Parent', ax);
    %title('关联点匹配');
    %legend(ax, '图像1', '图像2');
end


