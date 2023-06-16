

%% 主程序，现阶段用来调试函数(注释多行快捷键Ctrl+R)

%% reader test (**需要GUI进行路径的选择)

% imageReader 记得替换成自己的dataset所在的目录，记得把我的目录注释掉
image = imreader('D:\CV-Challenge\Dataset\delivery_area\images\dslr_images_undistorted');

% camera.txt reader
camInfo = cameraReader('D:\CV-Challenge\Dataset\delivery_area\dslr_calibration_undistorted\cameras.txt');
% create camera object 详情见CAMERA类构造说明
camera = Camera(camInfo);
%% 图片显示
imshow(image{1});
%% FeatureDetect test (**需要GUI的trigger来启动一系列函数的调用)
% features = detectHarrisFeatures(rgb2gray(image{1})); %单一图片的HarrisFeatures
correspondences = {};
E = {};
for i = 1:numel(image)-1

    [m1,m2] = featureDetection(image{i},image{i+1}); 
    correspondences{i} = cat(1,m1.Location',m2.Location');
    E{i} = epa(correspondences{i},camera.IntrinMat);
% corr_robust = F_ransac(correspondences); ### 输出有问题
% 8P alg.
end
%% Extrinsic Matrix test (需要每两张图计算一组T_hat,R, 保存在字典中，最后对于对应的点的世界坐标需要递归做-Ti，*Ri'的操作)

%% PointCloud merge test (变换到一个世界坐标系后进行点云的生成和加和)