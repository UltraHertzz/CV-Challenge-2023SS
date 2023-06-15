

%% 主程序，现阶段用来调试函数(注释多行快捷键Ctrl+R)

%% reader test (**需要GUI进行路径的选择)

% imageReader
%image = imreader('D:\CV-Challenge\Dataset\delivery_area\images\dslr_images_undistorted');

% camera.txt reader
camInfo = cameraReader('D:\CV-Challenge\Dataset\delivery_area\dslr_calibration_undistorted\cameras.txt');
% create camera object 详情见CAMERA类构造说明
camera = Camera(camInfo);
%% FeatureDetect test (**需要GUI的trigger来进行一系列函数的调用)

%% Extrinsic Matrix test (需要每两张图计算一组T_hat,R, 保存在字典中，最后对于对应的点的世界坐标需要递归做-Ti，*Ri'的操作)

%% PointCloud merge test (变换到一个世界坐标系后进行点云的生成和加和)