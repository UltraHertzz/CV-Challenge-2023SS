

%% 主程序，现阶段用来调试函数(注释多行快捷键Ctrl+R)

%% reader test (**需要GUI进行路径的选择)

% imageReader 替换成自己的dataset所在的目录，记得把我的目录注释掉
image = imreader('E:/cv-challenge/Dataset/delivery_area_dslr_undistorted/delivery_area/images/dslr_images_undistorted');
% camera.txt reader
camInfo = cameraReader('E:/cv-challenge/Dataset/delivery_area_dslr_undistorted/delivery_area/dslr_calibration_undistorted/cameras.txt');
% create camera object 详情见CAMERA类构造说明
camera = Camera(camInfo);

%% 图片显示 
imshow(image{1});
%% FeatureDetect test(jcp)
% imageReader 替换成自己的dataset所在的目录，记得把我的目录注释掉

image1 = preprocessImage_jcp(image{1});
image2 = preprocessImage_jcp(image{2});

featureDetector = "SIFT";
% 寻找关联点
[matchedPoints1, matchedPoints2] = findCorrespondingPoints(image1, image2,featureDetector);
k = camera.IntrinMat;
correspondences = cat(1,matchedPoints1.Location',matchedPoints2.Location');
correspondences_robust = F_ransac_jcp(correspondences, 'tolerance', 0.04);
%可视化测试
figure;
imshow(image1);
hold on;
h = imshow(image2);
set(h,'AlphaData',0.5);
%标记对应的图像点
for i = 1:size(correspondences_robust,2)
    x1 = correspondences_robust(1,i);
    y1 = correspondences_robust(2,i);
    x2 = correspondences_robust(3,i);
    y2 = correspondences_robust(4,i);

    %随机选两种不同颜色来标记对应的图像点
    color1 = rand(1,3);
    color2 = rand(1,3);

    %在图1上标记对应的点
    plot(x1,y1,'o','MarkerFaceColor',color1,'MarkerSize',8);
    %在图2上标记对应的点
    plot(x2,y2,'o','MarkerFaceColor',color2,'MarkerSize',8);
    %连接对应的点
    line([x1,x2],[y1,y2],'Color','cyan');
end
hold off;

E = epa_jcp(correspondences_robust,camera.IntrinMat);
%% 测试reconstruction(jcp)
[T1, R1, T2, R2, U, V] = TR_from_E(E);
[T,R,lambda,P1] = reconstruction_jcp(T1,T2,R1,R2,correspondences,k);
[repo_error,x2_repro] = backprojection_jcp(correspondences,P1,image2,T,R,k);

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