function preprocessedImage = preprocessImage_jcp(image)
% preprocessImage 图像预处理函数（包含去噪和对比度增强）
%   输入 RGB 图像，输出预处理后的图像

    % 转换为灰度图像
    grayImage = rgb2gray(image);
    
    % 去噪处理（使用中值滤波器）
    denoisedImage = medfilt2(grayImage);
    
    % 对比度增强（使用直方图均衡化）
    enhancedImage = histeq(denoisedImage);
    
    % 转换回 RGB 图像
    preprocessedImage = cat(3, enhancedImage, enhancedImage, enhancedImage);
end