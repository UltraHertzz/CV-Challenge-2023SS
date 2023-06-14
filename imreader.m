function image = imreader(dir,extension)
% IMREADER 
%   读取图片数据并另存为.mat格式(load RGBD data and saving as (*.mat) format)
%   输入I  ：图片文件所在地址
%   输入II ：图片扩展名
    
    % Define the folder containing the images 
    imageFolder = 'dir';

    % Get a list of all image files in the folder
    imageFiles = dir(fullfile(imageFolder,'*.%s', extension));

    % load image file recursively
    for i = 1:numel(imageFiles)
        % Read the image
        imagePath = fullfile(imageFolder, imageFiles(i).name);
        image = imread(imagePath);

        % Perform image processing or feature detection if needed

        % Store 3D point and RGB value data in a structure
        pointCloudData = struct();
        pointCloudData.Points = [];
        pointCloudData.RGB = [];

        % Save the point cloud data to a .mat file
        [~, imageName, ~] = fileparts(imagePath);
        savePath = fullfile(imageFolder,[imageName, '.mat']);
        save(savePath, 'pointCloudData');

        % Display progress
        fprintf('Processed image %d of %d: %s\n', i ,numel(imageFiles), imageFiles(i).name);
    end
   
end

