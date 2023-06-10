function y = imreader(dir,TUMDataSet)
%IMREADER 读取图片数据(.csv)并另存为.mat格式(load RGBD data(*.csv) and saving as (*.mat) format)
%   此处显示详细说明
    if TUMDataSet
        baseDownloadURL = 'https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz';
        dataFolder = fullfile(tempdir,'tum_rgbt_dataset',filesep);
        options = weboptions('Timeout',inf);
        tgzFileName = [dataFolder,'fr3_office.tgz'];
        folderExists = exist(dataFolder,dir);

        y = 1;
    end
end

