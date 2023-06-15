classdef Camera
    %Camera class
    %   相机类，通过Cam1 = Camera(camInfo) 构造
    %   返回相机宽    Cam1.WIDTH
    %   相机高        Cam1.HEIGHT, 
    %   相机内参矩阵  Cam1.IntrinMat
    properties
        WIDTH      
        HEIGHT
        IntrinMat
    end
    %% 
    methods
        function obj = Camera(camInfo)
            % IMREADERCLASS 构造此类的实例
            %   此处显示详细说
            obj.WIDTH = camInfo(1);
            obj.HEIGHT = camInfo(2);
            obj.IntrinMat = [camInfo(3),          0, camInfo(5);
                                      0, camInfo(4), camInfo(6);
                                      0,          0,          1];
        end
        
    end
end

