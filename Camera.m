classdef Camera
    %Camera class
    %   此处显示详细说明
    properties
        CAMERA_ID
        MODEL
        WIDTH
        HEIGHT
        PARAMS
    end
    %% 
    methods
        function obj = Camera(CAMERA_ID,MODEL,WIDTH,HEIGHT,varargin)
            %IMREADERCLASS 构造此类的实例
            %   此处显示详细说明
            obj.CAMERA_ID = CAMERA_ID;
            obj.MODEL = MODEL;
            obj.WIDTH = WIDTH;
            obj.HEIGHT = HEIGHT;
            obj.PARAMS = varargin;
        end
        
        function camIntrincMat = intrix(~)
            % Camera Intrinsic Matrix
            %   获取相机内参 (get camera intrinsic matrix)
            if obj.MODEL == "PINHOLE"
                f_x,f_y,c_x,c_y = varargin;
                camIntrincMat = [f_x,0,c_x;0,f_y,c_y;0,0,1];
            end
        end
    end
end

