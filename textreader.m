function [varargin] = textreader(fromTxt,varargin)
%TEXTREADER 此处显示有关此函数的摘要
%   此处显示详细说明
    if fromTxt
        if exist('cameras.txt','file') == 0
            error(disp('Could not found "cameras.txt",please rename your camera property files'))
        end
        varargin = importdata("cameras.txt");
    end
end


