function [correspondences_robust,largest_set_F] = F_ransac_jcp(correspondences,varargin)
    %输入参数
    inp = inputParser;
    default_epsilon = 0.5;
    inp.addOptional('epsilon',default_epsilon,@(x) isnumeric(x) && (x>=0) && (x<=1));
    default_p = 0.5;
    inp.addOptional('p',default_p,@(x) isnumeric(x) && (x>=0) && (x<=1));
    default_tolerance = 0.01;
    inp.addOptional('tolerance',default_tolerance,@(x) isnumeric(x));

    %解析输入
    parse(inp,varargin{:});

    %提取元素
    epsilon = inp.Results.epsilon;
    p = inp.Results.p;
    tolerance = inp.Results.tolerance;
    
    %初始化部分参数
    %所需点的数量
    num_k = 8;
    %迭代次数
    s = log(1-p)/log(1-epsilon^num_k);
    %记录迄今为止最大共识集中的对应点数量；
    largest_set_size = 0;
    %记录迄今为止最大共识集中的Sampson距离
    largest_set_dist = inf;
    %用于储存迄今为止找到的最大共识集中的的基础矩阵的缓冲区
    largest_set_F = zeros(3,3);
    
    correspondences_robust = [];
    i = 0;
    num_points = size(correspondences,2);
    
    while i < s
        %使用k个随机选择的对应点估计基础矩阵F
        sample_indices = randperm(num_points,num_k);
        sample_correspondences = correspondences(:,sample_indices);
        F = epa_jcp(sample_correspondences);

        %得到x1_pixel,x2_pixel的坐标位置
        x1_pixel = [sample_correspondences(1:2,:);ones(1,size(sample_correspondences,2))];
        x2_pixel = [sample_correspondences(3:4,:);ones(1,size(sample_correspondences,2))];

        %计算所有对应点的sampson距离
        sd = sampson_dist_jcp(F,x1_pixel,x2_pixel);

        %根据容差值确定共识集
        consensus_set_indices = find(sd<tolerance);
        consensus_set_size = length(consensus_set_indices);

        %计算包含点的数量和绝对Sampson距离
        included_points = sample_correspondences(:,consensus_set_indices);
        num_included_points = size(included_points,2);
        abs_sd = sum(sd(consensus_set_indices));

        %根据共识集的大小和Sampson距离更新最大集合
        if num_included_points > largest_set_size ||(num_included_points == largest_set_size && abs_sd <largest_set_dist)
            largest_set_size = num_included_points;
            largest_set_dist = abs_sd;
            largest_set_F = F;
            correspondences_robust = included_points;
        end

        %更新迭代次数和最大集合信息
        i = i+1;
    end


end

