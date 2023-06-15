function features =  Harris_detector(input_image, varargin)


%哈里斯角点检测
%segment_length : 控制图片中心 所检测图片部分大小
%k： 角点阈值
%tau： 响应值去噪阈值
%title_size:  tile 特征窗口
%do_plot: 绘图
%min_dist: 检测点周围空白阈值
%N：每个tile里面 最多特征数量
%输入参数
p = inputParser;
defaultsegment_length = 15;
defaultk = 0.05;
defaulttau = 10^6;
defaultmin_dist = 20;
defaulttile_size = [200,200];
default_N = 5;
defaultdo_plot = false;


    Fcn_1 = @(x)isnumeric(x) && (x>1) && mod(x,2) && isscalar(x);
    Fcn_2 = @(x)isnumeric(x) && isscalar(x) && (x>=0) && (x<=1);
    Fcn_3 = @(x)isnumeric(x) && isscalar(x) && (x>0);
    Fcn_4 = @(x)islogical(x);
    Fcn_5 = @(x)isnumeric(x) && (x>1);
    Fcn_6 = @(x)isnumeric(x);
    Fcn_7 = @(x)isnumeric(x) && (x>1);
    
    p.addRequired('input_image');
    p.addOptional('segment_length',defaultsegment_length,Fcn_1);
    p.addOptional('k',defaultk,Fcn_2);
    p.addOptional('tau',defaulttau,Fcn_3);
    p.addOptional('do_plot',defaultdo_plot,Fcn_4);
    p.addOptional('min_dist',defaultmin_dist,Fcn_5);
    p.addOptional('tile_size',defaulttile_size,Fcn_6);
    p.addOptional('N',default_N,Fcn_7);
    p.parse(input_image,varargin{:});
    
    if size(p.Results.tile_size)==1
      Result_tile_size = [p.Results.tile_size,p.Results.tile_size];
    else
      Result_tile_size = p.Results.tile_size; 
    end
    
    min_dist = p.Results.min_dist; %设定 两个feature之间的下限值 去除噪声影响
    N = p.Results.N;
    tile_size = Result_tile_size;
    segment_length = p.Results.segment_length;
    k = p.Results.k;
    tau = p.Results.tau;
    do_plot = p.Results.do_plot; 
    
    if size(input_image,3) ~= 1
        error( "Image format has to be NxMx1");
    else
        gray = double(input_image);     
    end


    % Approximation of the image gradient 
    [Ix,Iy]=sobel_xy(gray);
    % Weighting
    w = fspecial('gaussian',[1,segment_length],segment_length/5);
    % Harris Matrix G
    G11 = conv2(w,w,Ix.^2,'same'); %分别对哈里斯矩阵的元素进行 加权加强中心元素的作用 使用高斯Filter
    G22 = conv2(w,w,Iy.^2,'same');
    G12 = conv2(w,w,Iy.*Ix,'same');
    
    %Harris矩阵计算 其响应值
     H = ((G11.*G22 - G12.^2) - k*(G11 + G22).^2);
    Feat = zeros(size(H));
    Feat((ceil(segment_length/2)+1):(size(H,1)-ceil(segment_length/2)),...
         (ceil(segment_length/2)+1):(size(H,2)-ceil(segment_length/2)))=1;  %将梯度矩阵的边缘置零

    R = H.*Feat;
    R(R<tau)=0;
%     [row,col] = find(R); 
    corners = R;
%     features = [col,row]'; 
    
    

    corner_ex = zeros(size(corners)+2*min_dist);
    corner_ex(min_dist+1:min_dist+size(corners,1),min_dist+1:min_dist+size(corners,2)) = corners;
    corners = corner_ex;
    [sorted, sorted_index] = sort(corners(:),'descend');
    sorted_index(sorted==0) = [];
%prepation
acc_array = zeros(ceil(size(input_image,1)/tile_size(1)),ceil(size(input_image,2)/tile_size(2)));
features =  zeros(2,min(numel(acc_array)*N,numel(sorted_index)));

Cake = cake(min_dist);  %调用cake函数要额外写一个
count = 1;
    for i = 1:numel(sorted_index)
        current = sorted_index(i);
        if corners(current) == 0
            continue;
        else 
            x_corners = floor(current/size(corners,1));     % 下标转换为行列号
            y_corners = current-x_corners*size(corners,1);
            x_corners = x_corners+1;
        end
            x_acc_array = ceil((x_corners-min_dist)/tile_size(2));
            y_acc_array = ceil((y_corners-min_dist)/tile_size(1));

            corners(y_corners-min_dist:y_corners+min_dist,x_corners-min_dist:x_corners+min_dist) = corners(y_corners-min_dist:y_corners+min_dist,x_corners-min_dist:x_corners+min_dist).*Cake;
           
        if acc_array(y_acc_array,x_acc_array) < N
            acc_array(y_acc_array,x_acc_array) = acc_array(y_acc_array,x_acc_array)+1; %对应位置0+1
            features(:,count) = [x_corners-min_dist;y_corners-min_dist]; %dont know why -min_dist
            count = count+1;
        end
    end
    features(:,all(features==0,1)) = []; %删除全为0的一列
    
    if p.Results.do_plot == true
        figure;
        imshow(input_image);
        hold on
  
        plot(features(1,:),features(2,:),'o')
    end
end

