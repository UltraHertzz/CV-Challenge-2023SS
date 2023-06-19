function [T,R,lambda,P1] = reconstruction_jcp(T1,T2,R1,R2,correspondences,k)
    %%获取点并初始化参数
    %获取对应的图像点
    x1_1 = zeros(2,size(correspondences,2));
    x2_1 = zeros(2,size(correspondences,2));
    x1_1 = correspondences(1:2,:);
    x2_1 = correspondences(3:4,:);

    %转换为齐次坐标系
    x1_homo = [x1_1;ones(1,size(x1_1,2))];
    x2_homo = [x1_1;ones(1,size(x1_1,2))];

    %校准坐标
    x1_calibrated = k\x1_homo;
    x2_calibrated = k\x2_homo;
    x1 = x1_calibrated;
    x2 = x2_calibrated;

    %初始化结果变量
    T_cell = cell(1,4);
    R_cell = cell(1,4);
    d_cell = cell(1,4);
    d1 = zeros(size(x1,2),2);

    for i = 1:4
        if mod(i,2) == 0 
            T_cell{1,i} = T2;
        else
            T_cell{1,i} = T1;
        end
    end
    for i = 1:4
        if i<3
            R_cell{1,i} = R1;
        else
            R_cell{1,i} = R2;
        end
    end
    for i =1:4
        d_cell{1,i} = d1;
    end

    %% Reconstruction
    num_combinations = length(T_cell);
    num_x = size(x1,2);
    M1_3 = cell(1,num_combinations);
    M2_3 = cell(1,num_combinations);

    %遍历 all combinations of T_cell{i}和R_cell{i}
    for i = 1:num_combinations
        M1 = zeros(3*num_x,num_x + 1);
        M2 = zeros(3*num_x,num_x + 1);
        for j = 1:num_x
            %计算x2_hat
            x2_hat = zeros(3);
            x2_hat(1,2) = -x2(3,j);
            x2_hat(1,3) = x2(2,j);
            x2_hat(2,1) = x2(3,j);
            x2_hat(2,3) = -x2(1,j);
            x2_hat(3,1) = -x2(2,j);
            x2_hat(3,2) = x2(1,j);

            %计算x1_hat
            x1_hat = zeros(3);
            x1_hat(1,2) = -x1(3,j);
            x1_hat(1,3) = x1(2,j);
            x1_hat(2,1) = x1(3,j);
            x1_hat(2,3) = -x1(1,j);
            x1_hat(3,1) = -x1(2,j);
            x1_hat(3,2) = x1(1,j);

            M1(3*j-2:3*j,j) = x2_hat*R_cell{1,i}*x1(:,j);
            M1(3*j-2:3*j,num_x + 1) = x2_hat*T_cell{1,i};
            M2(3*j-2:3*j,j) = x1_hat*(R_cell{1,i})'*x2(:,j);
            M2(3*j-2:3*j,num_x + 1) = -x1_hat*(R_cell{1,i})'*T_cell{1,i};
        end

        M1_3{1,i} = M1;
        M2_3{1,i} = M2;

        %利用SVD解出Md=b
        [~,~,V1] = svd(M1_3{1,i});
        [~,~,V2] = svd(M1_3{1,i});
        d1 = V1(:,end);
        d2 = V2(:,end);

        %归一化向量d1和d2
        if d1(end)~=0
            d1 = d1/d1(end);
        else
            d1(end) = 1;
        end
        if d2(end)~=0
            d2 = d2/d2(end);
        else
            d2(end) = 1;
        end

        %储存depth信息
        d_cell{1,i} = [d1(1:end-1) d2(1:end-1)];
    end

    %找出最大d_cell
    best_index = 0;
    max_postive_entries = 0;
    for i = 1:num_combinations
        postive_entries = sum(d_cell{1,i} > 0, 'all');
        if postive_entries > max_postive_entries
            best_index = i;
            max_postive_entries = postive_entries;
        end
    end

    %得到最好的R,T和lambda值
    R = R_cell{1,best_index};
    T = T_cell{1,best_index};
    lambda = d_cell{1,best_index};
    %下面的M1，M2可以删去，没什么实际用处
    M1 = M1_3{1,4};
    M2 = M2_3{1,4};
    %% 计算和可视化3D坐标
    %相机坐标系1的坐标
    camC1 = [-0.2 0.2 0.2 -0.2; 0.2 0.2 -0.2 -0.2;1 1 1 1];
    camC2 = R'*camC1-R'*T;

    %使用lambda和k计算图像点P1的世界坐标
    %转化成齐次坐标形式
    correspondences_homogeneous = [correspondences(1:2,1:end);ones(1,size(correspondences,2))];

    %初始化P1_homogeneous矩阵
    P1_homogeneous = zeros(size(correspondences_homogeneous));

    %逐点计算相机坐标系1中图像点的世界坐标
    for i = 1:size(correspondences_homogeneous,2)
        P1_homogeneous(:,i) = lambda(i,1)*correspondences_homogeneous(:,i);
    end

    P1 = zeros(size(correspondences_homogeneous));

    %使用相机矩阵K进行投影

    for i = 1:size(correspondences_homogeneous,2)
        P1(:,i) = k\P1_homogeneous(:,i);
    end

    figure;
    scatter3(P1(1,:),P1(2,:),P1(3,:),'filled');
    text(P1(1,:),P1(2,:),P1(3,:),num2str((1:size(P1,2))'),'FontSize',8,'Color','k');
    hold on;

    %绘制相机坐标系1和2
    plot3(camC1(1,:),camC1(2,:),camC1(3,:),'b','LineWidth',2);
    plot3(camC1(1,[1 2]),camC1(2,[1,2]),camC1(3,[1,2]),'b','LineWidth',2);
    plot3(camC1(1,[2 3]),camC1(2,[2,3]),camC1(3,[2,3]),'b','LineWidth',2);
    plot3(camC1(1,[3 4]),camC1(2,[3,4]),camC1(3,[3,4]),'b','LineWidth',2);
    plot3(camC1(1,[4 1]),camC1(2,[4,1]),camC1(3,[4,1]),'b','LineWidth',2);

    plot3(camC2(1,:),camC2(2,:),camC2(3,:),'r','LineWidth',2);
    plot3(camC2(1,[1 2]),camC2(2,[1,2]),camC2(3,[1,2]),'r','LineWidth',2);
    plot3(camC2(1,[2 3]),camC2(2,[2,3]),camC2(3,[2,3]),'r','LineWidth',2);
    plot3(camC2(1,[3 4]),camC2(2,[3,4]),camC2(3,[3,4]),'r','LineWidth',2);
    plot3(camC2(1,[4 1]),camC2(2,[4,1]),camC2(3,[4,1]),'r','LineWidth',2);


    %标记相机坐标系
    text(camC1(1,1),camC1(2,1),camC1(3,1),'Cam1','FontSize',12,'Color','b');
    text(camC2(1,1),camC2(2,1),camC2(3,1),'Cam2','FontSize',12,'Color','r');


    %设置相机位置和视角朝向
    campos([43,-22,-87]);
    camup([0,-1,0]);


    %标记坐标轴并激活网络
    xlabel('x');
    ylabel('y');
    zlabel('z');
    grid on;

end

