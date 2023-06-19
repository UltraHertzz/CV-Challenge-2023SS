function [EF] = epa_jcp(correspondences,k)
    
    %将对应的图像点转换为齐次坐标
    x1 = [correspondences(1:2,:);ones(1,size(correspondences,2))];
    x2 = [correspondences(3:4,:);ones(1,size(correspondences,2))];

    if nargin >1 && ~isempty(k)

        x1_s = (k\x1);
        x2_s = (k\x2);
    else
        x1_s = x1;
        x2_s = x2;
    end

    %构造矩阵A
    A=zeros(size(x1_s,2),9);
    for i= 1:size(x1_s,2)
        A(i,:) = (kron(x1_s(:,i),x2_s(:,i)))';
    end

    %对矩阵A进行奇异值分解
    [~,~,V] =svd(A);

    %返回校准后的齐次坐标x1和x2、矩阵A和V
    x1 = x1_s;
    x2 = x2_s;
    
    %根据右奇异向量矩阵V估计本质矩阵/基础矩阵
    if nargin >1 && ~isempty(k)
        %构造本质矩阵E
        G = reshape(V(:,end),3,3);
        [Ug,~,Vg] = svd(G);
        sig1 = zeros(3,3);
        sig1(1,1) = 1;
        sig1(2,2) = 1;
        E = Ug*sig1*Vg';
        EF = E;
    else
        %构造基础矩阵F
        G = reshape(V(:,end),3,3);
        [Ug,sig,Vg] = svd(G);
        sig1 = zeros(3,3);
        sig1(1,1) = sig(1,1);
        sig1(2,2) = sig(2,2);
        F = Ug*sig1*Vg';
        EF = F;
    end

end
