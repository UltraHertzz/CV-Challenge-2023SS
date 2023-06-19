function [T1,R1,T2,R2,U,V] = TR_from_E_jcp(E)
    %%g该函数用于计算可能的平移和旋转值

    %对本质矩阵进行奇异值分解
    [U,Z,V] = svd(E);

    %确保U和V是旋转矩阵
    if det(U) < 0
        U(:,3) = -U(:,3);
    end
    if det(V) < 0 
        V(:,3) = -V(:,3);
    end

    %计算可能的平移向量和旋转矩阵
    W = [0 -1 0;1 0 0;0 0 1];
    T1 = U(:,3);
    T2 = -U(:,3);
    R1 = U*W'*V';
    R2 = U*W*V';

end