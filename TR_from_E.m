function [T1, R1, T2, R2, U, V] = TR_from_E(E)
    % This function calculates the possible values for T and R 
    % from the essential matrix
    %% SVD von der essentiellen Matrix
    [U,~,V] = svd(E);
    

    if det(U) < 0
        U(:,3) = -U(:,3);
    end
    
    if det(V) < 0
        V(:,3) = -V(:,3);
    end

    W = [0 -1 0; 1 0 0; 0 0 1]; 
    Z = [0 1 0; -1 0 0; 0 0 0]; 
    
    S = U*Z*U'; 
    R1 = U*W'*V'; 
    R2 = U*W*V'; 
    
 
    T1 = [S(2,3); S(3,1); S(1,2)]; 
    T2 = -T1; 
    
end
