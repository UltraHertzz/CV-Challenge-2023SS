function sd = sampson_dist_jcp(F,x1_pixel,x2_pixel)
    %%该函数用于计算sampson距离
    
    num =size(x1_pixel,2);
    e3_hat = [0,-1,0;1,0,0;0,0,0];
    numerator = zeros(1,num);
    denominator =zeros(1,num);
    sd = zeros(num,1);

    %计算分子项和分母项
    numerator = (diag((x2_pixel'*F*x1_pixel))').^2;
    denominator = sum((e3_hat*F*x1_pixel).^2)+sum(((x2_pixel'*F*e3_hat).^2)');

    %计算Sampson距离
    sd = numerator./denominator;

end