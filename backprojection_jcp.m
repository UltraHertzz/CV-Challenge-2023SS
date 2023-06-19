function [repo_error,x2_repro] = backprojection_jcp(correspondences,P1,Image2,T,R,k)
    P2 = R*P1+T;

    %对齐次坐标进行归一化
    P2_normalized = P2(1:3,:)./P2(3,:);

    %将后投影点投影到图像2中
    x2_repro_homogeneous = k*P2_normalized;
    x2_repro_homogeneous = x2_repro_homogeneous(1:2,:)./x2_repro_homogeneous(3,:);

    %计算后投影均方差
    repo_error = mean(sqrt(sum((correspondences(3:4,:)-x2_repro_homogeneous).^2,1)));

    %在图像2上可视化特征点和后投影点
    figure;
    imshow(Image2);
    hold on
    scatter(correspondences(1,:),correspondences(2,:),'r','filled');
    scatter(x2_repro_homogeneous(1,:),x2_repro_homogeneous(2,:),'g','filled');
    legend('检测到的特征点','后投影点');
    title('后投影点');
    plot(correspondences(1,:),correspondences(2,:),'r');
    plot(x2_repro_homogeneous(1,:),x2_repro_homogeneous(2,:),'g');
    text(correspondences(1,:),correspondences(2,:),num2str((size(correspondences,2))'),'FontSize',8,'Color','r');
    text(x2_repro_homogeneous(1,:),x2_repro_homogeneous(2,:),num2str((1:size(x2_repro_homogeneous,2))'),'FontSize',8,'Color','g');
    hold off

    %返回后投影均方差和后投影点
    x2_repro = [x2_repro_homogeneous;ones(1,size(x2_repro_homogeneous,2))];

end
