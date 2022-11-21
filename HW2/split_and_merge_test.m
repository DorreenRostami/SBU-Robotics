clc;
clear;
Points = importdata('data_lidar.txt',',');
Points = [Points(:,1).' ; Points(:,2).'];

%ploting input points
figure;
plt_place = 1;
subplot(2,3,plt_place);hold on
plot(Points(1,:),Points(2,:),'--or');
title('input points');
for epsilon = [0.1 0.3 0.7]
    result = split_and_merge(Points,epsilon);
    plt_place = plt_place + 1;
    subplot(2,2,plt_place);
    hold on
    plot(Points(1,:),Points(2,:),'--or');
    plot(result(1,:),result(2,:),'green');
    title(['\epsilon = ' num2str(epsilon)]);
end
