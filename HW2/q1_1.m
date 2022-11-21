clear;clc;
r = importdata('lidar.txt', ',');
X = [];
Y = [];
% x = r cos  y = r sin
for i = 1:90
   x1 = r(i)*cosd(i-1);
   X = [X, x1];
   y1 = r(i)*sind(i-1);
   Y = [Y, y1];
end

%alef
%scattering two walls (1:90)
figure
scatter(X,Y);
title('Scatter result with Lidar')
axis([0 1 0 1]);

%dal
% data for split and merge
Z = [X.',Y.'];
writematrix(Z,'data_lidar.txt','Delimiter', ',');

%be
% getting just one wall
X = X(1:42);
Y = Y(1:42);
Z = [X.',Y.'];
c = ones(size(X.'));
p = pseudo_inverse(Z, c);
disp(p);
writematrix(Z,'data_lidar.dat','Delimiter', ',');

a = p(1,1);
b = p(2,1);
x = 0: 0.01:1;
% ax + by = 1
y = (1/b) - ((a/b)*x);
figure
plot(x,y);
title('Pseudo inverse - Lidar')
axis([0 1 0 1]);