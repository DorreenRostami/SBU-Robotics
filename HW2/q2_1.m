clear;clc;
%r = importdata('golabi_distance.txt', ',');
r = importdata('sonar.txt', ',');
X = [];
Y = [];
% x = r cos  y = r sin
for i = 1:256
   x1 = r(i)*cosd(1.41*i-1.41);
   X = [X, x1];
   y1 = r(i)*sind(1.41*i-1.41);
   Y = [Y, y1];
end

%scattering two walls
figure
scatter(X,Y);
title('Sonar output')

% data for split and merge
Z = [X.',Y.'];
writematrix(Z,'data_sonar.txt','Delimiter', ',');

% getting just one wall
X = X(24:107);
Y = Y(24:107);
Z = [X.',Y.'];
c = ones(size(X.'));
p = pseudo_inverse(Z, c);
disp(p);
writematrix(Z,'data_sonar.dat','Delimiter', ',');

a = p(1,1);
b = p(2,1);
x = 0: 0.01:1;
y = (1/b) - ((a/b)*x);
figure
plot(x,y);
title('The wall with the painting')
axis([0 1 0 1000]);