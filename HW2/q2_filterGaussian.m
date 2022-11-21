clear;
clc;
img = imread('img.jpg');
%img = imnoise(img, 'gaussian');
figure
imshow(img);
title('Noisy image')
sigma = 1;
kernel = zeros(5,5);
%sum of elements of kernel
sum = 0;    
for i = 1:5
    for j = 1:5
        kernel(i,j) = exp(-1*((i-3)^2 + (j-3)^2)/(2*sigma^2));
        sum = sum + kernel(i,j);
    end
end

%normalizing kernel
kernel = kernel/sum;
[m,n] = size(img);
%outImg = padarray(img, [2,2]);
outImg = uint8(convn(img, kernel, 'same'));
figure
imshow(outImg);
title('Gaussian filter')
imwrite(outImg, 'out.png');