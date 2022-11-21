% output from the gaussian filter image
img = im2double(rgb2gray(imread('out.png')));

% masks
x=[-1 -1 -1;0 0 0;1 1 1]/6;
y=[-1 0 1; -1 0 1; -1 0 1]/6;
Gx=abs(conv2(img,y,'same'));
Gy=abs(conv2(img,x,'same'));
G = sqrt( Gx.^2 + Gy.^2);
out = G > 0.04; % Threshold
figure;
imshow(out);
title('Prewitt filter')