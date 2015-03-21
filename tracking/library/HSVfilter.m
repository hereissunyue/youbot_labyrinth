% Take an RGB image and expected hsv value with 
% threshold in and return an filtered binary image
function [HSV] = HSVfilter(RGB, HSVVal, threshold)
% Extract the size of the RGB image
[M,N,~] = size(RGB);
% Preset the matrix
I1 = zeros(M,N); I2 = zeros(M,N); I3 = zeros(M,N);
% Transfer the image data type from RGB to HSV
HSVimg = rgb2hsv(RGB);
diffH = abs(HSVimg(:,:,1) - HSVVal(1));
diffS = abs(HSVimg(:,:,2) - HSVVal(2));
diffV = abs(HSVimg(:,:,3) - HSVVal(3));
T1 = threshold(1);
T2 = threshold(2);
T3 = threshold(3);
I1( find(diffH < T1) ) = 1; %#ok<*FNDSB>
I2( find(diffS < T2) ) = 1;
I3( find(diffV < T3) ) = 1;
HSV = I1.*I2.*I3;
end