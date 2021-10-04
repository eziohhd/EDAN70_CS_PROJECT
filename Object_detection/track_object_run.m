clear;
clc;
%Create a raspi object and a camera board object
rpi = raspi();
cam = cameraboard(rpi, 'Resolution', '640x480');
%record(cam,'myvideo.mp4',60)
figure(1);
for i = 1:1000000
%     [img,bwbw,xm,ym] = track_object(snapshot(cam), 50);
 img = snapshot(cam);

    grayImage=rgb2gray(img);
% image(grayImage)

%Histogram
% imhist(grayImage)

%Turn it into binary object
binaryImage=grayImage<50;
% imshow(binaryImage,[])

%Remove small objects
% binaryImage=imclearborder(binaryImage);
% binaryImage=bwareaopen(binaryImage,10);

%Find black object
[x,y]=find(binaryImage);

%Bound the object in a red rectangular box
 if ~isempty(x) && ~isempty(y)
    xm = round(mean(x));
    ym = round(mean(y));
    xx = max(1, min(x-2)):min(max(x+2), size(binaryImage, 1));
    yy = max(1, min(y-2)):min(max(y+2), size(binaryImage, 2));
%     xx = max(1, xm-5):min(xm+5, size(binaryImage, 1));
%     yy = max(1, ym-5):min(ym+5, size(binaryImage, 2));
    bwbw = zeros(size(img), 'uint8');
    bwbw(xx, yy(1):yy(2)) = 255;
    bwbw(xx, yy(end-1):yy(end)) = 255;
    bwbw(xx(1):xx(2), yy) = 255;
    bwbw(xx(end-1):xx(end), yy) = 255;
 end
    image(img+bwbw);
    drawnow;
    X=sprintf('Position is %d,%d',xm,ym);
    disp(X)
end
