function [maskedRgbImage,skinPixels] = skin_segmentation(rgbImage)

% clean up with a median filter.
rgbImage(:,:,1) = medfilt2(rgbImage(:,:,1), [3 3]);
rgbImage(:,:,2) = medfilt2(rgbImage(:,:,2), [3 3]);
rgbImage(:,:,3) = medfilt2(rgbImage(:,:,3), [3 3]);

%blur the S and H channels to make the colorsmore continuous and not as quantized.
hsv = rgb2hsv(rgbImage);
% Get separate channels.
h = hsv(:, :, 1);
s = hsv(:, :, 2);
v = hsv(:, :, 3);
% Blur h and v channels.  Don't blur v channel so the image doesn't look blurred.
s = conv2(s, ones(3)/9, 'same');
h = conv2(h, ones(3)/9, 'same');
% Recombine.
hsv = cat(3, h, s, v);
rgbImage = hsv2rgb(hsv);

% Convert to hsv color space.
hsv = rgb2hsv(rgbImage);
h = hsv(:, :, 1);
s = hsv(:, :, 2);
v = hsv(:, :, 3);


% Try to get a binary image of skin.
hBinary = h < 0.08;
sBinary = s > 0.20;
vBinary = v > 0.37;
skinPixels = hBinary & sBinary & vBinary;
skinPixels=imerode(skinPixels,strel('disk',20));
skinPixels=imerode(skinPixels,strel('disk',20));

skinPixels=imdilate(skinPixels,strel('disk',40));
skinPixels=imdilate(skinPixels,strel('disk',40));
skinPixels=imdilate(skinPixels,strel('disk',40));
skinPixels=imdilate(skinPixels,strel('disk',40));
% Mask the image.
maskedRgbImage = bsxfun(@times, rgbImage, cast(~skinPixels,class(rgbImage)));
