function [img] = imgwarp(image,template,homography)

% Order: upper left, upper right, bottom right, bottom left
u_page_corners = [1; size(template, 2); size(template,2); 1];
v_page_corners = [1; 1; size(template, 1); size(template, 1)];

minx = floor(min(u_page_corners));
maxx = ceil(max(u_page_corners));
miny = floor(min(v_page_corners));
maxy = ceil(max(v_page_corners));

[x,y] = meshgrid(minx:maxx,miny:maxy);

points = homography\[x(:)';y(:)'; ones(1,size(x(:)',2))];
points = points(1:2,:)./ [points(3,:)' points(3,:)']';

xi=reshape(points(1,:)',size(x,1),length(points(1,:)')/size(x,1));
yi=reshape(points(2,:)',size(y,1),length(points(2,:)')/size(y,1));


I = [];
for i = 1:size(image,3)
    I2=interp2(1:size(image,2), 1:size(image,1),double(image(:,:,i)),xi,yi,'nearest');
    I = cat(3,I,I2);
end


img = uint8(I);

end
