function [img_rgb] = homography_image_arucos(image, template, image_path, template_path)
%Compute the homography between image and template; return image in
%template space

% Find Aruco marker's coordinates in the image
py.getCorners.run(image_path);
aux = load('cornersIds.mat');
aux_corners = squeeze(aux.corners);
corners_image = permute(aux_corners, [1,3,2]);
ids_image = aux.ids;
clear aux aux_corners

% Find Aruco marker's coordinates in the template
py.getCorners.run(template_path);
aux = load('cornersIds.mat');
aux_corners = squeeze(aux.corners);
corners_template = permute(aux_corners, [1,3,2]);
ids_template = aux.ids;
clear aux aux_corners


% Sort ids and get sorting order; sort corners list according to sorting
% order

[val,pos]=intersect(ids_template,ids_image,'stable');
corners_template=corners_template(pos,:,:);

[ids_image, sortID_image] = sort(ids_image);
corners_image = corners_image(sortID_image, :, :);

[ids_template, sortID_template] = sort(val);
corners_template = corners_template(sortID_template,:,:);

% Get number of points: #arucos * #corners
no_points = size(corners_template,3) * size(corners_template,1);


% Build set of matching points. image and template points are switched because
% we want the inverse of the homography.
u_im = reshape(corners_template(:,1,:), [no_points, 1]);
v_im = reshape(corners_template(:,2,:), [no_points, 1]);
u_temp = reshape(corners_image(:,1,:), [no_points, 1]);
v_temp = reshape(corners_image(:,2,:), [no_points, 1]);

u_page_corners = [1; size(template, 2); size(template,2); 1];
v_page_corners = [1; 1; size(template, 1); size(template, 1)];

% Build Matrices
A=[];b=[];

for i=1:no_points
    
    A = [A; [u_im(i) v_im(i) 1 0 0 0 -u_im(i)*u_temp(i) -v_im(i)*u_temp(i) ];...
        [0 0 0 u_im(i) v_im(i) 1 -u_im(i)*v_temp(i) -v_im(i)*v_temp(i) ]];
    
    b=[b; [u_temp(i); v_temp(i)]];

end

% Get the pseudoinverse
h = [inv(A'*A)*A'*b; 1];

% Reshape to get Hinv
Hinv = reshape(h,[3,3])';
%%
% Get the coordinates of the template page corners
minx = floor(min(u_page_corners));
maxx = ceil(max(u_page_corners));
miny = floor(min(v_page_corners));
maxy = ceil(max(v_page_corners));

% Create a grid with the same size as the template
[x,y] = meshgrid(minx:maxx,miny:maxy);

% Reproject the grid to the image perspective
pp = Hinv*[x(:)'; y(:)'; ones(1,size(x(:)',2))];

% Divide by the third to get the grid's u and v
pp = pp(1:2,:)./ [pp(3,:)' pp(3,:)']';

% Resize the grid to the size of the template
xi=reshape(pp(1,:)',size(x,1),length(pp(1,:)')/size(x,1));
yi=reshape(pp(2,:)',size(y,1),length(pp(2,:)')/size(y,1));

% Find the values of 'image' in the positions of xi and yi
img_rgb = [];
for i = 1:3
    I2=interp2(1:size(image,2), 1:size(image,1),double(image(:,:,i)),xi,yi,'nearest');
    img_rgb = cat(3,img_rgb,I2);
end

% Transform final input format
img_rgb = uint8(img_rgb);


end

