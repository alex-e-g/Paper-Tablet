function [] = pivproject2021(task, path_to_template ,path_to_output_folder , arg1, arg2)
%% Compute the homography between images in a directory and a template

%   if task = 1,2,3 , arg1 is the path to the image input folder (ex:
%   /home/jpc/piv/rgbcamera ) where input images are stored. Images are named
%   rgb_number.jpg (or rgb_number.png) and depth images are named
%   depth_number.png (depth images will be in png format). For these cases
%   arg2 does not exist. if task=4, arg1 is the path to images of camera 1
%   and arg2 the path to images of camera 2 (all rgb images).

%   path_to_template:      string with the path to the template image
%   path_to_output_folder: string with the path where output images are stored

% Check if arg2 has been passed; assign value if not
if ~exist('arg2','var')
      arg2 = "No_path";
end

% Check if output directory exists; if not, create it
if not(isfolder(path_to_output_folder))
    mkdir(path_to_output_folder)
% else
%     % delete outputs from previous tests and create the folder
%     rmdir(path_to_output_folder, 's');
%     mkdir(path_to_output_folder)
end

addpath(arg1);

% Set up python in matlab
% try
%     pyversion('C:\Users\Catarina\anaconda3\python.exe');  % Change to your own python directory
% catch
%     fprintf('Python already loaded.\n');
% end

%% First task:
% receive a folder with a set of images and generate the corresponding
% corrected images, saved to the output folder). The template and the input
% set of images contain Aruco markers.

if task == 1

    % Get input rgb images from arg1
    try
        disp('Looking for jpg images...');
        rgb1_imds = imageDatastore(fullfile(arg1),'FileExtensions',".jpg");
    catch
        try
            disp('Looking for png images...');
            rgb1_imds = imageDatastore(fullfile(arg1),'FileExtensions',".png");

        catch
            fprintf('ERROR: Image input files do not exist in this path.');
            return
        end
        
    end
            
    % Get template image
    try
        template = imread(path_to_template);
    catch
        fprintf('ERROR: Template image does not exist in this path.')
        return
    end
    
    disp('Calculating projections...');
   
    %figure;
    for i = 1:size(rgb1_imds.Files, 1)
        
        input_image = imread(rgb1_imds.Files{i});
        image_path = char(rgb1_imds.Files(i));
        
        
        % Project this frame onto template space
        try
            projection = homography_image_arucos(input_image, template, image_path, path_to_template);
        catch
            disp('ERROR: Check if python is setup in MATLAB and format of directory (OpenCV cannot read directories with spaces or special characters).');
            return
        end
        [~,name,~] = fileparts(rgb1_imds.Files(i));
        output_path = strcat(path_to_output_folder,'\', name, ".png");
        imwrite(projection, output_path);
    end
    
    disp('All projections calculated.');
    
end
%% Task 2:
% 

if task == 2
    try
        disp('Looking for jpg images...');
        rgb1_imds = imageDatastore(fullfile(arg1),'FileExtensions',".jpg");
    catch
        try
            disp('Looking for png images...');
            rgb1_imds = imageDatastore(fullfile(arg1),'FileExtensions',".png");
        catch
            fprintf('ERROR: Image input files do not exist in this path.');
            return
        end
        
    end
            
    % Get template image
    try
        template = imread(path_to_template);
    catch
        fprintf('ERROR: Template image does not exist in this path.')
        return
    end
    
    disp('Calculating projections...');
    
last_good = 0;
no_skin = 0;
for i = 1:size(rgb1_imds.Files, 1)

    image = imread(rgb1_imds.Files{i});
    image_path = char(rgb1_imds.Files(i));
    try
        py.siftMatch.run(image_path,path_to_template)

        aux = load("matched_points.mat");
        u_im = aux.src(:,1);
        v_im = aux.src(:,2);
        u_temp = aux.dst(:,1);
        v_temp = aux.dst(:,2);
    catch
        disp('ERROR py.siftMatch: Check if python is setup in MATLAB and format of directory (OpenCV cannot read directories with spaces or special characters).');
        return
    end
    
    try
        [I,h] = homography_image_ransac(image,template,u_im,v_im,u_temp,v_temp);

            if det(h) <= 0.1  || cond(h(1:2,1:2)) >= 2.2
                imwrite(I,"projection.jpg");
                py.siftMatch.run("projection.jpg",path_to_template);
                aux = load("matched_points.mat");

                u_im2 = aux.src(:,1);
                v_im2 = aux.src(:,2);
                u_temp2 = aux.dst(:,1);
                v_temp2 = aux.dst(:,2);

                [~,h1] = homography_image_ransac(I,template,u_im2,v_im2,u_temp2,v_temp2);
                
                I = imgwarp(image,template,h1*h);
                H = h1*h;
                if cond(H(1:2,1:2)) < 3 && det(H) > 0.1
                    last_good = i;
                    last_h = H;
                elseif last_good~=0
                    h = last_h;
                    image2_path = char(rgb1_imds.Files(last_good));
                    image2 = imread(image2_path);
                    py.siftMatch.run(image_path,image2_path)

                    aux = load("matched_points.mat");
                    u_im2 = aux.src(:,1);
                    v_im2 = aux.src(:,2);
                    u_temp2 = aux.dst(:,1);
                    v_temp2 = aux.dst(:,2);  
                    [~,h2] = homography_image_ransac(image,image2,u_im2,v_im2,u_temp2,v_temp2);
                    I = imgwarp(image,template,h*h2);
                    H = h*h2;
                    if cond(H(1:2,1:2)) < 3 && det(H) > 0.1
                        last_good = i;
                        last_h = H;
                    end

                end
            else
                last_good = i;
                last_h = h;
            end
            if last_good == i
                [segmented,mask] = skin_segmentation(I);
                if sum(mask,'all') > 0 && no_skin ~=0
                    image2_path = char(rgb1_imds.Files(no_skin));
                    image2 = imread(image2_path);
                    I2 = imgwarp(image2,template,h_no_skin);
                    maskedI = bsxfun(@times, I2, cast(mask,class(I2)));

                    I = im2uint8(segmented) + maskedI;
                elseif sum(mask,'all') == 0
                    no_skin = last_good;
                    h_no_skin = last_h;
                end
                projection = I;
            else
                projection = zeros(size(I));
            end
            [~,name,~] = fileparts(rgb1_imds.Files(i));
            output_path = strcat(path_to_output_folder,'\', name, ".png");
            imwrite(projection, output_path);
    
    catch
        fprintf('ERROR: Could not compute homography. Check if there are enough matching keypoints.');
    end
   

end

disp('All projections calculated.');

end



%% Task 3:
% 

if task == 3
    
end

%% Task 4:
% 

if task == 4
    % Get camera images
    try
        disp('Looking for jpg images...');
        rgb1_imds = imageDatastore(fullfile(arg1),'FileExtensions',".jpg");
        rgb2_imds = imageDatastore(fullfile(arg2),'FileExtensions',".jpg");
    catch
        try
            disp('Looking for png images...');
            rgb1_imds = imageDatastore(fullfile(arg1),'FileExtensions',".png");
            rgb2_imds = imageDatastore(fullfile(arg2),'FileExtensions',".png");
        catch
            fprintf('ERROR: Image input files do not exist in this path.');
            return
        end

    end

    % Get template image
    try
        template = imread(path_to_template);
    catch
        fprintf('ERROR: Template image does not exist in this path.')
        return
    end

    % Initialise variables
    disp('Calculating projections...');
    last_good_1 = 0;      % previous good projection for image 1
    last_good_2 = 0;      % previous good projection for image 2
    no_skin1 = 0;         % previous frame where there was no skin (img 1)
    no_skin2 = 0;         % previous frame where there was no skin (img 2)
    previous_image = 0;   % previous reconstructed image 
    bool1 = false;        % boolean variable to qualify homography of image 1
    bool2 = false;        % boolean variable to qualify homography of image 2

    % Process each image i
    for i = 1:size(rgb1_imds.Files, 1)
        % Read both images
        image1 = imread(rgb1_imds.Files{i});
        image2 = imread(rgb2_imds.Files{i});
        image_path1 = char(rgb1_imds.Files(i));
        image_path2 = char(rgb2_imds.Files(i));

        % Matching points Image 1 - Template
        try
            py.siftMatch.run(image_path1, path_to_template)
            aux = load("matched_points.mat");
            u_im1 = aux.src(:,1);
            v_im1 = aux.src(:,2);
            u_temp1 = aux.dst(:,1);
            v_temp1 = aux.dst(:,2);
        catch
            disp('ERROR py.siftMatch: Check if python is setup in MATLAB and format of directory (OpenCV cannot read directories with spaces or special characters).');
            return
        end

        % Matching points Image 2 - Template
        py.siftMatch.run(image_path2, path_to_template)
        aux = load("matched_points.mat");
        u_im2 = aux.src(:,1);
        v_im2 = aux.src(:,2);
        u_temp2 = aux.dst(:,1);
        v_temp2 = aux.dst(:,2);


       try
            % Process Image 1 -------------------------------------------
            % Homography Image 1 -> Template (gives Projection 1) [h1]
            [I_1,h1] = homography_image_ransac(image1, template, u_im1, v_im1, u_temp1, v_temp1);

                % If this homography is bad
                if det(h1) <= 0.1  || cond(h1(1:2,1:2)) >= 2.2

                    % Find Homography Pojection 1 -> Template [hp1]
                    imwrite(I_1,"projection.jpg");
                    py.siftMatch.run("projection.jpg", path_to_template);
                    aux = load("matched_points.mat");
                    u_im_p1 = aux.src(:,1);
                    v_im_p1 = aux.src(:,2);
                    u_temp_p1 = aux.dst(:,1);
                    v_temp_p1 = aux.dst(:,2);

                    [~,hp1] = homography_image_ransac(I_1,template,u_im_p1,v_im_p1,u_temp_p1,v_temp_p1);

                    I_1 = imgwarp(image1,template,hp1*h1);

                    % Better Homography Image 1 -> Template [H_p1]
                    H_p1 = hp1*h1;

                    % If this homography is good
                    if cond(H_p1(1:2,1:2)) < 3 && det(H_p1) > 0.1
                        last_good_1 = i; % store as the most recent good image
                        last_h1 = H_p1;   % store as the most recent good homography
                        bool1 = true;

                    % Else use previous good projection (if it exists)
                    elseif last_good_1~=0
                        h1 = last_h1;
                        image_last_path_1 = char(rgb1_imds.Files(last_good_1));
                        image_last_1 = imread(image_last_path_1);

                        % Homography Image 1 -> Previous good [h_last1]
                        py.siftMatch.run(image_path1, image_last_path_1)
                        aux = load("matched_points.mat");
                        u_im_last = aux.src(:,1);
                        v_im_last = aux.src(:,2);
                        u_temp_last = aux.dst(:,1);
                        v_temp_last = aux.dst(:,2);  
                        [~,h_last1] = homography_image_ransac(image1,image_last_1,u_im_last,v_im_last,u_temp_last,v_temp_last);
                        I_1 = imgwarp(image_last_1,template,h1*h_last1);

                        % Better Homography Image 1 -> Template
                        H = h1*h_last1;

                        % If this homography is good, save it
                        if cond(H(1:2,1:2)) < 3 && det(H) > 0.1
                            last_good_1 = i;
                            last_h1 = H;
                            bool1 = true;
                        end

                    end

                % Else if first homography was already good
                else
                    last_good_1 = i;
                    last_h1 = h1;
                    bool1 = true;
                end

                % If this projection is good
                if last_good_1 == i

                    % Try to remove skin
                      [segmented,skinmask1] = skin_segmentation(I_1);
                    
                    % If there is skin now and there wasn't before
                    if sum(skinmask1,'all') > 0 && no_skin1 ~=0
                        image_last_path_1 = char(rgb1_imds.Files(no_skin1));
                        image_last_1 = imread(image_last_path_1);
                        Iskinmask = imgwarp(image_last_1,template,h_no_skin1);
                        maskedI1 = bsxfun(@times, Iskinmask, cast(skinmask1,class(Iskinmask)));
                        I_1 = im2uint8(segmented) + maskedI1;
                    
                    % If there is no skin now
                    elseif sum(skinmask1,'all') == 0
                        no_skin1 = last_good_1;
                        h_no_skin1 = last_h1;
                    end

                    projection_1 = I_1;
                else
                    projection_1 = zeros(size(I_1));
                end

            % Process Image 2 -------------------------------------------
            % Homography Image 2 -> Template (gives Projection 2) [h2]
            [I_2,h2] = homography_image_ransac(image2, template, u_im2, v_im2, u_temp2, v_temp2);


                % If this homography is bad
                if det(h2) <= 0.1  || cond(h2(1:2,1:2)) >= 2.2

                    % Find Homography Pojection 2 -> Template [hp2]
                    imwrite(I_2,"projection.jpg");
                    try
                        py.siftMatch.run("projection.jpg", path_to_template);
                        aux = load("matched_points.mat");
                        u_im_p2 = aux.src(:,1);
                        v_im_p2 = aux.src(:,2);
                        u_temp_p2 = aux.dst(:,1);
                        v_temp_p2 = aux.dst(:,2);
                    catch
                        disp('ERROR py.siftMatch: Check if python is setup in MATLAB and format of directory (OpenCV cannot read directories with spaces or special characters).');
                        return
                    end
                    [~,hp2] = homography_image_ransac(I_2,template,u_im_p2,v_im_p2,u_temp_p2,v_temp_p2);

                    I_2 = imgwarp(image2,template,hp2*h2);

                    % Better Homography Image 2 -> Template [H_p2]
                    H_p2 = hp2*h2;

                    % If this homography is good
                    if cond(H_p2(1:2,1:2)) < 3 && det(H_p2) > 0.1
                        last_good_2 = i;    % store as the most recent good image
                        last_h2 = H_p2;      % store as the most recent good homography
                        bool2 = true;

                    % Else use previous good homography (if it exists)
                    elseif last_good_2~=0
                        h2 = last_h2;
                        image_last_path_2 = char(rgb2_imds.Files(last_good_2));
                        image_last_2 = imread(image_last_path_2);

                        % Homography Image 2 -> Previous good [h_last2]
                        py.siftMatch.run(image_path2, image_last_path_2)
                        aux = load("matched_points.mat");
                        u_im_last = aux.src(:,1);
                        v_im_last = aux.src(:,2);
                        u_temp_last = aux.dst(:,1);
                        v_temp_last = aux.dst(:,2);  
                        [~,h_last2] = homography_image_ransac(image2,image_last_2,u_im_last,v_im_last,u_temp_last,v_temp_last);
                        I_2 = imgwarp(image_last_2,template,h2*h_last2);

                        % Better Homography Image 2 -> Template [H]
                        H = h2*h_last2;

                        % If this homography is good, save it
                        if cond(H(1:2,1:2)) < 3 && det(H) > 0.1
                            last_good_2 = i;
                            last_h2 = H;
                            bool2 = true;
                        end

                    end

                % Else if the first homography is good
                else
                    last_good_2 = i;
                    last_h2 = h2;
                    bool2 = true;
                end

                % If this projection is good
                if last_good_2 == i

                    % Try to remove skin
                      [segmented,skinmask2] = skin_segmentation(I_2);                  
                    
                    % If there is skin now and there wasn't before
                    if sum(skinmask2,'all') > 0 && no_skin2 ~=0
                        
                        image_last_path_2 = char(rgb2_imds.Files(no_skin2));
                        image_last_2 = imread(image_last_path_2);
                        Iskinmask = imgwarp(image_last_2,template,h_no_skin2);
                        maskedI2 = bsxfun(@times, Iskinmask, cast(skinmask2,class(Iskinmask)));
                        I_2 = im2uint8(segmented) + maskedI2;
                    
                    % If there is no skin now
                    elseif sum(skinmask2,'all') == 0
                        no_skin2 = last_good_2;
                        h_no_skin2 = last_h2;
                    end
                    projection_2 = I_2;
                else
                    projection_2 = zeros(size(I_2));
                end


                % Combine both images
                if bool1 && bool2

                    % Mask area where these 2 pictures overlap 
                    mask1 = imfill(im2gray(projection_1) ~= 0, 'holes');
                    mask2 = imfill(im2gray(projection_2) ~=0, 'holes');
                    overlapMask = mask1 & mask2;

                    % Adjust hsv histogram of image 1 to match image 2
%                     if bwarea(mask1) > bwarea(mask2)
%                         projection_2 = match_histograms_hsv(projection_2, projection_1);
%                     else
%                         projection_1 = match_histograms_hsv(projection_1, projection_2);
%                     end

                    
                    % Overlapping pixels:
                    % If 1 doesnt have the hand but 2 does, use 1
                    if no_skin1 == i && no_skin2 ~=i
                        masked_projection1 = bsxfun(@times, projection_1, cast(mask1,class(projection_1)));
                        masked_projection2 = bsxfun(@times, projection_2, cast(~overlapMask,class(projection_2)));
                        projection = masked_projection1 + masked_projection2;
                    % If 2 doesnt have the hand but 1 does, use 2
                    elseif no_skin1~= i && no_skin2 == i
                        masked_projection1 = bsxfun(@times, projection_1, cast(~overlapMask,class(projection_1)));
                        masked_projection2 = bsxfun(@times, projection_2, cast(mask2,class(projection_2)));
                        projection = masked_projection1 + masked_projection2;
                    % Other cases, use 2
                    else
                        masked_projection1 = bsxfun(@times, projection_1, cast(~overlapMask,class(projection_1)));
                        masked_projection2 = bsxfun(@times, projection_2, cast(mask2,class(projection_2)));
                        projection = masked_projection1 + masked_projection2;
                    end
                    

                % If only 1 is good
                elseif bool1             
                    projection = projection_1;
                % If only 2 is good
                elseif bool2            
                    projection = projection_2;
                % If neither is good
                elseif length(previous_image) > 1              
                    projection = previous_image;
                end


                % If there are empty spots, use previous image to fill them
                if i~=1 && length(previous_image) > 1
                    mask = imfill(im2gray(projection) ~= 0, 'holes');
                    mask_previous = imfill(im2gray(previous_image) ~=0, 'holes');
                    filling_mask = ~mask & mask_previous;  % where projection is empty and previous_image is not
%                     filling_mask = imopen(filling_mask, strel('disk', 5)); % avoid replacing isolated black pixels
                    masked_projection = bsxfun(@times, projection, cast(~filling_mask,class(projection)));
                    masked_projection2 = bsxfun(@times, previous_image, cast(filling_mask,class(previous_image)));
                    projection = masked_projection + masked_projection2;
                end

                % Save result
                [~,name,~] = fileparts(rgb1_imds.Files(i));
                output_path = strcat(path_to_output_folder,'\', name, ".png");
                imwrite(projection, output_path); 

                % Update variables
                if bool1 && bool2 && no_skin1 == i && no_skin2 ==i
                    previous_image = projection;
                end
                bool1 = false;
                bool2 = false;

        catch
            fprintf('ERROR: Could not compute homography. Check if there are enough matching keypoints.');
        end
    
    end

%%
disp('Done.')

end

