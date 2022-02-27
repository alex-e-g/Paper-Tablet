function [img,homography] = homography_image_ransac(image,template,u_im,v_im,u_temp,v_temp)
%Compute the homography between image and template; return image in
%template space

nr_points = 4;
errorthresh=4;
n_inliers = 0;

for t = 1:5000
  % estimate homography
  inds = randperm(size(u_temp,1),nr_points);

  source = [ u_im(inds,:)';v_im(inds,:)';ones(1,length(u_im(inds,:)))];
  plane = [u_temp(inds,:)';v_temp(inds,:)';ones(1,length(u_temp(inds,:)))];
    
  A = zeros(2*nr_points,9);
   
  A(1:2:2*nr_points,1:3) = source';
  A(2:2:2*nr_points,4:6) = source';
  A(1:2:2*nr_points,7:9) = -((ones(3,1)*plane(1,:)).* source)';
  A(2:2:2*nr_points,7:9) = -((ones(3,1)*plane(2,:)).* source)';
    
  [~,~,V] = svd(A);
  homography = reshape( V(:,end),[3,3])';
  homography = homography/homography(3,3);
    
  test = homography*[u_im,v_im,ones(length(u_im),1)]';
  test = test./test(3,:);
  test = test(1:2,:);
  error = sqrt(sum(([u_temp v_temp]- test').^2,2));
     
  n_inliers_aux = sum(error<errorthresh);
  if n_inliers_aux > n_inliers
      n_inliers = n_inliers_aux;
      ind = find(error<errorthresh);
  end
         
end
source = [u_im(ind)';v_im(ind)';ones(1,length(u_im(ind)))];
plane = [u_temp(ind)';v_temp(ind)';ones(1,length(u_temp(ind)))];
nr_points = n_inliers;
A = zeros(2*nr_points,9);

A(1:2:2*nr_points,1:3) = source';
A(2:2:2*nr_points,4:6) = source';
A(1:2:2*nr_points,7:9) = -((ones(3,1)*plane(1,:)).* source)';
A(2:2:2*nr_points,7:9) = -((ones(3,1)*plane(2,:)).* source)';

[~,~,V] = svd(A);
homography = reshape(V(:,end),[3,3])';
homography = homography/homography(3,3);

img = imgwarp(image,template,homography);

end

