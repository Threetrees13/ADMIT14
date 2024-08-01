%% Read the Image of Objects to Be Measured
imOrig = imread("reference.jpg");
magnification = 25;
% figure; imshow(imOrig, InitialMagnification = magnification);
% title("Input Image");

%% Undistort the Image
% Since the lens introduced little distortion, use 'full' output view to illustrate that
% the image was undistored. If we used the default 'same' option, it would be difficult
% to notice any difference when compared to the original image. Notice the small black borders.
[im, newIntrinsics] = undistortImage(imOrig, intrinsics, OutputView = "full");
% figure; imshow(im, InitialMagnification = magnification);
% title("Undistorted Image");


%% Map points in image to real cordinates

worldPoints = []*10;

controlPoints_img =[313 102;347 100;382 97;418 96;455 96;491 96;311 136; 346 134; 382 133;418 131; 455 130;492 130]; 

% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
newOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
imagePoints = controlPoints_img + newOrigin; % adds newOrigin to every row of imagePoints

% Compute extrinsic parameters of the camera.
camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, intrinsics);

% Get the world coordinates of the corners            
worldPoints1 = img2world2d(imagePoints, camExtrinsics, intrinsics);
 
figure()
plot(worldPoints(:,1),worldPoints(:,2),"bx",'LineWidth',1.5,'MarkerSize',5);
hold on
plot(worldPoints1(:,1),worldPoints1(:,2),"ro",'LineWidth',1.5,'MarkerSize',5);
legend("Ground Truth","Estimates");
hold off
grid on

corr_coeff = corr2(worldPoints1(:,[1,2]),worldPoints);


