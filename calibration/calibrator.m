function [intrinsics,camExtrinsics,newOrigin] = calibrator()
%%\
addpath('.\images');
images = dir('images\*.jpg');
files = cell(1, length(images));
for j = 1:length(images)
    files{j} = fullfile(which(images(j).name));
end

% Display one of the calibration images
magnification = 25;
I = imread(files{1});
figure; imshow(I, InitialMagnification = magnification);
title("One of the Calibration Images");


%% Estimate Camera Parameters

% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 300; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
                                     ImageSize = imageSize);

intrinsics = cameraParams.Intrinsics;

figure()
showExtrinsics(cameraParams);

% Evaluate calibration accuracy.
figure; showReprojectionErrors(cameraParams);
title("Reprojection Errors");


% %% Read the Image of Objects to Be Measured
imOrig = imread('ref1.jpg');
figure; imshow(imOrig, InitialMagnification = magnification);
title("Input Image");

%% Undistort the Image
% Since the lens introduced little distortion, use 'full' output view to illustrate that
% the image was undistored. If we used the default 'same' option, it would be difficult
% to notice any difference when compared to the original image. Notice the small black borders.
[im, newIntrinsics] = undistortImage(imOrig, intrinsics, OutputView = "full");
figure; imshow(im, InitialMagnification = magnification);
title("Undistorted Image");


%% Map points in image to real cordinates

% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
newOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

count = 1;
for k = 8:-1:1
    for j = 1:5
        worldPoints(count,1) = 300*(k-1);
        worldPoints(count,2) = 300*(j-1);
        count = count + 1;
    end   
end

% Compute extrinsic parameters of the camera.
camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, intrinsics);


% Get the world coordinates of the corners            
worldPoints1 = img2world2d(imagePoints, camExtrinsics, intrinsics);

figure()
plot(worldPoints(:,1),worldPoints(:,2),"bx");
hold on
plot(worldPoints1(:,1),worldPoints1(:,2),"ro");
legend("Ground Truth","Estimates");
hold off
grid on

camPose = extr2pose(camExtrinsics);
figure()
plotCamera(AbsolutePose=camPose,Size=200);
hold on
pcshow([worldPoints,zeros(size(worldPoints,1),1)], ...
  VerticalAxisDir="down",MarkerSize=40);

end
