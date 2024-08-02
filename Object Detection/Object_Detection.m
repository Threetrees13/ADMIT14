data = load("gTruth Data");
trainingData = data;

dataDir = "file path"

trainingData.imageFilename = fullfile(dataDir,trainingData.imageFilename);
imds = imageDatastore(pwd,'FileExtensions', {'.png', '.jpg', '.JPEG', '.tif', '.tiff'});
blds = boxLabelDatastore(trainingData(:,2:end));

% for data created by video labeler

 [imds,blds] = objectDetectorTrainingData(gTruth)

%After getting Training Data from Image labeler or video labeler, we need
%to combine imds and blds for detector

ds = combine(imds,blds)

% Larger inputsize require more computational power
inputSize = [416 416 3];

trainingDataForEstimation = transform(ds,@(data)preprocessData(data,inputSize));
%% Estimate Anchor Boxes
numAnchors = 6;
[anchors, meanIoU] = estimateAnchorBoxes(trainingDataForEstimation,numAnchors);
area = anchors(:,1).*anchors(:,2);
[~,idx] = sort(area,"descend");
anchors = anchors(idx,:);
anchorBoxes = {anchors(1:3,:);anchors(4:6,:)};
%% training

classes = ["car"];

detector = yolov4ObjectDetector("tiny-yolov4-coco",classes,anchorBoxes,InputSize=inputSize);

options = trainingOptions("sgdm", ...
    InitialLearnRate=0.0001, ...
    MiniBatchSize=64,...
    MaxEpochs=40, ...
    ResetInputNormalization=false,...
    VerboseFrequency=30,Plots="training-progress");

 trainedDetector = trainYOLOv4ObjectDetector(ds,detector,options);
%% detect in image
 I = imread("01.JPEG");
 [bboxes, scores, labels] = detect(trainedDetector,I,Threshold=0.05);
detectedImg = insertObjectAnnotation(I,"Rectangle",bboxes,labels);
figure
imshow(detectedImg)
%% detect in video

videoFile = 'B4K11-04-154219-154432.mp4';
videoReader = VideoReader(videoFile);

% Create a VideoWriter object to write the output video
outputVideoFile = 'output_video_KF_0613).mp4';
videoWriter = VideoWriter(outputVideoFile, 'MPEG-4');
open(videoWriter);

% Read frames from the video and detect objects in each frame
while hasFrame(videoReader)
    frame = readFrame(videoReader); % Read the current frame

    % Detect objects in the current frame
    [bboxes, scores, labels] = detect(trainedDetector, frame, 'Threshold', 1);

    % Insert bounding boxes and labels into the frame
    detectedImg = insertObjectAnnotation(frame, 'rectangle', bboxes, labels);

    % Write the frame with detections to the output video
    writeVideo(videoWriter, detectedImg);
end

% Close the VideoWriter object
close(videoWriter);


%% real time, for this method I use image acqusition app to get the image from camera

v = videoinput("cam");
figure;
 

 while true

     frame = getsnapshot(v);
 
  
% Detect objects in the current frame
 [bboxes, scores, labels] = detect(trainedDetector, frame, 'Threshold', 0.02);

    % Insert bounding boxes and labels into the frame
  detectedImg = insertObjectAnnotation(frame, 'rectangle', bboxes, labels);
   
  imshow(detectedImg);
% 
 end
% stop(v);